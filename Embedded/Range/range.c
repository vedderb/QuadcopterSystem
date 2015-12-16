/*
	Copyright 2013-2015 Benjamin Vedder benjamin@vedder.se
	Copyright 2013-2014 Daniel Skarin	daniel.skarin@sp.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "webserver-nogui.h"
#include <string.h>
#include <stdlib.h>
#include "uip.h"
#include "stm32f4xx_conf.h"
#include <math.h>

#include "range.h"
#include "signal.h"
#include "udp_commands.h"
#include "cc2520-arch-sfd.h"

#include "ultrasonic_serial.h"
#include "packet.h"
#include "buffer.h"
#include "usartinterface.h"
#include "comm.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define MAX_PAYLOAD_LEN 120

static uip_ipaddr_t client_addr;
static struct uip_udp_conn *server_conn, *client_conn, *clock_conn;
static float drift_corr = 0.0;
static float drift_acc = 0.0;
static uint32_t prev_clock_update_time = 0;

static int32_t clock_error = 0, clock_max_error = 0, clock_status = NOT_SYNCHRONIZED;
static int32_t clock_incorrect_msgs = 0;

static LEVEL_OF_SERVICE_t los_rec = LEVEL_OF_SERVICE_HIGH;

PROCESS(udp_server_process, "UDP server process");
PROCESS(comp_process, "Computation process");
PROCESS(clock_send_process, "Clock send process");
PROCESS(srf10_process, "SRF10 process");
PROCESS(safety_process, "Safety process");

AUTOSTART_PROCESSES(&udp_server_process, &webserver_nogui_process, &corr_process,
		&comp_process, &clock_send_process, &srf10_process, &safety_process);

#define SLOT_PERIOD 	50000 // Microseconds
volatile uint32_t next_timer = SLOT_PERIOD;
#define N_SLOTS			(ANCHOR_NUM + 2)

#define HEIGHT_SLOT1		2
#define HEIGHT_SLOT2		5
#if NODE_ID == 2
  #define ANCHOR_NODE		1
  #define MY_SLOT			0
#elif NODE_ID == 3
  #define ANCHOR_NODE		1
  #define MY_SLOT			1
#elif NODE_ID == 4
  #define ANCHOR_NODE		1
  #define MY_SLOT			3
#elif NODE_ID == 5
  #define ANCHOR_NODE		1
  #define MY_SLOT			4
#else
  #define ANCHOR_NODE		0
  #define MY_SLOT			-1 // No send slot
#endif

// Global variables
volatile uint32_t current_slot = 0;

// Private variables
volatile static uint32_t last_slot = 0;
volatile static uint32_t last_adc_buffer_num = 0;
volatile static CMD_CLIENT_t last_cmd = CMD_CLIENT_SEND_PULSE_SAMPLE_ONLYDIST;
volatile static uint32_t last_cmd_receiver = 0;
volatile static int send_clock_now = 0;
volatile static int measure_height_now = 0;

PROCESS_THREAD(clock_send_process, ev, data)
{
	static struct etimer timer;

	PROCESS_BEGIN();

	etimer_set(&timer, CLOCK_CONF_SECOND / 1000);

	for (;;) {
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

		// Only send the clock if the transmission can be started right away.
		if (send_clock_now) {
			if (!comm_is_tx_busy()) {
				send_clock_now = 0;
				int32_t i = 0;
				uint8_t buf[20];
				buffer_append_uint32(buf, TIM5->CNT, &i);
				comm_sendmsg(PACKET_INT_CMD_CLOCK, buf, i);
			}
		}

		etimer_reset(&timer);
	}

	PROCESS_END();
	return 0;
}

PROCESS_THREAD(safety_process, ev, data)
{
	static struct etimer timer;

	PROCESS_BEGIN();

	etimer_set(&timer, CLOCK_CONF_SECOND / 50);

	for (;;) {
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

		uint8_t buf[1];
		buf[0] = los_rec;
		comm_sendmsg(PACKET_INT_CMD_LOS, buf, 1);

		etimer_reset(&timer);
	}

	PROCESS_END();
	return 0;
}

PROCESS_THREAD(comp_process, ev, data) {
	PROCESS_BEGIN();

	for(;;) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		if (last_cmd_receiver == last_slot) {
			signal_compute(last_slot, last_adc_buffer_num, last_cmd);
			last_cmd = CMD_CLIENT_SEND_PULSE_SAMPLE_ONLYDIST;
		} else {
			signal_compute(last_slot, last_adc_buffer_num, CMD_CLIENT_SEND_PULSE_SAMPLE_ONLYDIST);
		}
	}

	PROCESS_END();
	return 0;
}

PROCESS_THREAD(srf10_process, ev, data) {
	PROCESS_BEGIN();
	static int range_done = 0;

	for(;;) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		if (measure_height_now) {
			measure_height_now = 0;
			srf10_start_range_handler();
			range_done = 1;
		} else {
			if (range_done) {
				srf10_get_range_handler();
				range_done = 0;
			}
		}
	}

	PROCESS_END();
	return 0;
}

static void range_handler(void) {
	uint32_t current_timer = next_timer;

	uint32_t next_timer_tmp = next_timer;
	next_timer_tmp += SLOT_PERIOD;
	if (next_timer_tmp > next_timer) {
		next_timer += SLOT_PERIOD;
	} else {
		next_timer = SLOT_PERIOD;
	}

	// Send clock
	send_clock_now = 1;

	static uint32_t last_sample_slot = 0;

	TIM_SetCompare1(TIM5, next_timer);
	current_slot = (current_timer % (N_SLOTS * SLOT_PERIOD)) / SLOT_PERIOD;

	if (ANCHOR_NODE) {
		if (current_slot == MY_SLOT && clock_status == SYNCHRONIZED) {
			signal_generate_pulse();
		}
	} else {
		if (current_slot == HEIGHT_SLOT1 || current_slot == HEIGHT_SLOT2) {
			GPIO_SetBits(GPIOC, GPIO_Pin_12);
			measure_height_now = 1;
			process_poll(&srf10_process);
		} else {
			GPIO_ResetBits(GPIOC, GPIO_Pin_12);
			signal_start_sampling(last_adc_buffer_num);
			process_poll(&srf10_process);

			if (last_adc_buffer_num == 0) {
				last_adc_buffer_num = 1;
			} else {
				last_adc_buffer_num = 0;
			}

			last_slot = last_sample_slot;
			last_sample_slot = current_slot;

			process_poll(&comp_process);
		}
	}

	// Clock drift correction
	drift_acc += drift_corr * ((float)SLOT_PERIOD / 1000.0);
	int32_t corr_now = (int32_t)roundf(drift_acc);
	TIM5->CNT -= corr_now;
	drift_acc -= corr_now;
}

void TIM5_IRQHandler(void) {
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1 ) != RESET) {
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
		range_handler();
	}
}

void timer_init() {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	PrescalerValue = (uint16_t) ((168000000 / 2) / 1000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// Output compare
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = SLOT_PERIOD;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable);

	// Prescaler configuration
	TIM_PrescalerConfig(TIM5, PrescalerValue, TIM_PSCReloadMode_Immediate);

	// Disable ARR buffering
	TIM_ARRPreloadConfig(TIM5, DISABLE);

	// Enable TIM5 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM5, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);

	TIM5->CNT = 0;
}


static void tcpip_handler() {
	char buf[MAX_PAYLOAD_LEN];
	if (uip_newdata()) {
		unsigned char *data = (unsigned char *)uip_appdata;
		data[uip_datalen()] = 0;
		uint8_t cmd = data[0];
		uint8_t receiver = data[1];
		LEVEL_OF_SERVICE_t los = data[1];

		//uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
		uip_ip6addr(&server_conn->ripaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
		uip_ipaddr_copy(&client_conn->ripaddr, &client_addr);

		// Send back command
		buf[0] = CMD_SERVER_TEXT;
		sprintf(buf+1, "Command: 0x%X\n"
				"Clock: %lu, Clock error: %li, Max error: %li",
				cmd, TIM5->CNT/1000, clock_error, clock_max_error);
		uip_udp_packet_send(server_conn, buf, strlen(buf+1) + 1);

		switch(cmd) {
		case CMD_CLIENT_SEND_PULSE_SAMPLE:
		case CMD_CLIENT_SEND_PULSE_SAMPLE_ONLYDIST:
		case CMD_CLIENT_SEND_PULSE_SAMPLE_ALLDATA:
			last_cmd_receiver = receiver;
			last_cmd = cmd;
			break;

		case CMD_CLIENT_SET_LOS:
			los_rec = los;
			break;

		default:
			break;
		}

		// Restore server connection to allow data from any node
		memset(&server_conn->ripaddr, 0, sizeof(server_conn->ripaddr));
	}
}



PROCESS_THREAD(udp_server_process, ev, data) {
#if UIP_CONF_ROUTER
	uip_ipaddr_t ipaddr;
#endif /* UIP_CONF_ROUTER */

	PROCESS_BEGIN();
	PRINTF("UDP server started\r\n");

#if UIP_CONF_ROUTER
	uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
	uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
#endif /* UIP_CONF_ROUTER */

	server_conn = udp_new(NULL, UIP_HTONS(3001), NULL );
	udp_bind(server_conn, UIP_HTONS(3000));

	clock_conn = udp_new(NULL, UIP_HTONS(CLOCK_UDP_PORT+1), NULL);
	udp_bind(clock_conn, UIP_HTONS(CLOCK_UDP_PORT));


#if NODE_ID == 7
	uip_ip6addr(&client_addr, 0xfe80, 0, 0, 0, 0x200, 0, 0, 0x8);
#else
	uip_ip6addr(&client_addr, 0xfe80, 0, 0, 0, 0x200, 0, 0, 0x7);
#endif

	client_conn = udp_new(&client_addr, UIP_HTONS(3000), NULL );
	udp_bind(client_conn, UIP_HTONS(3001));

	comm_init();
	timer_init();
	signal_init();
	cc2520_arch_sfd_init();
	ultrasonic_init();

	for(;;) {
		PROCESS_YIELD();
		if (ev == tcpip_event) {
			if (uip_udp_conn  == server_conn)
				tcpip_handler();
			else if (uip_udp_conn == clock_conn) {
				leds_toggle(LEDS_RED);
				// Update counter for Timer 5
				uint32_t local_clock = TIM5->CNT;
				uint32_t master_clock = uip_ntohl(*((uint32_t *) uip_appdata));
				clock_error = sfd_timestamp - master_clock;
				if (clock_status == NOT_SYNCHRONIZED) {
					local_clock -= clock_error;
					TIM5->CNT = local_clock;
					next_timer = local_clock + 2*SLOT_PERIOD;
					next_timer -= next_timer % SLOT_PERIOD;
					TIM_SetCompare1(TIM5, next_timer);
					clock_status = SYNCHRONIZED;
					clock_error = 0;
					clock_incorrect_msgs = 0;
				} else if ( (abs(clock_error) < MAX_CLOCK_OFFSET) ) {
					local_clock -= clock_error;
					drift_corr += (float)clock_error / ((float)(local_clock - prev_clock_update_time) / 1000.0);
					prev_clock_update_time = local_clock;
					TIM5->CNT = local_clock;
					if (local_clock >= next_timer) {
						range_handler();
					}
					clock_incorrect_msgs = 0;
					if (abs(clock_error) > abs(clock_max_error)) {
						clock_max_error = clock_error;
					}
				} else {
					clock_incorrect_msgs++;
					if (clock_incorrect_msgs >= 2) {
						clock_status = NOT_SYNCHRONIZED;
						drift_corr = 0.0;
						TIM5->CNT = 0;
					}
				}
			}
		}
	}

	PROCESS_END();
	return 0;
}

