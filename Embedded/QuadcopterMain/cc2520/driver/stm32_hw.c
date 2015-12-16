/***********************************************************************************
 * INCLUDES
 */
#include "stm32_hw.h"


/***********************************************************************************
 * GLOBAL VARIABLES
 */


/***********************************************************************************
 * PRIVATE VARIABLES
 */
static const SPIConfig spicfg = {
		NULL,
		CC2520_SPI_CSN_PORT,
		CC2520_SPI_CSN_PIN,
		SPI_CR1_BR_1 // 5.25 MHz
};


/***********************************************************************************
 * FUNCTIONS
 */
static void halRadioSpiInit(void);
static void halMcuRfInterfaceInit(void);

/***********************************************************************************
 * @fn          halRadioSpiInit
 *
 * @brief       Initalise Radio SPI interface
 *
 * @param       none
 *
 * @return      none
 */
static void halRadioSpiInit(void)
{
	/*
	 * Initializes the SPI driver 2. The SPI2 signals are routed as follow:
	 * PB12 - NSS.
	 * PB13 - SCK.
	 * PB14 - MISO.
	 * PB15 - MOSI.
	 *
	 * TODO: Initialize cc2520
	 */
	palSetPad(CC2520_SPI_CSN_PORT, CC2520_SPI_CSN_PIN);
	palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);           /* NSS.     */
	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_SPI) |
			PAL_STM32_OSPEED_HIGHEST);           /* SCK.     */
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_SPI));              /* MISO.    */
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_SPI) |
			PAL_STM32_OSPEED_HIGHEST);           /* MOSI.    */

	spiStart(&CC2520_SPI, &spicfg);
}


/***********************************************************************************
 * @fn      halMcuRfInterfaceInit
 *
 * @brief   Initialises SPI interface to CC2520 and configures reset and vreg
 *          signals as MCU outputs.
 *
 * @param   none
 *
 * @return  none
 */
static void halMcuRfInterfaceInit(void)
{
	// Initialize the CC2520 interface
	CC2520_RESET_OPIN(0);
	CC2520_BASIC_IO_DIR_INIT();
}


/***********************************************************************************
 * @fn      halAssyInit
 *
 * @brief   Initialize interfaces between radio and MCU
 *
 * @param   none
 *
 * @return  none
 */
void halAssyInit(void)
{
	CC2520_GPIO0_DIR_OUT();
	CC2520_GPIO1_DIR_OUT();
	CC2520_GPIO2_DIR_OUT();
	CC2520_GPIO3_DIR_OUT();
	CC2520_GPIO4_DIR_OUT();
	CC2520_GPIO5_DIR_OUT();

	halRadioSpiInit();
	halMcuRfInterfaceInit();
#ifndef MRFI_CC2520
	//halDigioConfig(&pinRadio_GPIO0);
#endif
}

unsigned char halSpiExc(unsigned char x) {
	unsigned char rx;
	spiExchange(&CC2520_SPI, 1, &x, &rx);
	return rx;
}
