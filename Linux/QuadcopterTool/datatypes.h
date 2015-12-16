/*
    Copyright 2013 - 2015 Benjamin Vedder	benjamin@vedder.se

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

#ifndef DATATYPES_H
#define DATATYPES_H

#include <stdint.h>

// Point
typedef struct {
    double x;
    double y;
} POINT_t;

// Line segment
typedef struct {
    POINT_t p1;
    POINT_t p2;
} SEGMENT_t;

// Collision opposite vector
typedef struct {
    POINT_t dir;
    double mag;
} COL_OPP_VECTOR_t;

// Position state_structure
typedef struct {
    double px;
    double py;
    double vx;
    double vy;
    double acc_roll_err;
    double acc_pitch_err;
    double roll;
    double pitch;
    double yaw;
} POS_STATE_t;

typedef struct {
    int id;
    double px;
    double py;
    double pz;
} ANCHOR_SETTINGS_t;

typedef struct {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
} MAP_LIMITS_t;

// Input to the copter controller
typedef struct {
    double rollCmd; // Range [-1.0 1.0]
    double pitchCmd; // Range [-1.0 1.0]
    double yawCmd; // Range [-1.0 1.0]
    double throttleCmd; // Range [-1.0 1.0]
} CONTROL_INPUT_t;

// ITS station message
typedef struct {
    int id; // The ID of the sender
    uint64_t timestamp; // What the sender thought the global time was
    POS_STATE_t state; // The position/orientation/speed of the sender
} ITS_MSG_t;

// Risk types
typedef enum {
    LDM_ELEMENT_TYPE_POINT = 0,
    LDM_ELEMENT_TYPE_LINE,
    LDM_ELEMENT_TYPE_QUADCOPTER, // A point, but dynamic
    LDM_ELEMENT_TYPE_COMFORT_ZONE // My own comfort zone
} LDM_ELEMENT_TYPE_t;

// LDM risk element with type and risk contour
typedef struct {
    double px;
    double py;
    double rotation;
    double width;
    double height;
    int has_collision;
} LDM_RISK_ELEMENT_t;

// A node in the LDM of the ITS station
typedef struct {
    uint64_t timestamp;
    int id;
    LDM_ELEMENT_TYPE_t type;
    union {
        POS_STATE_t pos;
        SEGMENT_t segment;
    };
    LDM_RISK_ELEMENT_t risk;
} LDM_ELEMENT_t;

// Packet commands
typedef enum {
    PACKET_CMD_SET_STICK_INPUT = 0,
    PACKET_CMD_GET_CONTROL_PARAMETERS,
    PACKET_CMD_SET_CONTROL_PARAMETERS,
    PACKET_CMD_GET_ORIENTATION,
    PACKET_CMD_RESET_GYRO_STICKS,
    PACKET_CMD_RESET_ORIENTATION,
    PACKET_CMD_GET_RAW_IMU,
    PACKET_CMD_OVERRIDE_POWER,
    PACKET_CMD_PRINT,
    PACKET_CMD_GET_POS,
    PACKET_CMD_SET_POS,
    PACKET_CMD_GET_ALTITUDE,
    PACKET_CMD_MAP,
    PACKET_CMD_LDM_ELEMENT,
    PACKET_CMD_LINE_SEGMENT,
    PACKET_CMD_GET_ULTRA_DIST,
    PACKET_CMD_RADIO_ALIVE,
    PACKET_CMD_SAFETY_MSG,
    PACKET_CMD_STATUS_MSG,
    PACKET_CMD_SET_ANCHORS,
    PACKET_CMD_GET_ANCHORS,
    PACKET_CMD_EMERGENCY_STOP,
    PACKET_CMD_SET_YAW,
    PACKET_CMD_SET_LED_EXT_IND_LANDED,
    PACKET_CMD_SIM
} PACKET_CMD_t;

// PID control parameters
typedef struct {
    volatile double roll_p_gain;
    volatile double roll_i_gain;
    volatile double roll_d_gain_process;
    volatile double roll_d_gain_error;

    volatile double pitch_p_gain;
    volatile double pitch_i_gain;
    volatile double pitch_d_gain_process;
    volatile double pitch_d_gain_error;

    volatile double yaw_p_gain;
    volatile double yaw_i_gain;
    volatile double yaw_d_gain_process;
    volatile double yaw_d_gain_error;
} CONTROL_PARAMETERS_t;

// Raw IMU value structure
typedef struct {
    double accX;
    double accY;
    double accZ;
    double gyroX;
    double gyroY;
    double gyroZ;
    double magX;
    double magY;
    double magZ;
} RAW_IMU_t;

// PPM structure
typedef struct {
    double roll;
    double pitch;
    double yaw;
    double throttle;
    double pot1;
    double pot2;
} PPM_RADIO_t;

// The first POS_MASK_BITS bits are the type, the rest of the bits
// are attributes for the type
typedef enum {
    POS_TYPE_NORMAL = 0,
    POS_TYPE_COLLISION
} POS_TYPE_t;

typedef enum {
    POS_MASK_TYPE = 0xF,
    POS_MASK_BITS = 4,
    // Normal
    POS_MASK_PERCEIVED = (1 << (POS_MASK_BITS + 0)),
    POS_MASK_FOLLOWING = (1 << (POS_MASK_BITS + 1)),
    POS_MASK_HAS_COLLISION = (1 << (POS_MASK_BITS + 2))
    // Collision
} POS_MASK_t;

typedef enum {
    MAP_CMD_REMOVE_QUAD = 0,
    MAP_CMD_REMOVE_ALL_QUADS,
    MAP_CMD_REMOVE_COLLISION,
    MAP_CMD_REMOVE_ALL_COLLISIONS,
    MAP_CMD_REMOVE_ALL_RISKS,
    MAP_CMD_REMOVE_ALL_LINE_SEGMENTS,
    MAP_CMD_REMOVE_ANCHOR,
    MAP_CMD_REMOVE_ALL_ANCHORS
} MAP_CMD_t;

typedef enum {
    SIM_CMD_POS_ERROR,
    SIM_CMD_FAULTS
} SIM_CMD_t;

// UDP Commands
typedef enum {
    CMD_CLIENT_SEND_PULSE_SAMPLE = 100,
    CMD_CLIENT_SEND_PULSE_SAMPLE_ALLDATA,
    CMD_CLIENT_SEND_PULSE_SAMPLE_ONLYDIST,
    CMD_CLIENT_SET_LOS
} CMD_CLIENT_t;

typedef enum {
    CMD_SERVER_TEXT  = 0,
    CMD_SERVER_LOGGED_PULSE,
    CMD_SERVER_LOGGED_CORRELATION,
    CMD_SERVER_LOGGED_PULSE_PART,
    CMD_SERVER_LOGGED_CORRELATION_PART,
    CMD_SERVER_DISTANCES,
    CMD_SERVER_QUAD_PACKET
} CMD_SERVER_t;

typedef enum {
    LEVEL_OF_SERVICE_LOW = 0,
    LEVEL_OF_SERVICE_MEDIUM,
    LEVEL_OF_SERVICE_HIGH
} LEVEL_OF_SERVICE_t;

typedef struct {
    int id;
    LEVEL_OF_SERVICE_t los_local;
    LEVEL_OF_SERVICE_t los_global;
} SAFETY_MSG_t;

typedef struct {
    int id;
    unsigned char fw_version_major;
    unsigned char fw_version_minor;
    double px;
    double py;
    double pz;
    double roll;
    double pitch;
    double yaw;
    double vx;
    double vy;
    double vbat;
    uint16_t adc_in[4];
} STATUS_MSG_t;

typedef enum {
    LED_EXT_INDICATION_LANDED_DIR = 0,
    LED_EXT_INDICATION_LANDED_LOS,
    LED_EXT_INDICATION_LANDED_LOS_DIR,
    LED_EXT_INDICATION_LANDED_BATTERY
} LED_EXT_INDICATION_LANDED;

#endif // DATATYPES_H
