#ifndef __MODULE_H
#define __MODULE_H

/**
 * \file   module.h
 * \brief  Contains all constants and functions to access features of robot modules.
 * \author Alessandro Crespi
 * \date   July 2009
 */

#include <math.h>
#include "config.h"

// PID registers

#define MREG_MODE               0x28
#define MREG_SETPOINT_SOURCE    0x29
#define MREG_HW_OPTIONS         0x2A
#define MREG_SW_OPTIONS         0x2B
#define MREG_CONTROL_TYPE       0x2C
#define MREG_ERROR_FLAGS        0x2D
#define MREG_STATUS_FLAGS       0x2E
#define MREG_SETPOINT           0x2F
#define MREG_POSITION           0x30
#define MREG_SPEED              0x31
#define MREG_TORQUE             0x32
#define MREG_DIVIDER_POS        0x33
#define MREG_POSITION_HH        0x34
#define MREG_POSITION_HL        0x35
#define MREG_POSITION_LH        0x36
#define MREG_POSITION_LL        0x37
#define MREG_DIVIDER_SPEED      0x38
#define MREG_SPEED_H            0x39
#define MREG_SPEED_L            0x3A
#define MREG_DIVIDER_TORQUE     0x3B
#define MREG_TORQUE_H           0x3C
#define MREG_TORQUE_L           0x3D
#define MREG_TORQUE_BIAS_H      0x3E
#define MREG_TORQUE_BIAS_L      0x3F
#define MREG_KP_POS             0x40
#define MREG_KP_DIV_POS         0x41
#define MREG_KD_POS             0x42
#define MREG_KD_DIV_POS         0x43
#define MREG_FILTER_POS         0x44
#define MREG_KP_SPEED           0x45
#define MREG_KP_DIV_SPEED       0x46
#define MREG_KI_SPEED           0x47
#define MREG_KI_DIV_SPEED       0x48
#define MREG_KP_TORQUE          0x49
#define MREG_KP_DIV_TORQUE      0x4A
#define MREG_KI_TORQUE          0x4B
#define MREG_KI_DIV_TORQUE      0x4C
#define MREG_SAMPLE_TIME_H      0x4D
#define MREG_SAMPLE_TIME_L      0x4E
#define MREG_BLOCKED_TIME       0x4F
#define MREG_INT_GEN_PERIOD     0x50
#define MREG_INT_GEN_AMPLITUDE  0x51
#define MREG_INT_GEN_OFFSET     0x52
#define MREG_SOFT_STOP_MIN      0x53
#define MREG_SOFT_STOP_MAX      0x54
#define MREG_ACCELERATION       0x55
#define MREG_MAX_SPEED          0x56
#define MREG_STATIC_FRICTION    0x57
#define MREG_HW_CURRENT_LIMIT   0x58
#define MREG_SW_CURRENT_LIMIT   0x59
#define MREG_DELTA_8BIT         0x5A
#define MREG_PID_P_8BIT         0x5B
#define MREG_PID_I_8BIT         0x5C
#define MREG_PID_D_8BIT         0x5D
#define MREG_PID_OUT_8BIT       0x5E
#define MREG_EXT_DEVICE         0x61   // IR led (legs only)
#define MREG_IR_INPUT           0x62   // IR input (legs only)
#define MREG_RESET_VALUE        0x63   // coordinate reset (legs only)

// PID MREG_MODE values

///@{
#define MODE_IDLE               0
#define MODE_NORMAL             1
#define MODE_STOP               2
#define MODE_SLEEP              3
#define MODE_RESET              4
#define MODE_SETPOINT_RESET     6
///@}

// PID MREG_CONTROL_TYPE values

///@{
#define CONTROL_OPEN_LOOP       0
#define CONTROL_POS             1
#define CONTROL_POS_PROFILE     2
#define CONTROL_SPEED           3
#define CONTROL_SPEED_PROFILE   4
#define CONTROL_TORQUE          5
#define CONTROL_ZERO_FRICTION   6
///@}

// PID MREG_SETPOINT_SOURCE values

///@{
#define SETPOINT_SRC_I2C        0
#define SETPOINT_SRC_ANALOG     1
#define SETPOINT_SRC_SQUARE     2
#define SETPOINT_SRC_TRIANGLE   3
#define SETPOINT_SRC_SINUS      4
///@}

// PID MREG_HW_OPTIONS bits

///@{
#define HWO_STARTUP_NORMAL      0x01
#define HWO_ANALOG_SETPOINT     0x02
#define HWO_LED                 0x04
#define HWO_ENC_025             0x08
#define HWO_INVERT_TORQUE       0x10
///@}

// PID MREG_SW_OPTIONS bits

///@{
#define SWO_SEPARATE_D_TERM     0x01
#define SWO_ANTIRESET_WINDUP    0x02
#define SWO_SOFTSTOP_MIN_EN     0x04
#define SWO_SOFTSTOP_MAX_EN     0x08
#define SWO_ERROR_ON_SOFTSTOP   0x10
#define SWO_STOP_MOTOR_BLOCKED  0x20
#define SWO_SOFT_CURRENT_CTRL   0x40
#define SWO_INVERT_DIRECTION    0x80
///@}

// Other hardware registers
#ifdef HARDWARE_V3
#define MREG16_EXT_VOLTAGE      0x01
#define MREG16_BATT_VOLTAGE     0x02
#define MREG16_BATT_CURRENT     0x03
#define MREG16_BATT_ACC         0x04
#define MREG16_TEMPERATURE      0x05
#define MREG32_LED              0x01
#endif

/// Resolution of the motor's encoder (pulses per turn of the axis)
#define ENCODER_RESOLUTION      512

/** \def   BODY_GEARBOX_RATIO
 *  \brief Gearbox ratio of body joints
 *  \def   LEGS_GEARBOX_RATIO
 *  \brief Gearbox ratio of the leg joints
 *  \def   BODY_POS_DIVIDER
 *  \brief PID register value for position divider of body joints
 *  \def   LEGS_POS_DIVIDER
 *  \brief PID register value for position divider of leg joints
 *  \def   BODY_POS_DIVIDER_VAL
 *  \brief Should always be \f$ 2^{BODY\_POS\_DIVIDER} \f$
 *  \def   LEGS_POS_DIVIDER_VAL
 *  \brief Should always be \f$ 2^{LEGS\_POS\_DIVIDER} \f$
 */

#ifdef HARDWARE_V2
#define BODY_GEARBOX_RATIO      125
#define LEGS_GEARBOX_RATIO      50
#define BODY_POS_DIVIDER        7
#define BODY_POS_DIVIDER_VAL    128
#define LEGS_POS_DIVIDER        7
#define LEGS_POS_DIVIDER_VAL    128
#endif

#ifdef HARDWARE_V3
#define BODY_GEARBOX_RATIO      135
#define LEGS_GEARBOX_RATIO      90
#define BODY_POS_DIVIDER        7
#define BODY_POS_DIVIDER_VAL    128
#define LEGS_POS_DIVIDER        8
#define LEGS_POS_DIVIDER_VAL    256
#endif

#ifndef BODY_GEARBOX_RATIO
#error The version of the robot elements is not defined!
#endif

/// PID units per output axis full turn, body joints
#define UNITS_PER_TURN_BODY ((ENCODER_RESOLUTION * BODY_GEARBOX_RATIO) / BODY_POS_DIVIDER_VAL)
/// PID units per output axis full turn, leg joints
#define UNITS_PER_TURN_LEGS ((ENCODER_RESOLUTION * LEGS_GEARBOX_RATIO) / LEGS_POS_DIVIDER_VAL)

#ifndef M_TWOPI
#define M_TWOPI (2 * M_PI)
#endif

#define BODY_OUTPUT_RATIO_RAD (UNITS_PER_TURN_BODY / M_TWOPI)
#define BODY_OUTPUT_RATIO_DEG (UNITS_PER_TURN_BODY / 360.0f)
#define LEGS_OUTPUT_RATIO_RAD (UNITS_PER_TURN_LEGS / M_TWOPI)
#define LEGS_OUTPUT_RATIO_DEG (UNITS_PER_TURN_LEGS / 360.0f)

/// Converts radians to PID units for body joints
#define RAD_TO_OUTPUT_BODY(x) ((int8_t) (x * BODY_OUTPUT_RATIO_RAD))
/// Converts degrees to PID units for body joints
#define DEG_TO_OUTPUT_BODY(x) ((int8_t) (x * BODY_OUTPUT_RATIO_DEG))
/// Converts radians to PID units for leg joints
#define RAD_TO_OUTPUT_LEGS(x) ((int8_t) (x * LEGS_OUTPUT_RATIO_RAD))
/// Converts degrees to PID units for leg joints
#define DEG_TO_OUTPUT_LEGS(x) ((int8_t) (x * LEGS_OUTPUT_RATIO_DEG))

#endif // __MODULE_H
