/*
 *  Driver for VESC v6 ported to ROS melodic by Gero Schwäricke
 *  <gero.schwaericke@tum.de> from Technical University of Munich,
 *  Institute for Cyber-Physical Systems in Production Engineering
 *
 *  All credits for the implementation of the original tools vesc_driver
 *  from Michael T. Boulet <boulet@ll.mit.edu> licensed under BSD and the
 *  VESC Tool from Benjamin Vedder <benjamin@vedder.se> licensed under
 *  GNU GPLv3 or later go to their respective authors.
 *
 *  The following copyright notice stems from the source code of the
 *  VESC Tool by Benjamin Vedder. Some parts of his project were copied
 *  (and modified) to this project vesc_v6_driver for ROS by Gero
 *  Schwäricke. The license for those parts is thus the same as from the
 *  original work: GNU General Public License as published by the Free
 *  Software Foundation, either version 3 of the License, or (at your
 *  option) any later version.
 *  All parts based on the vesc_driver by Michael T. Boulet
 *  <boulet@ll.mit.edu> are licensed under BSD lincese as the original
 *  work.
 */
/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    VESC Tool is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VESC Tool is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef DATATYPES_H
#define DATATYPES_H

#include <memory>
#include <string>
#include <vector>

#include <stdint.h>

typedef struct {
  std::string name;
  std::string systemPath;
  bool isVesc;
} VSerialInfo_t;

typedef enum {
  CFG_T_UNDEFINED = 0,
  CFG_T_DOUBLE,
  CFG_T_INT,
  CFG_T_QSTRING,
  CFG_T_ENUM,
  CFG_T_BOOL
} CFG_T;

typedef enum {
  VESC_TX_UNDEFINED = 0,
  VESC_TX_UINT8,
  VESC_TX_INT8,
  VESC_TX_UINT16,
  VESC_TX_INT16,
  VESC_TX_UINT32,
  VESC_TX_INT32,
  VESC_TX_DOUBLE16,
  VESC_TX_DOUBLE32,
  VESC_TX_DOUBLE32_AUTO
} VESC_TX_T;

// General purpose drive output mode
typedef enum {
  GPD_OUTPUT_MODE_NONE = 0,
  GPD_OUTPUT_MODE_MODULATION,
  GPD_OUTPUT_MODE_VOLTAGE,
  GPD_OUTPUT_MODE_CURRENT
} gpd_output_mode;

typedef enum {
  FAULT_CODE_NONE = 0,
  FAULT_CODE_OVER_VOLTAGE,
  FAULT_CODE_UNDER_VOLTAGE,
  FAULT_CODE_DRV,
  FAULT_CODE_ABS_OVER_CURRENT,
  FAULT_CODE_OVER_TEMP_FET,
  FAULT_CODE_OVER_TEMP_MOTOR,
  FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
  FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
  FAULT_CODE_MCU_UNDER_VOLTAGE,
  FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
  FAULT_CODE_ENCODER_SPI,
  FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
  FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
  FAULT_CODE_FLASH_CORRUPTION,
  FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
  FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
  FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
  FAULT_CODE_UNBALANCED_CURRENTS
} mc_fault_code;

typedef enum {
  DISP_POS_MODE_NONE = 0,
  DISP_POS_MODE_INDUCTANCE,
  DISP_POS_MODE_OBSERVER,
  DISP_POS_MODE_ENCODER,
  DISP_POS_MODE_PID_POS,
  DISP_POS_MODE_PID_POS_ERROR,
  DISP_POS_MODE_ENCODER_OBSERVER_ERROR
} disp_pos_mode;

struct MC_VALUES {
 public:
  MC_VALUES() {
    v_in = 0.0;
    temp_mos = 0.0;
    temp_mos_1 = 0.0;
    temp_mos_2 = 0.0;
    temp_mos_3 = 0.0;
    temp_motor = 0.0;
    current_motor = 0.0;
    current_in = 0.0;
    id = 0.0;
    iq = 0.0;
    rpm = 0.0;
    duty_now = 0.0;
    amp_hours = 0.0;
    amp_hours_charged = 0.0;
    watt_hours = 0.0;
    watt_hours_charged = 0.0;
    tachometer = 0;
    tachometer_abs = 0;
    position = 0.0;
    fault_code = FAULT_CODE_NONE;
    vesc_id = 0;
  }

  double v_in;
  double temp_mos;
  double temp_mos_1;
  double temp_mos_2;
  double temp_mos_3;
  double temp_motor;
  double current_motor;
  double current_in;
  double id;
  double iq;
  double rpm;
  double duty_now;
  double amp_hours;
  double amp_hours_charged;
  double watt_hours;
  double watt_hours_charged;
  int tachometer;
  int tachometer_abs;
  double position;
  mc_fault_code fault_code;
  int vesc_id;
  std::string fault_str;
};

struct SETUP_VALUES {
 public:
  SETUP_VALUES() {
    temp_mos = 0.0;
    temp_motor = 0.0;
    current_motor = 0.0;
    current_in = 0.0;
    duty_now = 0.0;
    rpm = 0.0;
    speed = 0.0;
    v_in = 0.0;
    battery_level = 0.0;
    amp_hours = 0.0;
    amp_hours_charged = 0.0;
    watt_hours = 0.0;
    watt_hours_charged = 0.0;
    tachometer = 0.0;
    tachometer_abs = 0.0;
    position = 0.0;
    fault_code = FAULT_CODE_NONE;
    vesc_id = 0;
    num_vescs = 0;
    battery_wh = 0.0;
  }

  double temp_mos;
  double temp_motor;
  double current_motor;
  double current_in;
  double duty_now;
  double rpm;
  double speed;
  double v_in;
  double battery_level;
  double amp_hours;
  double amp_hours_charged;
  double watt_hours;
  double watt_hours_charged;
  double tachometer;
  double tachometer_abs;
  double position;
  mc_fault_code fault_code;
  int vesc_id;
  int num_vescs;
  double battery_wh;
  std::string fault_str;
};

struct IMU_VALUES {
 public:
  IMU_VALUES() {
    roll = 0;
    pitch = 0;
    yaw = 0;
    accX = 0;
    accY = 0;
    accZ = 0;
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
    magX = 0;
    magY = 0;
    magZ = 0;
    q0 = 1;
    q1 = 0;
    q2 = 0;
    q3 = 0;
  }

  double roll;
  double pitch;
  double yaw;

  double accX;
  double accY;
  double accZ;

  double gyroX;
  double gyroY;
  double gyroZ;

  double magX;
  double magY;
  double magZ;

  double q0;
  double q1;
  double q2;
  double q3;
};

struct MCCONF_TEMP {
 public:
  double current_min_scale;
  double current_max_scale;
  double erpm_or_speed_min;
  double erpm_or_speed_max;
  double duty_min;
  double duty_max;
  double watt_min;
  double watt_max;
  std::string name;
};

typedef enum {
  DEBUG_SAMPLING_OFF = 0,
  DEBUG_SAMPLING_NOW,
  DEBUG_SAMPLING_START,
  DEBUG_SAMPLING_TRIGGER_START,
  DEBUG_SAMPLING_TRIGGER_FAULT,
  DEBUG_SAMPLING_TRIGGER_START_NOSEND,
  DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND,
  DEBUG_SAMPLING_SEND_LAST_SAMPLES
} debug_sampling_mode;

typedef enum {
  COMM_FW_VERSION = 0,
  COMM_JUMP_TO_BOOTLOADER,
  COMM_ERASE_NEW_APP,
  COMM_WRITE_NEW_APP_DATA,
  COMM_GET_VALUES,
  COMM_SET_DUTY,
  COMM_SET_CURRENT,
  COMM_SET_CURRENT_BRAKE,
  COMM_SET_RPM,
  COMM_SET_POS,
  COMM_SET_HANDBRAKE,
  COMM_SET_DETECT,
  COMM_SET_SERVO_POS,
  COMM_SET_MCCONF,
  COMM_GET_MCCONF,
  COMM_GET_MCCONF_DEFAULT,
  COMM_SET_APPCONF,
  COMM_GET_APPCONF,
  COMM_GET_APPCONF_DEFAULT,
  COMM_SAMPLE_PRINT,
  COMM_TERMINAL_CMD,
  COMM_PRINT,
  COMM_ROTOR_POSITION,
  COMM_EXPERIMENT_SAMPLE,
  COMM_DETECT_MOTOR_PARAM,
  COMM_DETECT_MOTOR_R_L,
  COMM_DETECT_MOTOR_FLUX_LINKAGE,
  COMM_DETECT_ENCODER,
  COMM_DETECT_HALL_FOC,
  COMM_REBOOT,
  COMM_ALIVE,
  COMM_GET_DECODED_PPM,
  COMM_GET_DECODED_ADC,
  COMM_GET_DECODED_CHUK,
  COMM_FORWARD_CAN,
  COMM_SET_CHUCK_DATA,
  COMM_CUSTOM_APP_DATA,
  COMM_NRF_START_PAIRING,
  COMM_GPD_SET_FSW,
  COMM_GPD_BUFFER_NOTIFY,
  COMM_GPD_BUFFER_SIZE_LEFT,
  COMM_GPD_FILL_BUFFER,
  COMM_GPD_OUTPUT_SAMPLE,
  COMM_GPD_SET_MODE,
  COMM_GPD_FILL_BUFFER_INT8,
  COMM_GPD_FILL_BUFFER_INT16,
  COMM_GPD_SET_BUFFER_INT_SCALE,
  COMM_GET_VALUES_SETUP,
  COMM_SET_MCCONF_TEMP,
  COMM_SET_MCCONF_TEMP_SETUP,
  COMM_GET_VALUES_SELECTIVE,
  COMM_GET_VALUES_SETUP_SELECTIVE,
  COMM_EXT_NRF_PRESENT,
  COMM_EXT_NRF_ESB_SET_CH_ADDR,
  COMM_EXT_NRF_ESB_SEND_DATA,
  COMM_EXT_NRF_ESB_RX_DATA,
  COMM_EXT_NRF_SET_ENABLED,
  COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
  COMM_DETECT_APPLY_ALL_FOC,
  COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
  COMM_ERASE_NEW_APP_ALL_CAN,
  COMM_WRITE_NEW_APP_DATA_ALL_CAN,
  COMM_PING_CAN,
  COMM_APP_DISABLE_OUTPUT,
  COMM_TERMINAL_CMD_SYNC,
  COMM_GET_IMU_DATA,
  COMM_BM_CONNECT,
  COMM_BM_ERASE_FLASH_ALL,
  COMM_BM_WRITE_FLASH,
  COMM_BM_REBOOT,
  COMM_BM_DISCONNECT
} COMM_PACKET_ID;

typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS,
  CAN_PACKET_SET_CURRENT_REL,
  CAN_PACKET_SET_CURRENT_BRAKE_REL,
  CAN_PACKET_SET_CURRENT_HANDBRAKE,
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
  CAN_PACKET_STATUS_2,
  CAN_PACKET_STATUS_3,
  CAN_PACKET_STATUS_4,
  CAN_PACKET_PING,
  CAN_PACKET_PONG,
  CAN_PACKET_DETECT_APPLY_ALL_FOC,
  CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
  CAN_PACKET_CONF_CURRENT_LIMITS,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
  CAN_PACKET_CONF_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_FOC_ERPMS,
  CAN_PACKET_CONF_STORE_FOC_ERPMS,
  CAN_PACKET_STATUS_5
} CAN_PACKET_ID;

typedef struct {
  int js_x;
  int js_y;
  int acc_x;
  int acc_y;
  int acc_z;
  bool bt_c;
  bool bt_z;
} chuck_data;

struct bldc_detect {
 public:
  double cycle_int_limit;
  double bemf_coupling_k;
  std::vector<int> hall_table;
  int hall_res;
};

typedef enum { NRF_PAIR_STARTED = 0, NRF_PAIR_OK, NRF_PAIR_FAIL } NRF_PAIR_RES;

#endif  // DATATYPES_H
