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

#include "vesc_v6_driver/commands.h"

#include "vesc_v6_driver/packet.h"

#include <ros/node_handle.h>

// pub msg
#include "vesc_v6_driver/ackReceived.h"
#include "vesc_v6_driver/bldcDetectReceived.h"
#include "vesc_v6_driver/bmConnRes.h"
#include "vesc_v6_driver/bmRebootRes.h"
#include "vesc_v6_driver/decodedAdcReceived.h"
#include "vesc_v6_driver/decodedChukReceived.h"
#include "vesc_v6_driver/decodedPpmReceived.h"
#include "vesc_v6_driver/detectAllFocReceived.h"
#include "vesc_v6_driver/encoderParamReceived.h"
#include "vesc_v6_driver/experimentSamplesReceived.h"
#include "vesc_v6_driver/focHallTableReceived.h"
#include "vesc_v6_driver/fwVersionReceived.h"
#include "vesc_v6_driver/gpdBufferNotifyReceived.h"
#include "vesc_v6_driver/gpdBufferSizeLeftReceived.h"
#include "vesc_v6_driver/motorLinkageReceived.h"
#include "vesc_v6_driver/motorRLReceived.h"
#include "vesc_v6_driver/nrfPairingRes.h"
#include "vesc_v6_driver/pingCanRx.h"
#include "vesc_v6_driver/printReceived.h"
#include "vesc_v6_driver/rotorPosReceived.h"
#include "vesc_v6_driver/samplesReceived.h"
#include "vesc_v6_driver/valuesImuReceived.h"
#include "vesc_v6_driver/valuesReceived.h"
#include "vesc_v6_driver/valuesSetupReceived.h"

void VescCommands::Setup(ros::NodeHandle nh) {
  mSendCan = false;
  mCanId = -1;
  mIsLimitedMode = false;

  mTimeoutCount = 100;
  mTimeoutFwVer = 0;
  mTimeoutValues = 0;
  mTimeoutValuesSetup = 0;
  mTimeoutImuData = 0;
  mTimeoutDecPpm = 0;
  mTimeoutDecAdc = 0;
  mTimeoutDecChuk = 0;
  mTimeoutPingCan = 0;

  /* TODO register ros timeout
  mTimer = new QTimer(this);
  mTimer->setInterval(10);
  mTimer->start();
  connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
  */
  mTimer = nh.createTimer(ros::Duration(10.0 / 1000.0),
                          &VescCommands::timerSlot, this);

  ackReceived_pub =
      nh.advertise<vesc_v6_driver::ackReceived>("vesc/ackReceived", 10);
  bmRebootRes_pub =
      nh.advertise<vesc_v6_driver::bmRebootRes>("vesc/bmRebootRes", 10);
  decodedPpmReceived_pub = nh.advertise<vesc_v6_driver::decodedPpmReceived>(
      "vesc/decodedPpmReceived", 10);
  experimentSamplesReceived_pub =
      nh.advertise<vesc_v6_driver::experimentSamplesReceived>(
          "vesc/experimentSamplesReceived", 10);
  gpdBufferNotifyReceived_pub =
      nh.advertise<vesc_v6_driver::gpdBufferNotifyReceived>(
          "vesc/gpdBufferNotifyReceived", 10);
  motorRLReceived_pub =
      nh.advertise<vesc_v6_driver::motorRLReceived>("vesc/motorRLReceived", 10);
  printReceived_pub =
      nh.advertise<vesc_v6_driver::printReceived>("vesc/printReceived", 10);
  valuesImuReceived_pub = nh.advertise<vesc_v6_driver::valuesImuReceived>(
      "vesc/valuesImuReceived", 10);
  bldcDetectReceived_pub = nh.advertise<vesc_v6_driver::bldcDetectReceived>(
      "vesc/bldcDetectReceived", 10);
  decodedAdcReceived_pub = nh.advertise<vesc_v6_driver::decodedAdcReceived>(
      "vesc/decodedAdcReceived", 10);
  detectAllFocReceived_pub = nh.advertise<vesc_v6_driver::detectAllFocReceived>(
      "vesc/detectAllFocReceived", 10);
  focHallTableReceived_pub = nh.advertise<vesc_v6_driver::focHallTableReceived>(
      "vesc/focHallTableReceived", 10);
  gpdBufferSizeLeftReceived_pub =
      nh.advertise<vesc_v6_driver::gpdBufferSizeLeftReceived>(
          "vesc/gpdBufferSizeLeftReceived", 10);
  nrfPairingRes_pub =
      nh.advertise<vesc_v6_driver::nrfPairingRes>("vesc/nrfPairingRes", 10);
  rotorPosReceived_pub = nh.advertise<vesc_v6_driver::rotorPosReceived>(
      "vesc/rotorPosReceived", 10);
  valuesReceived_pub =
      nh.advertise<vesc_v6_driver::valuesReceived>("vesc/valuesReceived", 10);
  bmConnRes_pub = nh.advertise<vesc_v6_driver::bmConnRes>("vesc/bmConnRes", 10);
  decodedChukReceived_pub = nh.advertise<vesc_v6_driver::decodedChukReceived>(
      "vesc/decodedChukReceived", 10);
  encoderParamReceived_pub = nh.advertise<vesc_v6_driver::encoderParamReceived>(
      "vesc/encoderParamReceived", 10);
  fwVersionReceived_pub = nh.advertise<vesc_v6_driver::fwVersionReceived>(
      "vesc/fwVersionReceived", 10);
  motorLinkageReceived_pub = nh.advertise<vesc_v6_driver::motorLinkageReceived>(
      "vesc/motorLinkageReceived", 10);
  pingCanRx_pub = nh.advertise<vesc_v6_driver::pingCanRx>("vesc/pingCanRx", 10);
  samplesReceived_pub =
      nh.advertise<vesc_v6_driver::samplesReceived>("vesc/samplesReceived", 10);
  valuesSetupReceived_pub = nh.advertise<vesc_v6_driver::valuesSetupReceived>(
      "vesc/valuesSetupReceived", 10);

  bmConnect_sub = nh.subscribe("vesc/bmConnect", 10,
                               &VescCommands::bmConnectCallback, this);
  getDecodedChuk_sub = nh.subscribe(
      "vesc/getDecodedChuk", 10, &VescCommands::getDecodedChukCallback, this);
  getValuesSelective_sub =
      nh.subscribe("vesc/getValuesSelective", 10,
                   &VescCommands::getValuesSelectiveCallback, this);
  gpdOutputSample_sub = nh.subscribe(
      "vesc/gpdOutputSample", 10, &VescCommands::gpdOutputSampleCallback, this);
  measureLinkage_sub = nh.subscribe(
      "vesc/measureLinkage", 10, &VescCommands::measureLinkageCallback, this);
  samplePrint_sub = nh.subscribe("vesc/samplePrint", 10,
                                 &VescCommands::samplePrintCallback, this);
  setCurrentBrake_sub = nh.subscribe(
      "vesc/setCurrentBrake", 10, &VescCommands::setCurrentBrakeCallback, this);
  setServoPos_sub = nh.subscribe("vesc/setServoPos", 10,
                                 &VescCommands::setServoPosCallback, this);
  bmDisconnect_sub = nh.subscribe("vesc/bmDisconnect", 10,
                                  &VescCommands::bmDisconnectCallback, this);
  getDecodedPpm_sub = nh.subscribe("vesc/getDecodedPpm", 10,
                                   &VescCommands::getDecodedPpmCallback, this);
  getValuesSetup_sub = nh.subscribe(
      "vesc/getValuesSetup", 10, &VescCommands::getValuesSetupCallback, this);
  gpdSetBufferIntScale_sub =
      nh.subscribe("vesc/gpdSetBufferIntScale", 10,
                   &VescCommands::gpdSetBufferIntScaleCallback, this);
  measureLinkageOpenloop_sub =
      nh.subscribe("vesc/measureLinkageOpenloop", 10,
                   &VescCommands::measureLinkageOpenloopCallback, this);
  sendAlive_sub = nh.subscribe("vesc/sendAlive", 10,
                               &VescCommands::sendAliveCallback, this);
  setDetect_sub = nh.subscribe("vesc/setDetect", 10,
                               &VescCommands::setDetectCallback, this);
  bmReboot_sub =
      nh.subscribe("vesc/bmReboot", 10, &VescCommands::bmRebootCallback, this);
  getFwVersion_sub = nh.subscribe("vesc/getFwVersion", 10,
                                  &VescCommands::getFwVersionCallback, this);
  getValuesSetupSelective_sub =
      nh.subscribe("vesc/getValuesSetupSelective", 10,
                   &VescCommands::getValuesSetupSelectiveCallback, this);
  gpdSetFsw_sub = nh.subscribe("vesc/gpdSetFsw", 10,
                               &VescCommands::gpdSetFswCallback, this);
  measureRL_sub = nh.subscribe("vesc/measureRL", 10,
                               &VescCommands::measureRLCallback, this);
  sendTerminalCmd_sub = nh.subscribe(
      "vesc/sendTerminalCmd", 10, &VescCommands::sendTerminalCmdCallback, this);
  setDutyCycle_sub = nh.subscribe("vesc/setDutyCycle", 10,
                                  &VescCommands::setDutyCycleCallback, this);
  detectAllFoc_sub = nh.subscribe("vesc/detectAllFoc", 10,
                                  &VescCommands::detectAllFocCallback, this);
  getGpdBufferSizeLeft_sub =
      nh.subscribe("vesc/getGpdBufferSizeLeft", 10,
                   &VescCommands::getGpdBufferSizeLeftCallback, this);
  gpdFillBuffer_sub = nh.subscribe("vesc/gpdFillBuffer", 10,
                                   &VescCommands::gpdFillBufferCallback, this);
  gpdSetMode_sub = nh.subscribe("vesc/gpdSetMode", 10,
                                &VescCommands::gpdSetModeCallback, this);
  pairNrf_sub =
      nh.subscribe("vesc/pairNrf", 10, &VescCommands::pairNrfCallback, this);
  sendTerminalCmdSync_sub =
      nh.subscribe("vesc/sendTerminalCmdSync", 10,
                   &VescCommands::sendTerminalCmdSyncCallback, this);
  setHandbrake_sub = nh.subscribe("vesc/setHandbrake", 10,
                                  &VescCommands::setHandbrakeCallback, this);
  detectMotorParam_sub =
      nh.subscribe("vesc/detectMotorParam", 10,
                   &VescCommands::detectMotorParamCallback, this);
  getImuData_sub = nh.subscribe("vesc/getImuData", 10,
                                &VescCommands::getImuDataCallback, this);
  gpdFillBufferInt8_sub =
      nh.subscribe("vesc/gpdFillBufferInt8", 10,
                   &VescCommands::gpdFillBufferInt8Callback, this);
  measureEncoder_sub = nh.subscribe(
      "vesc/measureEncoder", 10, &VescCommands::measureEncoderCallback, this);
  pingCan_sub =
      nh.subscribe("vesc/pingCan", 10, &VescCommands::pingCanCallback, this);
  setChukData_sub = nh.subscribe("vesc/setChukData", 10,
                                 &VescCommands::setChukDataCallback, this);
  setPos_sub =
      nh.subscribe("vesc/setPos", 10, &VescCommands::setPosCallback, this);
  getDecodedAdc_sub = nh.subscribe("vesc/getDecodedAdc", 10,
                                   &VescCommands::getDecodedAdcCallback, this);
  getValues_sub = nh.subscribe("vesc/getValues", 10,
                               &VescCommands::getValuesCallback, this);
  gpdFillBufferInt16_sub =
      nh.subscribe("vesc/gpdFillBufferInt16", 10,
                   &VescCommands::gpdFillBufferInt16Callback, this);
  measureHallFoc_sub = nh.subscribe(
      "vesc/measureHallFoc", 10, &VescCommands::measureHallFocCallback, this);
  reboot_sub =
      nh.subscribe("vesc/reboot", 10, &VescCommands::rebootCallback, this);
  setCurrent_sub = nh.subscribe("vesc/setCurrent", 10,
                                &VescCommands::setCurrentCallback, this);
  setRpm_sub =
      nh.subscribe("vesc/setRpm", 10, &VescCommands::setRpmCallback, this);

  mIsSetup = true;
}

void VescCommands::setLimitedMode(bool is_limited) {
  mIsLimitedMode = is_limited;
}

bool VescCommands::isLimitedMode() { return mIsLimitedMode; }

bool VescCommands::setSendCan(bool sendCan, int id) {
  if (id >= 0) {
    mCanId = id;
  }

  if (mCanId >= 0) {
    mSendCan = sendCan;
  } else {
    mSendCan = false;
  }

  return mSendCan == sendCan;
}

bool VescCommands::getSendCan() { return mSendCan; }

void VescCommands::setCanSendId(unsigned int id) {
  mCanId = static_cast<int>(id);
}

int VescCommands::getCanSendId() { return mCanId; }

void VescCommands::processPacket(ByteArray& vb) {
  if (mIsSetup == false) {
    ROS_ERROR(
        "The function processPacket was called before setting up VescCommands! "
        "Set up VescCommands before any usage of this function.");
    return;
  }

  COMM_PACKET_ID id = static_cast<COMM_PACKET_ID>(vb.PopFrontUint8());
  switch (id) {
    case COMM_FW_VERSION: {
      mTimeoutFwVer = 0;
      int8_t fw_major = -1;
      int8_t fw_minor = -1;
      std::string hw;
      ByteArray uuid;
      bool isPaired = false;

      if (vb.size() >= 2) {
        fw_major = vb.PopFrontInt8();
        fw_minor = vb.PopFrontInt8();
        hw = vb.PopFrontString();
      }

      if (vb.size() >= 12) {
        vb.PopFrontNBytes(uuid, 12);
      }

      if (vb.size() >= 1) {
        isPaired = vb.PopFrontInt8();
      }

      fwVersionReceived(fw_major, fw_minor, hw, uuid, isPaired);
    } break;

    case COMM_ERASE_NEW_APP:
    case COMM_WRITE_NEW_APP_DATA:
      ROS_WARN("This driver does not support firmware uploading. (rec: %u)",
               id);
      break;

    case COMM_GET_VALUES:
    case COMM_GET_VALUES_SELECTIVE: {
      mTimeoutValues = 0;
      MC_VALUES values;

      uint32_t mask = 0xFFFFFFFF;
      if (id == COMM_GET_VALUES_SELECTIVE) {
        mask = vb.PopFrontUint32();
      }

      if (mask & (static_cast<uint32_t>(1) << 0)) {
        values.temp_mos = vb.PopFrontDouble16(1e1);
      }
      if (mask & (static_cast<uint32_t>(1) << 1)) {
        values.temp_motor = vb.PopFrontDouble16(1e1);
      }
      if (mask & (static_cast<uint32_t>(1) << 2)) {
        values.current_motor = vb.PopFrontDouble32(1e2);
      }
      if (mask & (static_cast<uint32_t>(1) << 3)) {
        values.current_in = vb.PopFrontDouble32(1e2);
      }
      if (mask & (static_cast<uint32_t>(1) << 4)) {
        values.id = vb.PopFrontDouble32(1e2);
      }
      if (mask & (static_cast<uint32_t>(1) << 5)) {
        values.iq = vb.PopFrontDouble32(1e2);
      }
      if (mask & (static_cast<uint32_t>(1) << 6)) {
        values.duty_now = vb.PopFrontDouble16(1e3);
      }
      if (mask & (static_cast<uint32_t>(1) << 7)) {
        values.rpm = vb.PopFrontDouble32(1e0);
      }
      if (mask & (static_cast<uint32_t>(1) << 8)) {
        values.v_in = vb.PopFrontDouble16(1e1);
      }
      if (mask & (static_cast<uint32_t>(1) << 9)) {
        values.amp_hours = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 10)) {
        values.amp_hours_charged = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 11)) {
        values.watt_hours = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 12)) {
        values.watt_hours_charged = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 13)) {
        values.tachometer = vb.PopFrontInt32();
      }
      if (mask & (static_cast<uint32_t>(1) << 14)) {
        values.tachometer_abs = vb.PopFrontInt32();
      }
      if (mask & (static_cast<uint32_t>(1) << 15)) {
        values.fault_code = static_cast<mc_fault_code>(vb.PopFrontInt8());
        values.fault_str = faultToStr(values.fault_code);
      }

      if (vb.size() >= 4) {
        if (mask & (static_cast<uint32_t>(1) << 16)) {
          values.position = vb.PopFrontDouble32(1e6);
        }
      } else {
        values.position = -1.0;
      }

      if (vb.size() >= 1) {
        if (mask & (static_cast<uint32_t>(1) << 17)) {
          values.vesc_id = vb.PopFrontUint8();
        }
      } else {
        values.vesc_id = 255;
      }

      if (vb.size() >= 6) {
        if (mask & (static_cast<uint32_t>(1) << 18)) {
          values.temp_mos_1 = vb.PopFrontDouble16(1e1);
          values.temp_mos_2 = vb.PopFrontDouble16(1e1);
          values.temp_mos_3 = vb.PopFrontDouble16(1e1);
        }
      }

      valuesReceived(values, mask);
    } break;

    case COMM_PRINT:
      printReceived(vb.PopFrontString());
      break;

    case COMM_SAMPLE_PRINT:
      samplesReceived(vb);
      break;

    case COMM_ROTOR_POSITION:
      rotorPosReceived(vb.PopFrontDouble32(1e5));
      break;

    case COMM_EXPERIMENT_SAMPLE: {
      std::vector<double> samples;
      while (!vb.empty()) {
        samples.push_back(vb.PopFrontDouble32(1e4));
      }
      experimentSamplesReceived(samples);
    } break;

    case COMM_GET_MCCONF:
    case COMM_GET_MCCONF_DEFAULT:
      ROS_ERROR(
          "This driver does not support MC configuration modification. (rec: "
          "%u)",
          id);
      break;

    case COMM_GET_APPCONF:
    case COMM_GET_APPCONF_DEFAULT:
      ROS_ERROR(
          "This driver does not support APP configuration modification. (rec: "
          "%u)",
          id);
      break;

    case COMM_DETECT_MOTOR_PARAM: {
      bldc_detect param;
      param.cycle_int_limit = vb.PopFrontDouble32(1e3);
      param.bemf_coupling_k = vb.PopFrontDouble32(1e3);
      for (int i = 0; i < 8; i++) {
        param.hall_table.push_back(static_cast<int>(vb.PopFrontUint8()));
      }
      param.hall_res = static_cast<int>(vb.PopFrontUint8());
      bldcDetectReceived(param);
    } break;

    case COMM_DETECT_MOTOR_R_L: {
      double r = vb.PopFrontDouble32(1e6);
      double l = vb.PopFrontDouble32(1e3);
      motorRLReceived(r, l);
    } break;

    case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
      motorLinkageReceived(vb.PopFrontDouble32(1e7));
    } break;

    case COMM_DETECT_ENCODER: {
      double offset = vb.PopFrontDouble32(1e6);
      double ratio = vb.PopFrontDouble32(1e6);
      bool inverted = vb.PopFrontInt8();
      encoderParamReceived(offset, ratio, inverted);
    } break;

    case COMM_DETECT_HALL_FOC: {
      std::vector<int> table;
      for (int i = 0; i < 8; i++) {
        table.push_back(vb.PopFrontUint8());
      }
      int res = vb.PopFrontUint8();
      focHallTableReceived(table, res);
    } break;

    case COMM_GET_DECODED_PPM: {
      mTimeoutDecPpm = 0;
      double dec_ppm = vb.PopFrontDouble32(1e6);
      double ppm_last_len = vb.PopFrontDouble32(1e6);
      decodedPpmReceived(dec_ppm, ppm_last_len);
    } break;

    case COMM_GET_DECODED_ADC: {
      mTimeoutDecAdc = 0;
      double dec_adc = vb.PopFrontDouble32(1e6);
      double dec_adc_voltage = vb.PopFrontDouble32(1e6);
      double dec_adc2 = vb.PopFrontDouble32(1e6);
      double dec_adc_voltage2 = vb.PopFrontDouble32(1e6);
      decodedAdcReceived(dec_adc, dec_adc_voltage, dec_adc2, dec_adc_voltage2);
    } break;

    case COMM_GET_DECODED_CHUK:
      mTimeoutDecChuk = 0;
      decodedChukReceived(vb.PopFrontDouble32(1000000.0));
      break;

    case COMM_SET_MCCONF:
      ackReceived("MCCONF Write OK");
      break;

    case COMM_SET_APPCONF:
      ackReceived("APPCONF Write OK");
      break;

    case COMM_CUSTOM_APP_DATA:
      ROS_ERROR(
          "This driver does not support custom APP data transfer. (rec: %u)",
          id);
      break;

    case COMM_NRF_START_PAIRING:
      nrfPairingRes(static_cast<NRF_PAIR_RES>(vb.PopFrontInt8()));
      break;

    case COMM_GPD_BUFFER_NOTIFY:
      gpdBufferNotifyReceived();
      break;

    case COMM_GPD_BUFFER_SIZE_LEFT:
      gpdBufferSizeLeftReceived(vb.PopFrontInt16());
      break;

    case COMM_GET_VALUES_SETUP:
    case COMM_GET_VALUES_SETUP_SELECTIVE: {
      mTimeoutValuesSetup = 0;
      SETUP_VALUES values;

      uint32_t mask = 0xFFFFFFFF;
      if (id == COMM_GET_VALUES_SETUP_SELECTIVE) {
        mask = vb.PopFrontUint32();
      }

      if (mask & (static_cast<uint32_t>(1) << 0)) {
        values.temp_mos = vb.PopFrontDouble16(1e1);
      }
      if (mask & (static_cast<uint32_t>(1) << 1)) {
        values.temp_motor = vb.PopFrontDouble16(1e1);
      }
      if (mask & (static_cast<uint32_t>(1) << 2)) {
        values.current_motor = vb.PopFrontDouble32(1e2);
      }
      if (mask & (static_cast<uint32_t>(1) << 3)) {
        values.current_in = vb.PopFrontDouble32(1e2);
      }
      if (mask & (static_cast<uint32_t>(1) << 4)) {
        values.duty_now = vb.PopFrontDouble16(1e3);
      }
      if (mask & (static_cast<uint32_t>(1) << 5)) {
        values.rpm = vb.PopFrontDouble32(1e0);
      }
      if (mask & (static_cast<uint32_t>(1) << 6)) {
        values.speed = vb.PopFrontDouble32(1e3);
      }
      if (mask & (static_cast<uint32_t>(1) << 7)) {
        values.v_in = vb.PopFrontDouble16(1e1);
      }
      if (mask & (static_cast<uint32_t>(1) << 8)) {
        values.battery_level = vb.PopFrontDouble16(1e3);
      }
      if (mask & (static_cast<uint32_t>(1) << 9)) {
        values.amp_hours = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 10)) {
        values.amp_hours_charged = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 11)) {
        values.watt_hours = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 12)) {
        values.watt_hours_charged = vb.PopFrontDouble32(1e4);
      }
      if (mask & (static_cast<uint32_t>(1) << 13)) {
        values.tachometer = vb.PopFrontDouble32(1e3);
      }
      if (mask & (static_cast<uint32_t>(1) << 14)) {
        values.tachometer_abs = vb.PopFrontDouble32(1e3);
      }
      if (mask & (static_cast<uint32_t>(1) << 15)) {
        values.position = vb.PopFrontDouble32(1e6);
      }
      if (mask & (static_cast<uint32_t>(1) << 16)) {
        values.fault_code = (mc_fault_code)vb.PopFrontInt8();
        values.fault_str = faultToStr(values.fault_code);
      }
      if (mask & (static_cast<uint32_t>(1) << 17)) {
        values.vesc_id = vb.PopFrontUint8();
      }
      if (mask & (static_cast<uint32_t>(1) << 18)) {
        values.num_vescs = vb.PopFrontUint8();
      }
      if (mask & (static_cast<uint32_t>(1) << 19)) {
        values.battery_wh = vb.PopFrontDouble32(1e3);
      }

      valuesSetupReceived(values, mask);
    } break;

    case COMM_SET_MCCONF_TEMP:
      ackReceived("COMM_SET_MCCONF_TEMP Write OK");
      break;

    case COMM_SET_MCCONF_TEMP_SETUP:
      ackReceived("COMM_SET_MCCONF_TEMP_SETUP Write OK");
      break;

    case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
      motorLinkageReceived(vb.PopFrontDouble32(1e7));
      break;

    case COMM_DETECT_APPLY_ALL_FOC:
      detectAllFocReceived(vb.PopFrontInt16());
      break;

    case COMM_PING_CAN: {
      mTimeoutPingCan = 0;
      std::vector<int> devs;
      while (vb.size() > 0) {
        devs.push_back(vb.PopFrontUint8());
      }
      pingCanRx(devs, false);
    } break;

    case COMM_GET_IMU_DATA: {
      mTimeoutImuData = 0;

      IMU_VALUES values;

      uint32_t mask = vb.PopFrontUint16();

      if (mask & (static_cast<uint32_t>(1) << 0)) {
        values.roll = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 1)) {
        values.pitch = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 2)) {
        values.yaw = vb.PopFrontDouble32Auto();
      }

      if (mask & (static_cast<uint32_t>(1) << 3)) {
        values.accX = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 4)) {
        values.accY = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 5)) {
        values.accZ = vb.PopFrontDouble32Auto();
      }

      if (mask & (static_cast<uint32_t>(1) << 6)) {
        values.gyroX = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 7)) {
        values.gyroY = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 8)) {
        values.gyroZ = vb.PopFrontDouble32Auto();
      }

      if (mask & (static_cast<uint32_t>(1) << 9)) {
        values.magX = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 10)) {
        values.magY = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 11)) {
        values.magZ = vb.PopFrontDouble32Auto();
      }

      if (mask & (static_cast<uint32_t>(1) << 12)) {
        values.q0 = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 13)) {
        values.q1 = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 14)) {
        values.q2 = vb.PopFrontDouble32Auto();
      }
      if (mask & (static_cast<uint32_t>(1) << 15)) {
        values.q3 = vb.PopFrontDouble32Auto();
      }

      valuesImuReceived(values, mask);
    } break;

    case COMM_BM_CONNECT:
      bmConnRes(vb.PopFrontInt16());
      break;

    case COMM_BM_ERASE_FLASH_ALL:
    case COMM_BM_WRITE_FLASH:
      ROS_ERROR("This driver does not support flash modification. (rec: %u)",
                id);
      break;

    case COMM_BM_REBOOT:
      bmRebootRes(vb.PopFrontInt16());
      break;

    case COMM_BM_DISCONNECT:
      ackReceived("COMM_BM_DISCONNECT OK");
      break;

    default:
      break;
  }
}

void VescCommands::getFwVersion() {
  if (mTimeoutFwVer > 0) {
    return;
  }

  mTimeoutFwVer = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_FW_VERSION);
  emitData(vb);
}

void VescCommands::getValues() {
  if (mTimeoutValues > 0) {
    return;
  }

  mTimeoutValues = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_VALUES);
  emitData(vb);
}

void VescCommands::sendTerminalCmd(std::string cmd) {
  ByteArray vb;
  vb.AppendInt8(COMM_TERMINAL_CMD);
  vb.AppendString(cmd);
  emitData(vb);
}

void VescCommands::sendTerminalCmdSync(std::string cmd) {
  ByteArray vb;
  vb.AppendInt8(COMM_TERMINAL_CMD_SYNC);
  vb.AppendString(cmd);
  emitData(vb);
}

void VescCommands::setDutyCycle(double dutyCycle) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_DUTY);
  vb.AppendDouble32(dutyCycle, 1e5);
  emitData(vb);
}

void VescCommands::setCurrent(double current) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_CURRENT);
  vb.AppendDouble32(current, 1e3);
  emitData(vb);
}

void VescCommands::setCurrentBrake(double current) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_CURRENT_BRAKE);
  vb.AppendDouble32(current, 1e3);
  emitData(vb);
}

void VescCommands::setRpm(int rpm) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_RPM);
  vb.AppendInt32(rpm);
  emitData(vb);
}

void VescCommands::setPos(double pos) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_POS);
  vb.AppendDouble32(pos, 1e6);
  emitData(vb);
}

void VescCommands::setHandbrake(double current) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_HANDBRAKE);
  vb.AppendDouble32(current, 1e3);
  emitData(vb);
}

void VescCommands::setDetect(disp_pos_mode mode) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_DETECT);
  vb.AppendInt8(mode);
  emitData(vb);
}

void VescCommands::samplePrint(debug_sampling_mode mode, int sample_len,
                               int dec) {
  ByteArray vb;
  vb.AppendInt8(COMM_SAMPLE_PRINT);
  vb.AppendInt8(mode);
  vb.AppendUint16(sample_len);
  vb.AppendUint8(dec);
  emitData(vb);
}

void VescCommands::detectMotorParam(double current, double min_rpm,
                                    double low_duty) {
  ByteArray vb;
  vb.AppendInt8(COMM_DETECT_MOTOR_PARAM);
  vb.AppendDouble32(current, 1e3);
  vb.AppendDouble32(min_rpm, 1e3);
  vb.AppendDouble32(low_duty, 1e3);
  emitData(vb);
}

void VescCommands::reboot() {
  ByteArray vb;
  vb.AppendInt8(COMM_REBOOT);
  emitData(vb);
}

void VescCommands::sendAlive() {
  ByteArray vb;
  vb.AppendInt8(COMM_ALIVE);
  emitData(vb);
}

void VescCommands::getDecodedPpm() {
  if (mTimeoutDecPpm > 0) {
    return;
  }

  mTimeoutDecPpm = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_DECODED_PPM);
  emitData(vb);
}

void VescCommands::getDecodedAdc() {
  if (mTimeoutDecAdc > 0) {
    return;
  }

  mTimeoutDecAdc = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_DECODED_ADC);
  emitData(vb);
}

void VescCommands::getDecodedChuk() {
  if (mTimeoutDecChuk > 0) {
    return;
  }

  mTimeoutDecChuk = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_DECODED_CHUK);
  emitData(vb);
}

void VescCommands::setServoPos(double pos) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_SERVO_POS);
  vb.AppendDouble16(pos, 1e3);
  emitData(vb);
}

void VescCommands::measureRL() {
  ByteArray vb;
  vb.AppendInt8(COMM_DETECT_MOTOR_R_L);
  emitData(vb);
}

void VescCommands::measureLinkage(double current, double min_rpm,
                                  double low_duty, double resistance) {
  ByteArray vb;
  vb.AppendInt8(COMM_DETECT_MOTOR_FLUX_LINKAGE);
  vb.AppendDouble32(current, 1e3);
  vb.AppendDouble32(min_rpm, 1e3);
  vb.AppendDouble32(low_duty, 1e3);
  vb.AppendDouble32(resistance, 1e6);
  emitData(vb);
}

void VescCommands::measureEncoder(double current) {
  ByteArray vb;
  vb.AppendInt8(COMM_DETECT_ENCODER);
  vb.AppendDouble32(current, 1e3);
  emitData(vb);
}

void VescCommands::measureHallFoc(double current) {
  ByteArray vb;
  vb.AppendInt8(COMM_DETECT_HALL_FOC);
  vb.AppendDouble32(current, 1e3);
  emitData(vb);
}

void VescCommands::setChukData(chuck_data& data) {
  ByteArray vb;
  vb.AppendInt8(COMM_SET_CHUCK_DATA);
  vb.AppendUint8(data.js_x);
  vb.AppendUint8(data.js_y);
  vb.AppendUint8(data.bt_c);
  vb.AppendUint8(data.bt_z);
  vb.AppendInt16(data.acc_x);
  vb.AppendInt16(data.acc_y);
  vb.AppendInt16(data.acc_z);
  emitData(vb);
}

void VescCommands::pairNrf(int ms) {
  ByteArray vb;
  vb.AppendInt8(COMM_NRF_START_PAIRING);
  vb.AppendInt32(ms);
  emitData(vb);
}

void VescCommands::gpdSetFsw(float fsw) {
  ByteArray vb;
  vb.AppendInt8(COMM_GPD_SET_FSW);
  vb.AppendInt32((uint32_t)fsw);
  emitData(vb);
}

void VescCommands::getGpdBufferSizeLeft() {
  ByteArray vb;
  vb.AppendInt8(COMM_GPD_BUFFER_SIZE_LEFT);
  emitData(vb);
}

void VescCommands::gpdFillBuffer(std::deque<float> samples) {
  ByteArray vb;

  while (!samples.empty()) {
    vb.AppendDouble32Auto(samples.at(0));
    samples.pop_front();

    if (vb.size() > 400) {
      vb.push_front(COMM_GPD_FILL_BUFFER);
      emitData(vb);
      vb.clear();
    }
  }

  if (vb.size() > 0) {
    vb.push_front(COMM_GPD_FILL_BUFFER);
    emitData(vb);
  }
}

void VescCommands::gpdOutputSample(float sample) {
  ByteArray vb;
  vb.AppendInt8(COMM_GPD_OUTPUT_SAMPLE);
  vb.AppendDouble32Auto(sample);
  emitData(vb);
}

void VescCommands::gpdSetMode(gpd_output_mode mode) {
  ByteArray vb;
  vb.AppendInt8(COMM_GPD_SET_MODE);
  vb.AppendInt8(mode);
  emitData(vb);
}

void VescCommands::gpdFillBufferInt8(std::deque<int8_t> samples) {
  ByteArray vb;

  while (!samples.empty()) {
    vb.AppendInt8(samples.at(0));
    samples.pop_front();

    if (vb.size() > 400) {
      vb.push_front(COMM_GPD_FILL_BUFFER_INT8);
      emitData(vb);
      vb.clear();
    }
  }

  if (vb.size() > 0) {
    vb.push_front(COMM_GPD_FILL_BUFFER_INT8);
    emitData(vb);
  }
}

void VescCommands::gpdFillBufferInt16(std::deque<int16_t> samples) {
  ByteArray vb;

  while (!samples.empty()) {
    vb.AppendInt16(samples.at(0));
    samples.pop_front();

    if (vb.size() > 400) {
      vb.push_front(COMM_GPD_FILL_BUFFER_INT16);
      emitData(vb);
      vb.clear();
    }
  }

  if (vb.size() > 0) {
    vb.push_front(COMM_GPD_FILL_BUFFER_INT16);
    emitData(vb);
  }
}

void VescCommands::gpdSetBufferIntScale(float scale) {
  ByteArray vb;
  vb.AppendInt8(COMM_GPD_SET_BUFFER_INT_SCALE);
  vb.AppendDouble32Auto(scale);
  emitData(vb);
}

void VescCommands::getValuesSetup() {
  if (mTimeoutValuesSetup > 0) {
    return;
  }

  mTimeoutValuesSetup = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_VALUES_SETUP);
  emitData(vb);
}

void VescCommands::getValuesSelective(unsigned int mask) {
  if (mTimeoutValues > 0) {
    return;
  }

  mTimeoutValues = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_VALUES_SELECTIVE);
  vb.AppendUint32(mask);
  emitData(vb);
}

void VescCommands::getValuesSetupSelective(unsigned int mask) {
  if (mTimeoutValuesSetup > 0) {
    return;
  }

  mTimeoutValuesSetup = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_VALUES_SETUP_SELECTIVE);
  vb.AppendUint32(mask);
  emitData(vb);
}

void VescCommands::measureLinkageOpenloop(double current, double erpm_per_sec,
                                          double low_duty, double resistance) {
  ByteArray vb;
  vb.AppendInt8(COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP);
  vb.AppendDouble32(current, 1e3);
  vb.AppendDouble32(erpm_per_sec, 1e3);
  vb.AppendDouble32(low_duty, 1e3);
  vb.AppendDouble32(resistance, 1e6);
  emitData(vb);
}

void VescCommands::detectAllFoc(bool detect_can, double max_power_loss,
                                double min_current_in, double max_current_in,
                                double openloop_rpm, double sl_erpm) {
  ByteArray vb;
  vb.AppendInt8(COMM_DETECT_APPLY_ALL_FOC);
  vb.AppendInt8(detect_can);
  vb.AppendDouble32(max_power_loss, 1e3);
  vb.AppendDouble32(min_current_in, 1e3);
  vb.AppendDouble32(max_current_in, 1e3);
  vb.AppendDouble32(openloop_rpm, 1e3);
  vb.AppendDouble32(sl_erpm, 1e3);
  emitData(vb);
}

void VescCommands::pingCan() {
  if (mTimeoutPingCan > 0) {
    return;
  }

  mTimeoutPingCan = 500;

  ByteArray vb;
  vb.AppendInt8(COMM_PING_CAN);
  emitData(vb);
}

void VescCommands::getImuData(unsigned int mask) {
  if (mTimeoutImuData > 0) {
    return;
  }

  mTimeoutImuData = mTimeoutCount;

  ByteArray vb;
  vb.AppendInt8(COMM_GET_IMU_DATA);
  vb.AppendUint16(mask);
  emitData(vb);
}

void VescCommands::bmConnect() {
  ByteArray vb;
  vb.AppendInt8(COMM_BM_CONNECT);
  emitData(vb);
}

void VescCommands::bmReboot() {
  ByteArray vb;
  vb.AppendInt8(COMM_BM_REBOOT);
  emitData(vb);
}

void VescCommands::bmDisconnect() {
  ByteArray vb;
  vb.AppendInt8(COMM_BM_DISCONNECT);
  emitData(vb);
}

/** ros callback -> slot function **/

void VescCommands::getFwVersionCallback(
    const vesc_v6_driver::getFwVersion::ConstPtr& /*msg*/) {
  ROS_DEBUG("Received: %s", __func__);
  getFwVersion();
}
void VescCommands::getValuesCallback(
    const vesc_v6_driver::getValues::ConstPtr& /*msg*/) {
  getValues();
}
void VescCommands::sendTerminalCmdCallback(
    const vesc_v6_driver::sendTerminalCmd::ConstPtr& msg) {
  sendTerminalCmd(msg->cmd);
}
void VescCommands::sendTerminalCmdSyncCallback(
    const vesc_v6_driver::sendTerminalCmdSync::ConstPtr& msg) {
  sendTerminalCmdSync(msg->cmd);
}
void VescCommands::setDutyCycleCallback(
    const vesc_v6_driver::setDutyCycle::ConstPtr& msg) {
  setDutyCycle(msg->dutyCycle);
}
void VescCommands::setCurrentCallback(
    const vesc_v6_driver::setCurrent::ConstPtr& msg) {
  setCurrent(msg->current);
}
void VescCommands::setCurrentBrakeCallback(
    const vesc_v6_driver::setCurrentBrake::ConstPtr& msg) {
  setCurrentBrake(msg->current);
}
void VescCommands::setRpmCallback(const vesc_v6_driver::setRpm::ConstPtr& msg) {
  setRpm(msg->rpm);
}
void VescCommands::setPosCallback(const vesc_v6_driver::setPos::ConstPtr& msg) {
  setPos(msg->pos);
}
void VescCommands::setHandbrakeCallback(
    const vesc_v6_driver::setHandbrake::ConstPtr& msg) {
  setHandbrake(msg->current);
}
void VescCommands::setDetectCallback(
    const vesc_v6_driver::setDetect::ConstPtr& msg) {
  setDetect(static_cast<disp_pos_mode>(msg->mode));
}
void VescCommands::samplePrintCallback(
    const vesc_v6_driver::samplePrint::ConstPtr& msg) {
  samplePrint(static_cast<debug_sampling_mode>(msg->mode), msg->sample_len,
              msg->dec);
}
void VescCommands::detectMotorParamCallback(
    const vesc_v6_driver::detectMotorParam::ConstPtr& msg) {
  detectMotorParam(msg->current, msg->min_rpm, msg->low_duty);
}
void VescCommands::rebootCallback(
    const vesc_v6_driver::reboot::ConstPtr& /*msg*/) {
  reboot();
}
void VescCommands::sendAliveCallback(
    const vesc_v6_driver::sendAlive::ConstPtr& /*msg*/) {
  sendAlive();
}
void VescCommands::getDecodedPpmCallback(
    const vesc_v6_driver::getDecodedPpm::ConstPtr& /*msg*/) {
  getDecodedPpm();
}
void VescCommands::getDecodedAdcCallback(
    const vesc_v6_driver::getDecodedAdc::ConstPtr& /*msg*/) {
  getDecodedAdc();
}
void VescCommands::getDecodedChukCallback(
    const vesc_v6_driver::getDecodedChuk::ConstPtr& /*msg*/) {
  getDecodedChuk();
}
void VescCommands::setServoPosCallback(
    const vesc_v6_driver::setServoPos::ConstPtr& msg) {
  setServoPos(msg->pos);
}
void VescCommands::measureRLCallback(
    const vesc_v6_driver::measureRL::ConstPtr& /*msg*/) {
  measureRL();
}
void VescCommands::measureLinkageCallback(
    const vesc_v6_driver::measureLinkage::ConstPtr& msg) {
  measureLinkage(msg->current, msg->min_rpm, msg->low_duty, msg->resistance);
}
void VescCommands::measureEncoderCallback(
    const vesc_v6_driver::measureEncoder::ConstPtr& msg) {
  measureEncoder(msg->current);
}
void VescCommands::measureHallFocCallback(
    const vesc_v6_driver::measureHallFoc::ConstPtr& msg) {
  measureHallFoc(msg->current);
}
void VescCommands::setChukDataCallback(
    const vesc_v6_driver::setChukData::ConstPtr& msg) {
  chuck_data data;
  data.bt_c = msg->bt_c;
  data.bt_z = msg->bt_z;
  data.js_x = msg->js_x;
  data.js_y = msg->js_y;
  data.acc_x = msg->acc_x;
  data.acc_y = msg->acc_y;
  data.acc_z = msg->acc_z;
  setChukData(data);
}
void VescCommands::pairNrfCallback(
    const vesc_v6_driver::pairNrf::ConstPtr& msg) {
  pairNrf(msg->ms);
}
void VescCommands::gpdSetFswCallback(
    const vesc_v6_driver::gpdSetFsw::ConstPtr& msg) {
  gpdSetFsw(msg->fsw);
}
void VescCommands::getGpdBufferSizeLeftCallback(
    const vesc_v6_driver::getGpdBufferSizeLeft::ConstPtr& /*msg*/) {
  getGpdBufferSizeLeft();
}
void VescCommands::gpdFillBufferCallback(
    const vesc_v6_driver::gpdFillBuffer::ConstPtr& msg) {
  std::deque<float> samples(msg->samples.begin(), msg->samples.end());
  gpdFillBuffer(samples);
}
void VescCommands::gpdOutputSampleCallback(
    const vesc_v6_driver::gpdOutputSample::ConstPtr& msg) {
  gpdOutputSample(msg->sample);
}
void VescCommands::gpdSetModeCallback(
    const vesc_v6_driver::gpdSetMode::ConstPtr& msg) {
  gpdSetMode(static_cast<gpd_output_mode>(msg->mode));
}
void VescCommands::gpdFillBufferInt8Callback(
    const vesc_v6_driver::gpdFillBufferInt8::ConstPtr& msg) {
  std::deque<int8_t> samples(msg->samples.begin(), msg->samples.end());
  gpdFillBufferInt8(samples);
}
void VescCommands::gpdFillBufferInt16Callback(
    const vesc_v6_driver::gpdFillBufferInt16::ConstPtr& msg) {
  std::deque<int16_t> samples(msg->samples.begin(), msg->samples.end());
  gpdFillBufferInt16(samples);
}
void VescCommands::gpdSetBufferIntScaleCallback(
    const vesc_v6_driver::gpdSetBufferIntScale::ConstPtr& msg) {
  gpdSetBufferIntScale(msg->scale);
}
void VescCommands::getValuesSetupCallback(
    const vesc_v6_driver::getValuesSetup::ConstPtr& /*msg*/) {
  getValuesSetup();
}
void VescCommands::getValuesSelectiveCallback(
    const vesc_v6_driver::getValuesSelective::ConstPtr& msg) {
  getValuesSelective(msg->mask);
}
void VescCommands::getValuesSetupSelectiveCallback(
    const vesc_v6_driver::getValuesSetupSelective::ConstPtr& msg) {
  getValuesSetupSelective(msg->mask);
}
void VescCommands::measureLinkageOpenloopCallback(
    const vesc_v6_driver::measureLinkageOpenloop::ConstPtr& msg) {
  measureLinkageOpenloop(msg->current, msg->erpm_per_sec, msg->low_duty,
                         msg->resistance);
}
void VescCommands::detectAllFocCallback(
    const vesc_v6_driver::detectAllFoc::ConstPtr& msg) {
  detectAllFoc(msg->detect_can, msg->max_power_loss, msg->min_current_in,
               msg->max_current_in, msg->openloop_rpm, msg->sl_erpm);
}
void VescCommands::pingCanCallback(
    const vesc_v6_driver::pingCan::ConstPtr& /*msg*/) {
  pingCan();
}
void VescCommands::getImuDataCallback(
    const vesc_v6_driver::getImuData::ConstPtr& msg) {
  getImuData(msg->mask);
}
void VescCommands::bmConnectCallback(
    const vesc_v6_driver::bmConnect::ConstPtr& /*msg*/) {
  bmConnect();
}
void VescCommands::bmRebootCallback(
    const vesc_v6_driver::bmReboot::ConstPtr& /*msg*/) {
  bmReboot();
}
void VescCommands::bmDisconnectCallback(
    const vesc_v6_driver::bmDisconnect::ConstPtr& /*msg*/) {
  bmDisconnect();
}

void VescCommands::timerSlot(const ros::TimerEvent& /*event*/) {
  if (mTimeoutFwVer > 0) mTimeoutFwVer--;
  if (mTimeoutValues > 0) mTimeoutValues--;
  if (mTimeoutValuesSetup > 0) mTimeoutValuesSetup--;
  if (mTimeoutImuData > 0) mTimeoutImuData--;
  if (mTimeoutDecPpm > 0) mTimeoutDecPpm--;
  if (mTimeoutDecAdc > 0) mTimeoutDecAdc--;
  if (mTimeoutDecChuk > 0) mTimeoutDecChuk--;

  if (mTimeoutPingCan > 0) {
    mTimeoutPingCan--;

    if (mTimeoutPingCan == 0) {
      pingCanRx(std::vector<int>(), true);
    }
  }
}

void VescCommands::emitData(ByteArray& data) const {
  // Only allow firmware commands in limited mode
  if (mIsLimitedMode && data.at(0) > COMM_WRITE_NEW_APP_DATA) {
    if (!mLimitedSupportsFwdAllCan ||
        (data.at(0) != COMM_JUMP_TO_BOOTLOADER_ALL_CAN &&
         data.at(0) != COMM_ERASE_NEW_APP_ALL_CAN &&
         data.at(0) != COMM_WRITE_NEW_APP_DATA_ALL_CAN)) {
      return;
    }
  }

  if (mSendCan) {
    data.push_front((char)mCanId);
    data.push_front((char)COMM_FORWARD_CAN);
  }

  VescPacket::instance()->sendPacket(data);
}

std::string VescCommands::faultToStr(mc_fault_code fault) {
  switch (fault) {
    case FAULT_CODE_NONE:
      return "FAULT_CODE_NONE";
    case FAULT_CODE_OVER_VOLTAGE:
      return "FAULT_CODE_OVER_VOLTAGE";
    case FAULT_CODE_UNDER_VOLTAGE:
      return "FAULT_CODE_UNDER_VOLTAGE";
    case FAULT_CODE_DRV:
      return "FAULT_CODE_DRV";
    case FAULT_CODE_ABS_OVER_CURRENT:
      return "FAULT_CODE_ABS_OVER_CURRENT";
    case FAULT_CODE_OVER_TEMP_FET:
      return "FAULT_CODE_OVER_TEMP_FET";
    case FAULT_CODE_OVER_TEMP_MOTOR:
      return "FAULT_CODE_OVER_TEMP_MOTOR";
    case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE:
      return "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE";
    case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE:
      return "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE";
    case FAULT_CODE_MCU_UNDER_VOLTAGE:
      return "FAULT_CODE_MCU_UNDER_VOLTAGE";
    case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET:
      return "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET";
    case FAULT_CODE_ENCODER_SPI:
      return "FAULT_CODE_ENCODER_SPI";
    case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE:
      return "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
    case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE:
      return "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
    case FAULT_CODE_FLASH_CORRUPTION:
      return "FAULT_CODE_FLASH_CORRUPTION";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
      return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
      return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:
      return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
    case FAULT_CODE_UNBALANCED_CURRENTS:
      return "FAULT_CODE_UNBALANCED_CURRENTS";
      // default: return "Unknown fault";
  }
}

bool VescCommands::getLimitedSupportsFwdAllCan() const {
  return mLimitedSupportsFwdAllCan;
}

void VescCommands::setLimitedSupportsFwdAllCan(bool limitedSupportsFwdAllCan) {
  mLimitedSupportsFwdAllCan = limitedSupportsFwdAllCan;
}

void VescCommands::fwVersionReceived(const int8_t major, const int8_t minor,
                                     const std::string& hw,
                                     const ByteArray& uuid,
                                     const bool isPaired) const {
  vesc_v6_driver::fwVersionReceived::Ptr result(
      new vesc_v6_driver::fwVersionReceived);
  result->fw_major = major;
  result->fw_minor = minor;
  result->hw = hw;
  result->uuid.insert(result->uuid.end(), uuid.begin(), uuid.end());
  result->isPaired = isPaired;
  fwVersionReceived_pub.publish(result);
}

void VescCommands::ackReceived(const std::string& ackType) const {
  vesc_v6_driver::ackReceived::Ptr result(new vesc_v6_driver::ackReceived);
  result->ackType = ackType;
  ackReceived_pub.publish(result);
}

void VescCommands::valuesReceived(const MC_VALUES& values,
                                  const unsigned int mask) const {
  vesc_v6_driver::valuesReceived::Ptr result(
      new vesc_v6_driver::valuesReceived);
  result->v_in = values.v_in;
  result->temp_mos = values.temp_mos;
  result->temp_mos_1 = values.temp_mos_1;
  result->temp_mos_2 = values.temp_mos_2;
  result->temp_mos_3 = values.temp_mos_3;
  result->temp_motor = values.temp_motor;
  result->current_motor = values.current_motor;
  result->current_in = values.current_in;
  result->id = values.id;
  result->iq = values.iq;
  result->rpm = values.rpm;
  result->duty_now = values.duty_now;
  result->amp_hours = values.amp_hours;
  result->amp_hours_charged = values.amp_hours_charged;
  result->watt_hours = values.watt_hours;
  result->watt_hours_charged = values.watt_hours_charged;
  result->tachometer = values.tachometer;
  result->tachometer_abs = values.tachometer_abs;
  result->position = values.position;
  result->fault_code = values.fault_code;
  result->vesc_id = values.vesc_id;
  result->mask = mask;
  valuesReceived_pub.publish(result);
}

void VescCommands::printReceived(const std::string& str) const {
  vesc_v6_driver::printReceived::Ptr result(new vesc_v6_driver::printReceived);
  result->str = str;
  printReceived_pub.publish(result);
}

void VescCommands::samplesReceived(const ByteArray& bytes) const {
  vesc_v6_driver::samplesReceived::Ptr result(
      new vesc_v6_driver::samplesReceived);
  result->bytes.insert(result->bytes.end(), bytes.begin(), bytes.end());
  samplesReceived_pub.publish(result);
}

void VescCommands::rotorPosReceived(const double pos) const {
  vesc_v6_driver::rotorPosReceived::Ptr result(
      new vesc_v6_driver::rotorPosReceived);
  result->pos = pos;
  rotorPosReceived_pub.publish(result);
}

void VescCommands::experimentSamplesReceived(
    const std::vector<double>& samples) const {
  vesc_v6_driver::experimentSamplesReceived::Ptr result(
      new vesc_v6_driver::experimentSamplesReceived);
  result->samples.insert(result->samples.end(), samples.begin(), samples.end());
  experimentSamplesReceived_pub.publish(result);
}

void VescCommands::bldcDetectReceived(const bldc_detect& param) const {
  vesc_v6_driver::bldcDetectReceived::Ptr result(
      new vesc_v6_driver::bldcDetectReceived);
  result->cycle_int_limit = param.cycle_int_limit;
  result->bemf_coupling_k = param.bemf_coupling_k;
  result->hall_table.insert(result->hall_table.end(), param.hall_table.begin(),
                            param.hall_table.end());
  result->hall_res = param.hall_res;
  bldcDetectReceived_pub.publish(result);
}

void VescCommands::decodedPpmReceived(const double value,
                                      const double last_len) const {
  vesc_v6_driver::decodedPpmReceived::Ptr result(
      new vesc_v6_driver::decodedPpmReceived);
  result->value = value;
  result->last_len = last_len;
  decodedPpmReceived_pub.publish(result);
}

void VescCommands::decodedAdcReceived(const double value, const double voltage,
                                      const double value2,
                                      const double voltage2) const {
  vesc_v6_driver::decodedAdcReceived::Ptr result(
      new vesc_v6_driver::decodedAdcReceived);
  result->value = value;
  result->voltage = voltage;
  result->value2 = value2;
  result->voltage2 = voltage2;
  decodedAdcReceived_pub.publish(result);
}

void VescCommands::decodedChukReceived(const double value) const {
  vesc_v6_driver::decodedChukReceived::Ptr result(
      new vesc_v6_driver::decodedChukReceived);
  result->value = value;
  decodedChukReceived_pub.publish(result);
}

void VescCommands::motorRLReceived(const double r, const double l) const {
  vesc_v6_driver::motorRLReceived::Ptr result(
      new vesc_v6_driver::motorRLReceived);
  result->r = r;
  result->l = l;
  motorRLReceived_pub.publish(result);
}

void VescCommands::motorLinkageReceived(const double flux_linkage) const {
  vesc_v6_driver::motorLinkageReceived::Ptr result(
      new vesc_v6_driver::motorLinkageReceived);
  result->flux_linkage = flux_linkage;
  motorLinkageReceived_pub.publish(result);
}

void VescCommands::encoderParamReceived(const double offset, const double ratio,
                                        const bool inverted) const {
  vesc_v6_driver::encoderParamReceived::Ptr result(
      new vesc_v6_driver::encoderParamReceived);
  result->offset = offset;
  result->ratio = ratio;
  result->inverted = inverted;
  encoderParamReceived_pub.publish(result);
}

void VescCommands::focHallTableReceived(const std::vector<int>& hall_table,
                                        const int8_t res) const {
  vesc_v6_driver::focHallTableReceived::Ptr result(
      new vesc_v6_driver::focHallTableReceived);
  result->hall_table.insert(result->hall_table.end(), hall_table.begin(),
                            hall_table.end());
  result->res = res;
  focHallTableReceived_pub.publish(result);
}

void VescCommands::nrfPairingRes(const int8_t res) const {
  vesc_v6_driver::nrfPairingRes::Ptr result(new vesc_v6_driver::nrfPairingRes);
  result->res = res;
  nrfPairingRes_pub.publish(result);
}

void VescCommands::gpdBufferNotifyReceived() const {
  vesc_v6_driver::gpdBufferNotifyReceived::Ptr result(
      new vesc_v6_driver::gpdBufferNotifyReceived);
  gpdBufferNotifyReceived_pub.publish(result);
}

void VescCommands::gpdBufferSizeLeftReceived(const int16_t sizeLeft) const {
  vesc_v6_driver::gpdBufferSizeLeftReceived::Ptr result(
      new vesc_v6_driver::gpdBufferSizeLeftReceived);
  result->sizeLeft = sizeLeft;
  gpdBufferSizeLeftReceived_pub.publish(result);
}

void VescCommands::valuesSetupReceived(const SETUP_VALUES& values,
                                       const unsigned int mask) const {
  vesc_v6_driver::valuesSetupReceived::Ptr result(
      new vesc_v6_driver::valuesSetupReceived);
  result->temp_mos = values.temp_mos;
  result->temp_motor = values.temp_motor;
  result->current_motor = values.current_motor;
  result->current_in = values.current_in;
  result->duty_now = values.duty_now;
  result->rpm = values.rpm;
  result->speed = values.speed;
  result->v_in = values.v_in;
  result->battery_level = values.battery_level;
  result->amp_hours = values.amp_hours;
  result->amp_hours_charged = values.amp_hours_charged;
  result->watt_hours = values.watt_hours;
  result->watt_hours_charged = values.watt_hours_charged;
  result->tachometer = values.tachometer;
  result->tachometer_abs = values.tachometer_abs;
  result->position = values.position;
  result->fault_code = values.fault_code;
  result->vesc_id = values.vesc_id;
  result->num_vescs = values.num_vescs;
  result->battery_wh = values.battery_wh;
  result->fault_str = values.fault_str;
  result->mask = mask;
  valuesSetupReceived_pub.publish(result);
}

void VescCommands::detectAllFocReceived(const int16_t result) const {
  vesc_v6_driver::detectAllFocReceived::Ptr result_(
      new vesc_v6_driver::detectAllFocReceived);
  result_->result = result;
  detectAllFocReceived_pub.publish(result_);
}

void VescCommands::pingCanRx(const std::vector<int>& devs,
                             const bool isTimeout) const {
  vesc_v6_driver::pingCanRx::Ptr result(new vesc_v6_driver::pingCanRx);
  result->devs.insert(result->devs.end(), devs.begin(), devs.end());
  result->isTimeout = isTimeout;
  pingCanRx_pub.publish(result);
}

void VescCommands::valuesImuReceived(const IMU_VALUES& values,
                                     const unsigned int mask) const {
  vesc_v6_driver::valuesImuReceived::Ptr result(
      new vesc_v6_driver::valuesImuReceived);
  result->roll = values.roll;
  result->pitch = values.pitch;
  result->yaw = values.yaw;
  result->accX = values.accX;
  result->accY = values.accY;
  result->accZ = values.accZ;
  result->gyroX = values.gyroX;
  result->gyroY = values.gyroY;
  result->gyroZ = values.gyroZ;
  result->magX = values.magX;
  result->magY = values.magY;
  result->magZ = values.magZ;
  result->q0 = values.q0;
  result->q1 = values.q1;
  result->q2 = values.q2;
  result->q3 = values.q3;
  result->mask = mask;
  valuesImuReceived_pub.publish(result);
}

void VescCommands::bmConnRes(const int16_t res) const {
  vesc_v6_driver::bmConnRes::Ptr result(new vesc_v6_driver::bmConnRes);
  result->res = res;
  bmConnRes_pub.publish(result);
}

void VescCommands::bmRebootRes(const int16_t res) const {
  vesc_v6_driver::bmRebootRes::Ptr result(new vesc_v6_driver::bmRebootRes);
  result->res = res;
  bmRebootRes_pub.publish(result);
}

void VescCommands::emitEmptyValues() {
  MC_VALUES values;
  values.temp_mos = 0.0;
  values.temp_motor = 0.0;
  values.current_motor = 0.0;
  values.current_in = 0.0;
  values.id = 0.0;
  values.iq = 0.0;
  values.duty_now = 0.0;
  values.rpm = 0.0;
  values.v_in = 45.0;
  values.amp_hours = 0.0;
  values.amp_hours_charged = 0.0;
  values.watt_hours = 0.0;
  values.watt_hours_charged = 0.0;
  values.tachometer = 0;
  values.tachometer_abs = 0;
  values.fault_code = FAULT_CODE_NONE;
  values.fault_str = faultToStr(values.fault_code);
  values.position = 0.0;
  values.vesc_id = 0;

  valuesReceived(values, 0xFFFFFFFF);
}

void VescCommands::emitEmptySetupValues() {
  SETUP_VALUES values;
  values.temp_mos = 0.0;
  values.temp_motor = 0.0;
  values.current_motor = 0.0;
  values.current_in = 0.0;
  values.duty_now = 0.0;
  values.rpm = 0.0;
  values.speed = 0.0;
  values.v_in = 45.0;
  values.battery_level = 0.0;
  values.amp_hours = 0.0;
  values.amp_hours_charged = 0.0;
  values.watt_hours = 0.0;
  values.watt_hours_charged = 0.0;
  values.tachometer = 0.0;
  values.tachometer_abs = 0.0;
  values.position = 0.0;
  values.fault_code = FAULT_CODE_NONE;
  values.fault_str = faultToStr(values.fault_code);
  values.vesc_id = 0;
  values.num_vescs = 1;
  values.battery_wh = 0.0;

  valuesSetupReceived(values, 0xFFFFFFFF);
}
