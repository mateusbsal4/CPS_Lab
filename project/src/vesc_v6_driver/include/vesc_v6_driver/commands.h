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

#ifndef VESC_V6_COMMANDS_H
#define VESC_V6_COMMANDS_H

#include "vesc_v6_driver/bytearray.h"
#include "vesc_v6_driver/datatypes.h"
#include "vesc_v6_driver/singleton.h"

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/timer.h>

// sub msg
#include "vesc_v6_driver/bmConnect.h"
#include "vesc_v6_driver/bmDisconnect.h"
#include "vesc_v6_driver/bmReboot.h"
#include "vesc_v6_driver/detectAllFoc.h"
#include "vesc_v6_driver/detectMotorParam.h"
#include "vesc_v6_driver/getDecodedAdc.h"
#include "vesc_v6_driver/getDecodedChuk.h"
#include "vesc_v6_driver/getDecodedPpm.h"
#include "vesc_v6_driver/getFwVersion.h"
#include "vesc_v6_driver/getGpdBufferSizeLeft.h"
#include "vesc_v6_driver/getImuData.h"
#include "vesc_v6_driver/getValues.h"
#include "vesc_v6_driver/getValuesSelective.h"
#include "vesc_v6_driver/getValuesSetup.h"
#include "vesc_v6_driver/getValuesSetupSelective.h"
#include "vesc_v6_driver/gpdFillBuffer.h"
#include "vesc_v6_driver/gpdFillBufferInt16.h"
#include "vesc_v6_driver/gpdFillBufferInt8.h"
#include "vesc_v6_driver/gpdOutputSample.h"
#include "vesc_v6_driver/gpdSetBufferIntScale.h"
#include "vesc_v6_driver/gpdSetFsw.h"
#include "vesc_v6_driver/gpdSetMode.h"
#include "vesc_v6_driver/measureEncoder.h"
#include "vesc_v6_driver/measureHallFoc.h"
#include "vesc_v6_driver/measureLinkage.h"
#include "vesc_v6_driver/measureLinkageOpenloop.h"
#include "vesc_v6_driver/measureRL.h"
#include "vesc_v6_driver/pairNrf.h"
#include "vesc_v6_driver/pingCan.h"
#include "vesc_v6_driver/reboot.h"
#include "vesc_v6_driver/samplePrint.h"
#include "vesc_v6_driver/sendAlive.h"
#include "vesc_v6_driver/sendTerminalCmd.h"
#include "vesc_v6_driver/sendTerminalCmdSync.h"
#include "vesc_v6_driver/setChukData.h"
#include "vesc_v6_driver/setCurrent.h"
#include "vesc_v6_driver/setCurrentBrake.h"
#include "vesc_v6_driver/setDetect.h"
#include "vesc_v6_driver/setDutyCycle.h"
#include "vesc_v6_driver/setHandbrake.h"
#include "vesc_v6_driver/setPos.h"
#include "vesc_v6_driver/setRpm.h"
#include "vesc_v6_driver/setServoPos.h"

/**
 * Note: Commands for changing the VESC configurations are not supported.
 * Neither is firmware uploading.
 * @brief The VescCommands class
 */
class VescCommands : public Singleton<VescCommands> {
  friend class Singleton<VescCommands>;

 public:
  void Setup(ros::NodeHandle nh);

  void setLimitedMode(bool is_limited);
  bool isLimitedMode();
  bool setSendCan(bool sendCan, int id = -1);
  bool getSendCan();
  void setCanSendId(unsigned int id);
  int getCanSendId();
  // not supported: void setMcConfig(ConfigParams *mcConfig);
  // not supported: void setAppConfig(ConfigParams *appConfig);
  // not supported: void startFirmwareUpload(ByteArray &newFirmware, bool
  // isBootloader = false, bool fwdCan = false); not supported: double
  // getFirmwareUploadProgress(); not supported: std::string
  // getFirmwareUploadStatus(); not supported: void cancelFirmwareUpload(); not
  // supported: void checkMcConfig();
  void emitEmptyValues();
  void emitEmptySetupValues();
  bool getLimitedSupportsFwdAllCan() const;
  void setLimitedSupportsFwdAllCan(bool limitedSupportsFwdAllCan);

  void processPacket(ByteArray& data);

 private:  // signals
           //  void dataToSend(const ByteArray &data) const;
  void fwVersionReceived(const int8_t major, const int8_t minor,
                         const std::string& hw, const ByteArray& uuid,
                         const bool isPaired) const;
  void ackReceived(const std::string& ackType) const;
  void valuesReceived(const MC_VALUES& values, const unsigned int mask) const;
  void printReceived(const std::string& str) const;
  void samplesReceived(const ByteArray& bytes) const;
  void rotorPosReceived(const double pos) const;
  void experimentSamplesReceived(const std::vector<double>& samples) const;
  void bldcDetectReceived(const bldc_detect& param) const;
  void decodedPpmReceived(const double value, const double last_len) const;
  void decodedAdcReceived(const double value, const double voltage,
                          const double value2, const double voltage2) const;
  void decodedChukReceived(const double value) const;
  void motorRLReceived(const double r, const double l) const;
  void motorLinkageReceived(const double flux_linkage) const;
  void encoderParamReceived(const double offset, const double ratio,
                            const bool inverted) const;
  // not supported: void customAppDataReceived(ByteArray data);
  void focHallTableReceived(const std::vector<int>& hall_table,
                            const int8_t res) const;
  void nrfPairingRes(const int8_t res) const;
  // not supported: void mcConfigCheckResult(std::vector<std::string>
  // paramsNotSet);
  void gpdBufferNotifyReceived() const;
  void gpdBufferSizeLeftReceived(const int16_t sizeLeft) const;
  void valuesSetupReceived(const SETUP_VALUES& values,
                           const unsigned int mask) const;
  void detectAllFocReceived(const int16_t result) const;
  void pingCanRx(const std::vector<int>& devs, const bool isTimeout) const;
  void valuesImuReceived(const IMU_VALUES& values,
                         const unsigned int mask) const;
  void bmConnRes(const int16_t res) const;
  // not supported: void bmEraseFlashAllRes(int res);
  // not supported: void bmWriteFlashRes(int res);
  void bmRebootRes(const int16_t res) const;

 private:  // slots
  void getFwVersion();
  void getValues();
  void sendTerminalCmd(std::string cmd);
  void sendTerminalCmdSync(std::string cmd);
  void setDutyCycle(double dutyCycle);
  void setCurrent(double current);
  void setCurrentBrake(double current);
  void setRpm(int rpm);
  void setPos(double pos);
  void setHandbrake(double current);
  void setDetect(disp_pos_mode mode);
  void samplePrint(debug_sampling_mode mode, int sample_len, int dec);
  // not supported: void getMcconf();
  // not supported: void getMcconfDefault();
  // not supported: void setMcconf(bool check = true);
  // not supported: void getAppConf();
  // not supported: void getAppConfDefault();
  // not supported: void setAppConf();
  void detectMotorParam(double current, double min_rpm, double low_duty);
  void reboot();
  void sendAlive();
  void getDecodedPpm();
  void getDecodedAdc();
  void getDecodedChuk();
  void setServoPos(double pos);
  void measureRL();
  void measureLinkage(double current, double min_rpm, double low_duty,
                      double resistance);
  void measureEncoder(double current);
  void measureHallFoc(double current);
  // not supported: void sendCustomAppData(ByteArray data);
  // not supported: void sendCustomAppData(unsigned char *data, unsigned int
  // len);
  void setChukData(chuck_data& data);
  void pairNrf(int ms);
  void gpdSetFsw(float fsw);
  void getGpdBufferSizeLeft();
  void gpdFillBuffer(std::deque<float> samples);
  void gpdOutputSample(float sample);
  void gpdSetMode(gpd_output_mode mode);
  void gpdFillBufferInt8(std::deque<int8_t> samples);
  void gpdFillBufferInt16(std::deque<int16_t> samples);
  void gpdSetBufferIntScale(float scale);
  void getValuesSetup();
  // not supported: void setMcconfTemp(const MCCONF_TEMP &conf, bool is_setup,
  // bool store,
  //                                  bool forward_can, bool
  //                                  divide_by_controllers, bool ack);
  void getValuesSelective(unsigned int mask);
  void getValuesSetupSelective(unsigned int mask);
  void measureLinkageOpenloop(double current, double erpm_per_sec,
                              double low_duty, double resistance);
  void detectAllFoc(bool detect_can, double max_power_loss,
                    double min_current_in, double max_current_in,
                    double openloop_rpm, double sl_erpm);
  void pingCan();
  // not supported: void disableAppOutput(int time_ms, bool fwdCan);
  void getImuData(unsigned int mask);
  void bmConnect();
  // not supported: void bmEraseFlashAll();
  // not supported: void bmWriteFlash(uint32_t addr, ByteArray data);
  void bmReboot();
  void bmDisconnect();

 public:  // ros callback -> slot
  void getFwVersionCallback(const vesc_v6_driver::getFwVersion::ConstPtr& msg);
  void getValuesCallback(const vesc_v6_driver::getValues::ConstPtr& msg);
  void sendTerminalCmdCallback(
      const vesc_v6_driver::sendTerminalCmd::ConstPtr& msg);
  void sendTerminalCmdSyncCallback(
      const vesc_v6_driver::sendTerminalCmdSync::ConstPtr& msg);
  void setDutyCycleCallback(const vesc_v6_driver::setDutyCycle::ConstPtr& msg);
  void setCurrentCallback(const vesc_v6_driver::setCurrent::ConstPtr& msg);
  void setCurrentBrakeCallback(
      const vesc_v6_driver::setCurrentBrake::ConstPtr& msg);
  void setRpmCallback(const vesc_v6_driver::setRpm::ConstPtr& msg);
  void setPosCallback(const vesc_v6_driver::setPos::ConstPtr& msg);
  void setHandbrakeCallback(const vesc_v6_driver::setHandbrake::ConstPtr& msg);
  void setDetectCallback(const vesc_v6_driver::setDetect::ConstPtr& msg);
  void samplePrintCallback(const vesc_v6_driver::samplePrint::ConstPtr& msg);
  void detectMotorParamCallback(
      const vesc_v6_driver::detectMotorParam::ConstPtr& msg);
  void rebootCallback(const vesc_v6_driver::reboot::ConstPtr& msg);
  void sendAliveCallback(const vesc_v6_driver::sendAlive::ConstPtr& msg);
  void getDecodedPpmCallback(
      const vesc_v6_driver::getDecodedPpm::ConstPtr& msg);
  void getDecodedAdcCallback(
      const vesc_v6_driver::getDecodedAdc::ConstPtr& msg);
  void getDecodedChukCallback(
      const vesc_v6_driver::getDecodedChuk::ConstPtr& msg);
  void setServoPosCallback(const vesc_v6_driver::setServoPos::ConstPtr& msg);
  void measureRLCallback(const vesc_v6_driver::measureRL::ConstPtr& msg);
  void measureLinkageCallback(
      const vesc_v6_driver::measureLinkage::ConstPtr& msg);
  void measureEncoderCallback(
      const vesc_v6_driver::measureEncoder::ConstPtr& msg);
  void measureHallFocCallback(
      const vesc_v6_driver::measureHallFoc::ConstPtr& msg);
  void setChukDataCallback(const vesc_v6_driver::setChukData::ConstPtr& msg);
  void pairNrfCallback(const vesc_v6_driver::pairNrf::ConstPtr& msg);
  void gpdSetFswCallback(const vesc_v6_driver::gpdSetFsw::ConstPtr& msg);
  void getGpdBufferSizeLeftCallback(
      const vesc_v6_driver::getGpdBufferSizeLeft::ConstPtr& msg);
  void gpdFillBufferCallback(
      const vesc_v6_driver::gpdFillBuffer::ConstPtr& msg);
  void gpdOutputSampleCallback(
      const vesc_v6_driver::gpdOutputSample::ConstPtr& msg);
  void gpdSetModeCallback(const vesc_v6_driver::gpdSetMode::ConstPtr& msg);
  void gpdFillBufferInt8Callback(
      const vesc_v6_driver::gpdFillBufferInt8::ConstPtr& msg);
  void gpdFillBufferInt16Callback(
      const vesc_v6_driver::gpdFillBufferInt16::ConstPtr& msg);
  void gpdSetBufferIntScaleCallback(
      const vesc_v6_driver::gpdSetBufferIntScale::ConstPtr& msg);
  void getValuesSetupCallback(
      const vesc_v6_driver::getValuesSetup::ConstPtr& msg);
  void getValuesSelectiveCallback(
      const vesc_v6_driver::getValuesSelective::ConstPtr& msg);
  void getValuesSetupSelectiveCallback(
      const vesc_v6_driver::getValuesSetupSelective::ConstPtr& msg);
  void measureLinkageOpenloopCallback(
      const vesc_v6_driver::measureLinkageOpenloop::ConstPtr& msg);
  void detectAllFocCallback(const vesc_v6_driver::detectAllFoc::ConstPtr& msg);
  void pingCanCallback(const vesc_v6_driver::pingCan::ConstPtr& msg);
  void getImuDataCallback(const vesc_v6_driver::getImuData::ConstPtr& msg);
  void bmConnectCallback(const vesc_v6_driver::bmConnect::ConstPtr& msg);
  void bmRebootCallback(const vesc_v6_driver::bmReboot::ConstPtr& msg);
  void bmDisconnectCallback(const vesc_v6_driver::bmDisconnect::ConstPtr& msg);

 private:  // slots
  void timerSlot(const ros::TimerEvent& event);

 private:
  VescCommands() = default;

  void emitData(ByteArray& data) const;
  // not supported: void firmwareUploadUpdate(bool isTimeout);
  std::string faultToStr(mc_fault_code fault);

  ros::Timer mTimer;
  bool mSendCan;
  int mCanId;
  bool mIsLimitedMode;
  bool mLimitedSupportsFwdAllCan;

  bool mIsSetup = false;

  // FW upload state
  // not supported: ByteArray mNewFirmware;
  // not supported: bool mFirmwareIsUploading;
  // not supported: int mFirmwareState;
  // not supported: int mFimwarePtr;
  // not supported: int mFirmwareTimer;
  // not supported: int mFirmwareRetries;
  // not supported: bool mFirmwareIsBootloader;
  // not supported: bool mFirmwareFwdAllCan;
  // not supported: std::string mFirmwareUploadStatus;

  // not supported: ConfigParams *mMcConfig;
  // not supported: ConfigParams *mAppConfig;
  // not supported: ConfigParams mMcConfigLast;
  // not supported: bool mCheckNextMcConfig;

  int mTimeoutCount;
  int mTimeoutFwVer;
  // not supported: int mTimeoutMcconf;
  // not supported: int mTimeoutAppconf;
  int mTimeoutValues;
  int mTimeoutValuesSetup;
  int mTimeoutImuData;
  int mTimeoutDecPpm;
  int mTimeoutDecAdc;
  int mTimeoutDecChuk;
  int mTimeoutPingCan;

  ros::Publisher ackReceived_pub;
  ros::Publisher bmRebootRes_pub;
  ros::Publisher decodedPpmReceived_pub;
  ros::Publisher experimentSamplesReceived_pub;
  ros::Publisher gpdBufferNotifyReceived_pub;
  ros::Publisher motorRLReceived_pub;
  ros::Publisher printReceived_pub;
  ros::Publisher valuesImuReceived_pub;
  ros::Publisher bldcDetectReceived_pub;
  ros::Publisher decodedAdcReceived_pub;
  ros::Publisher detectAllFocReceived_pub;
  ros::Publisher focHallTableReceived_pub;
  ros::Publisher gpdBufferSizeLeftReceived_pub;
  ros::Publisher nrfPairingRes_pub;
  ros::Publisher rotorPosReceived_pub;
  ros::Publisher valuesReceived_pub;
  ros::Publisher bmConnRes_pub;
  ros::Publisher decodedChukReceived_pub;
  ros::Publisher encoderParamReceived_pub;
  ros::Publisher fwVersionReceived_pub;
  ros::Publisher motorLinkageReceived_pub;
  ros::Publisher pingCanRx_pub;
  ros::Publisher samplesReceived_pub;
  ros::Publisher valuesSetupReceived_pub;

  ros::Subscriber bmConnect_sub;
  ros::Subscriber getDecodedChuk_sub;
  ros::Subscriber getValuesSelective_sub;
  ros::Subscriber gpdOutputSample_sub;
  ros::Subscriber measureLinkage_sub;
  ros::Subscriber samplePrint_sub;
  ros::Subscriber setCurrentBrake_sub;
  ros::Subscriber setServoPos_sub;
  ros::Subscriber bmDisconnect_sub;
  ros::Subscriber getDecodedPpm_sub;
  ros::Subscriber getValuesSetup_sub;
  ros::Subscriber gpdSetBufferIntScale_sub;
  ros::Subscriber measureLinkageOpenloop_sub;
  ros::Subscriber sendAlive_sub;
  ros::Subscriber setDetect_sub;
  ros::Subscriber bmReboot_sub;
  ros::Subscriber getFwVersion_sub;
  ros::Subscriber getValuesSetupSelective_sub;
  ros::Subscriber gpdSetFsw_sub;
  ros::Subscriber measureRL_sub;
  ros::Subscriber sendTerminalCmd_sub;
  ros::Subscriber setDutyCycle_sub;
  ros::Subscriber detectAllFoc_sub;
  ros::Subscriber getGpdBufferSizeLeft_sub;
  ros::Subscriber gpdFillBuffer_sub;
  ros::Subscriber gpdSetMode_sub;
  ros::Subscriber pairNrf_sub;
  ros::Subscriber sendTerminalCmdSync_sub;
  ros::Subscriber setHandbrake_sub;
  ros::Subscriber detectMotorParam_sub;
  ros::Subscriber getImuData_sub;
  ros::Subscriber gpdFillBufferInt8_sub;
  ros::Subscriber measureEncoder_sub;
  ros::Subscriber pingCan_sub;
  ros::Subscriber setChukData_sub;
  ros::Subscriber setPos_sub;
  ros::Subscriber getDecodedAdc_sub;
  ros::Subscriber getValues_sub;
  ros::Subscriber gpdFillBufferInt16_sub;
  ros::Subscriber measureHallFoc_sub;
  ros::Subscriber reboot_sub;
  ros::Subscriber setCurrent_sub;
  ros::Subscriber setRpm_sub;
};

#endif  // VESC_V6_COMMANDS_H
