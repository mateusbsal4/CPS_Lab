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

#ifndef PACKET_H
#define PACKET_H

#include "vesc_v6_driver/bytearray.h"

#include "vesc_v6_driver/singleton.h"

#include <ros/timer.h>

// Singleton
class VescPacket : public Singleton<VescPacket> {
  friend class Singleton<VescPacket>;

 public:
  ~VescPacket();

  void sendPacket(ByteArray &data) const;
  static unsigned short crc16(const unsigned char *buf, unsigned int len);

 public:  // signals
          // void dataToSend(ByteArray &data);
          // void packetReceived(const std::vector<uint8_t> &packet);
 public:  // slots
  void processData(const std::vector<uint8_t> &data);

 private:  // slots
  void timerSlot();

 private:
  VescPacket();

  ros::Timer mTimer;
  int mRxTimer;
  int mByteTimeout;
  unsigned int mRxReadPtr;
  unsigned int mRxWritePtr;
  int mBytesLeft;
  unsigned int mMaxPacketLen;
  unsigned int mBufferLen;
  unsigned char *mRxBuffer;

  int try_decode_packet(unsigned char *buffer, unsigned int in_len,
                        int *bytes_left,
                        std::vector<ByteArray> &decodedPackets);
};

#endif  // PACKET_H
