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

#include "vesc_v6_driver/serial.h"

#include "vesc_v6_driver/packet.h"

VescSerial::~VescSerial() { Disconnect(true); }

void VescSerial::Setup(ros::NodeHandle nh) {}

bool VescSerial::Connect(const std::string& port) {
  if (port.empty()) return false;

  if (serial_.isOpen()) {
    ROS_ERROR(
        "Tried to open serial port twice! Disconnect first. Attempt ignored.");
    return true;
  }

  try {
    serial_.setPort(port);
    serial_.open();
  } catch (const std::exception& e) {
    std::stringstream ss;
    ss << "Failed to open the serial port to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  rx_thread_run_ = true;
  int result =
      pthread_create(&rx_thread_, nullptr, &VescSerial::rxThreadHelper, this);
  assert(0 == result);

  ROS_DEBUG("Serial connection to VESC established.");
  return true;
}

void VescSerial::Disconnect(const bool fast) {
  if (serial_.isOpen()) {
    // bring down read thread
    rx_thread_run_ = false;
    if (fast) {  // fast disconnect
      pthread_cancel(rx_thread_);
    }
    // wait for thread to stop
    int result = pthread_join(rx_thread_, nullptr);
    assert(0 == result);
    // close serial port
    serial_.close();
  }
  ROS_DEBUG("Serial connection to VESC closed.");
}

void VescSerial::Send(const ByteArray& data) {
  std::vector<uint8_t> data_vector(
      data.begin(),
      data.end());  // serial write only accepts vectors not deques

  size_t written = serial_.write(data_vector);
  if (written != data_vector.size()) {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << data_vector.size()
       << ".";
    throw SerialException(ss.str().c_str());
  }

  ROS_DEBUG("Send %lu bytes over Serial connection to VESC.",
            data_vector.size());
}

void* VescSerial::rxThread(void) {
  std::vector<uint8_t> buffer;
  buffer.reserve(4096);

  while (rx_thread_run_) {
    serial_.setTimeout(serial::Timeout::max(), 5000, 0, 5000,
                       0);  // timeout is 5 sec
    // wait until we receive some bytes
    while (serial_.waitReadable() == false) {
    }
    // attempt to read at least bytes_needed bytes from the serial port
    size_t bytes_read = serial_.read(buffer, serial_.available());
    if (bytes_read > 0) {
      VescPacket::instance()->processData(buffer);
      buffer.clear();
    }
  }
}
