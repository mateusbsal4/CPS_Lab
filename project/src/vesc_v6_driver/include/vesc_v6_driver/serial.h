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

#ifndef SERIAL_SINGLETON_H
#define SERIAL_SINGLETON_H

#include <ros/ros.h>
#include <serial/serial.h>

#include "vesc_v6_driver/bytearray.h"

#include "vesc_v6_driver/singleton.h"

class VescSerial : public Singleton<VescSerial> {
  friend class Singleton<VescSerial>;

 public:
  void Setup(ros::NodeHandle nh);
  bool Connect(const std::string& port);
  void Disconnect(const bool fast = false);
  void Send(const ByteArray& data);

 private:
  VescSerial() = default;
  ~VescSerial();

  serial::Serial serial_;

  void* rxThread(void);
  pthread_t rx_thread_;
  bool rx_thread_run_;

  static void* rxThreadHelper(void* context) {
    return static_cast<VescSerial*>(context)->rxThread();
  }
};

class SerialException : public std::exception {
  // Disable copy constructors
  SerialException& operator=(const SerialException&);
  std::string e_what_;

 public:
  SerialException(const char* description) {
    std::stringstream ss;
    ss << "SerialException " << description << " failed.";
    e_what_ = ss.str();
  }
  SerialException(const SerialException& other) : e_what_(other.e_what_) {}
  virtual ~SerialException() throw() {}
  virtual const char* what() const throw() { return e_what_.c_str(); }
};

#endif  // SERIAL_SINGLETON_H
