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

#include <ros/debug.h>
#include <ros/ros.h>

#include "vesc_v6_driver/commands.h"
#include "vesc_v6_driver/serial.h"


/// TODO
/// * Add header incl. timestamp to each message when received from vesc.

int main(int argc, char** argv) {
  ros::init(argc, argv, "vesc_v6_driver");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

#ifdef DEBUG
  // starts the node with logger level debug
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();
#endif

  // get vesc serial port address
  std::string port_param;
  if (private_nh.getParam("port", port_param) == false) {
    ROS_FATAL("VESC communication port parameter required (name=\"port\").");
    ros::shutdown();
    return -1;
  }

  VescCommands::instance()->Setup(nh);
  VescSerial::instance()->Setup(nh);
  VescSerial::instance()->Connect(port_param);

  ros::spin();

  return 0;
}
