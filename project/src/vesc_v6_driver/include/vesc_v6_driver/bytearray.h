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

#ifndef BYTEARRAY_H
#define BYTEARRAY_H

#include <deque>
#include <memory>
#include <string>
#include <vector>

class ByteArray;

typedef std::shared_ptr<ByteArray> SharedByteArray;

class ByteArray : public std::deque<uint8_t> {
 public:
  ByteArray();
  ByteArray(const std::vector<unsigned char> &vector);

  void AppendToVector(std::vector<unsigned char> &vector) const;

  void AppendInt32(int32_t number);
  void AppendUint32(uint32_t number);
  void AppendInt16(int16_t number);
  void AppendUint16(uint16_t number);
  void AppendInt8(int8_t number);
  void AppendUint8(uint8_t number);
  void AppendDouble32(double number, double scale);
  void AppendDouble16(double number, double scale);
  void AppendDouble32Auto(double number);
  void AppendString(std::string str);

  int32_t PopFrontInt32();
  uint32_t PopFrontUint32();
  int16_t PopFrontInt16();
  uint16_t PopFrontUint16();
  int8_t PopFrontInt8();
  uint8_t PopFrontUint8();
  double PopFrontDouble32(double scale);
  double PopFrontDouble16(double scale);
  double PopFrontDouble32Auto();
  std::string PopFrontString();
  void PopFrontNBytes(ByteArray &outputBytes, unsigned int n);

 private:
  uint8_t pop();
};

#endif  // BYTEARRAY_H
