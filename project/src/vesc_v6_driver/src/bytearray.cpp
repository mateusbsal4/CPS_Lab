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

#include "vesc_v6_driver/bytearray.h"

#include "vesc_v6_driver/datatypes.h"

#include <cmath>

#include <stdint.h>

namespace {
inline double roundDouble(double x) {
  return x < 0.0 ? ceil(x - 0.5) : floor(x + 0.5);
}
}  // namespace

ByteArray::ByteArray() : std::deque<uint8_t>() {}

ByteArray::ByteArray(const std::vector<unsigned char> &vector)
    : std::deque<uint8_t>() {
  insert(end(), vector.begin(), vector.end());
}

void ByteArray::AppendToVector(std::vector<unsigned char> &vector) const {
  vector.insert(vector.end(), begin(), end());
}

void ByteArray::AppendInt32(int32_t number) {
  push_back(static_cast<uint8_t>((number >> 24) & 0xFF));
  push_back(static_cast<uint8_t>((number >> 16) & 0xFF));
  push_back(static_cast<uint8_t>((number >> 8) & 0xFF));
  push_back(static_cast<uint8_t>(number & 0xFF));
}

void ByteArray::AppendUint32(uint32_t number) {
  push_back(static_cast<uint8_t>((number >> 24) & 0xFF));
  push_back(static_cast<uint8_t>((number >> 16) & 0xFF));
  push_back(static_cast<uint8_t>((number >> 8) & 0xFF));
  push_back(static_cast<uint8_t>(number & 0xFF));
}

void ByteArray::AppendInt16(int16_t number) {
  push_back(static_cast<uint8_t>((number >> 8) & 0xFF));
  push_back(static_cast<uint8_t>(number & 0xFF));
}

void ByteArray::AppendUint16(uint16_t number) {
  push_back(static_cast<uint8_t>((number >> 8) & 0xFF));
  push_back(static_cast<uint8_t>(number & 0xFF));
}

void ByteArray::AppendInt8(int8_t number) {
  push_back(static_cast<uint8_t>(number));
}

void ByteArray::AppendUint8(uint8_t number) { push_back(number); }

void ByteArray::AppendDouble32(double number, double scale) {
  AppendInt32(static_cast<int32_t>(roundDouble(number * scale)));
}

void ByteArray::AppendDouble16(double number, double scale) {
  AppendInt16(static_cast<int16_t>(roundDouble(number * scale)));
}

void ByteArray::AppendDouble32Auto(double number) {
  int e = 0;
  float fr = frexpf(number, &e);
  float fr_abs = fabsf(fr);
  uint32_t fr_s = 0;

  if (fr_abs >= 0.5) {
    fr_s = (uint32_t)((fr_abs - 0.5f) * 2.0f * 8388608.0f);
    e += 126;
  }

  uint32_t res = ((e & 0xFF) << 23) | (fr_s & 0x7FFFFF);
  if (fr < 0) {
    res |= 1 << 31;
  }

  AppendUint32(res);
}

void ByteArray::AppendString(std::string str) {
  for (auto &it : str) {
    push_back(static_cast<uint8_t>(it));
  }
  push_back(0);
}

int32_t ByteArray::PopFrontInt32() {
  if (size() < 4) {
    return 0;
  }

  uint32_t value =
      static_cast<uint32_t>(pop()) << 24 | static_cast<uint32_t>(pop()) << 16 |
      static_cast<uint32_t>(pop()) << 8 | static_cast<uint32_t>(pop());
  int32_t res = *reinterpret_cast<int32_t *>(&value);

  return res;
}

uint32_t ByteArray::PopFrontUint32() {
  if (size() < 4) {
    return 0;
  }

  uint32_t res =
      static_cast<uint32_t>(pop()) << 24 | static_cast<uint32_t>(pop()) << 16 |
      static_cast<uint32_t>(pop()) << 8 | static_cast<uint32_t>(pop());

  return res;
}

int16_t ByteArray::PopFrontInt16() {
  if (size() < 2) {
    return 0;
  }

  uint16_t value =
      static_cast<uint16_t>(pop()) << 8 | static_cast<uint16_t>(pop());
  int16_t res = *reinterpret_cast<int16_t *>(&value);

  return res;
}

uint16_t ByteArray::PopFrontUint16() {
  if (size() < 2) {
    return 0;
  }

  uint16_t res =
      static_cast<uint16_t>(pop()) << 8 | static_cast<uint16_t>(pop());

  return res;
}

int8_t ByteArray::PopFrontInt8() {
  if (size() < 1) {
    return 0;
  }

  uint8_t value = pop();
  int8_t res = *reinterpret_cast<int8_t *>(&value);

  return res;
}

uint8_t ByteArray::PopFrontUint8() {
  if (size() < 1) {
    return 0;
  }

  uint8_t res = pop();

  return res;
}

double ByteArray::PopFrontDouble32(double scale) {
  return static_cast<double>(PopFrontInt32() / scale);
}

double ByteArray::PopFrontDouble16(double scale) {
  return static_cast<double>(PopFrontInt16() / scale);
}

double ByteArray::PopFrontDouble32Auto() {
  uint32_t res = PopFrontUint32();

  int e = (res >> 23) & 0xFF;
  int fr = res & 0x7FFFFF;
  bool negative = res & (1 << 31);

  float f = 0.0;
  if (e != 0 || fr != 0) {
    f = (float)fr / (8388608.0 * 2.0) + 0.5;
    e -= 126;
  }

  if (negative) {
    f = -f;
  }

  return ldexpf(f, e);
}

std::string ByteArray::PopFrontString() {
  if (size() < 1) {
    return std::string();
  }

  std::string result;
  while (empty() == false) {
    uint8_t value = pop();
    const char c = *reinterpret_cast<char *>(&value);
    if (c == '\0') {
      break;
    }
    result.push_back(c);
  }
  return result;
}

void ByteArray::PopFrontNBytes(ByteArray &outputBytes, unsigned int charCount) {
  for (unsigned int i = 0; i < charCount && empty() == false; ++i) {
    outputBytes.push_back(pop());
  }
}

uint8_t ByteArray::pop() {
  if (size() == 0) return 0xFF;

  uint8_t res = front();
  pop_front();

  return res;
}
