//
// Created by christian on 4/13/25.
//

#ifndef DR16_H
#define DR16_H

#include <vector>
#include "typedefs.h"
#include "devices.h"

struct __attribute__((packed)) DR16 {
  uint16_t c0 : 11;  // 11
  uint16_t c1 : 11;  // 11
  uint16_t c2 : 11;  // 11
  uint16_t c3 : 11;  // 11
  uint8_t s1 : 2;    // 2
  uint8_t s2 : 2;    // 2
  unsigned int mouse_x : 16;
  unsigned int mouse_y : 16;
  unsigned int mouse_z : 16;
  unsigned int lmb : 8;
  unsigned int rmb : 8;
  unsigned int keys : 16;
  unsigned int wheel : 16;
};

/**
 * Parses an array of DR16 data to a DR16 object
 * @param data Array of bytes containing controller data
 * @return DR16 object
 */
inline DR16 &parseDR16(uint8_t data[18]) {
  const union {
    DR16 *controller;
    uint8_t *raw;
  } c{ .raw = data };
  return *c.controller;
  return (reinterpret_cast<DR16 &>(data[0]));
}

std::vector<uint8_t> readDR16() {
  std::vector<uint8_t> rxData(18, 0);
  if (!SerialInput.available()) return rxData;  // swap to ChassisInput.availible?
  // for (int i = 0; i < 9; ++i) SerialInput.read();  // deal with offset
  for (int index = 0; index < 18; index++) {
    rxData[index] = SerialInput.read();
  }

  return rxData;
}

#endif  //DR16_H
