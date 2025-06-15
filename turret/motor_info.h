//
// Created by christian on 4/13/25.
//

#ifndef MOTOR_INFO_H
#define MOTOR_INFO_H

#include "typedefs.h"
#include "CanConf.h"

#define UPDATE_RATE 1 // ms


enum MOTOR_GROUP {
  M3508_C0 = 0x200,
  M3508_C1 = 0x1FF,
  GM6020_C0 = 0x1FE,
  GM6020_C1 = 0x2FE,
  GM6020_V0 = 0x1FF,
  GM6020_V1 = 0x2FF,
};

enum MOTOR_TYPE {
  M3508 = 0,
  C620 = 2,
  C610 = 3,
  GM6020 = 1,
};

struct motor_info_t {
  int16_t angle = -1;
  int16_t speed = -1; // rpm
  int16_t torque = -1;
  uint8_t temp = -1;
};


/**
 * Holds all the data for a motor, and allows you to read from it whenever in an
 * "asynchronous" fashion (not waiting for new data)
 * not thread safe
 */
class motor {
  int id_;
  MOTOR_TYPE type_;
  motor_info_t data_{};

  static void _update_vars();

  void _update();

public:
  /**
   * Holds all motor data
   * @param type @see MOTOR_TYPE
   * @param ID int between 1-7
   */
  motor(MOTOR_TYPE type, int ID);

  /**
   * Reads the angle of the motor and returns it as an int
   * @see [GM6020 manual](https://rm-static.djicdn.com/tem/17348/RM%20GM6020%20%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E8%8B%B1%EF%BC%8920231103.pdf)
   * @return Current angle of motor
   */
  int16_t read_angle();

  int16_t read_speed();

  int16_t read_torque();

  uint8_t read_temp();

  void print();
};

/**
 * Writes values to a set of motors.
 *
 * @warning M3508_C0 and GM6020_V0 go to the same ID group of 0x1FF
 *
 * @note M3508_C0: M3508,  current, bank 0
 * @note M3508_C1: M3508,  current, bank 0
 * @note GM6020_C0: GM6020, current, bank 0
 * @note GM6020_C1: GM6020, current, bank 1
 * @note GM6020_V0: GM6020, voltage, bank 0
 * @note GM6020_V1: GM6020, voltage, bank 1
 *
 * @param group The motor group/bank to write to
 * @param m0 motor 1 / motor 5 if bank 2
 * @param m1 motor 2 / motor 6 if bank 2
 * @param m2 motor 3 / motor 7 if bank 2
 * @param m3 motor 4 / ignored if bank 2
 */
inline void write_motors(const MOTOR_GROUP group, const uint16_t m0, const uint16_t m1, const uint16_t m2,
                         const uint16_t m3) {
  // todo: do better conversion than shifting

  // ReSharper disable CppFunctionalStyleCast
  CAN_message_t msg{
    .id = uint32_t(group),
    .len = 8,
    .buf = {
      uint8_t(m0 & 0xFF),
      uint8_t(m0 >> 8),
      uint8_t(m1 & 0xFF),
      uint8_t(m1 >> 8),
      uint8_t(m2 & 0xFF),
      uint8_t(m2 >> 8),
      uint8_t(m3 & 0xFF),
      uint8_t(m3 >> 8),
    }
  };

  // Can1.write(msg);
}

#endif //MOTOR_INFO_H
