//
// Created by christian on 4/13/25.
//


#include "motor_info.h"

#include <iostream>
#include <sstream>

static long last_update = -100;
static motor_info_t all_motors[11];


void motor::_update_vars() {
  last_update = millis();
  CAN_message_t msg;
  while (Can1.read(msg))
    // ReSharper disable once CppCStyleCast
    // all_motors[msg.id - 0x201] = (motor_info_t &) (msg.buf[0]);
    all_motors[msg.id - 0x201] = {
        .angle = msg.buf[0] << 8 | msg.buf[1],
        .speed = msg.buf[2] << 8 | msg.buf[3],
        .torque = msg.buf[4] << 8 | msg.buf[5],
        .temp = msg.buf[6],
    };
}

void motor::_update() {
  if (millis() - last_update > UPDATE_RATE) {
    _update_vars();
  }

  switch (type_) {
    case C620:
    case C610:
    case M3508:
      data_ = all_motors[id_];
      break;
    case GM6020:
      data_ = all_motors[id_ + 4];
      break;
  }
}

motor::motor(const MOTOR_TYPE type, const int ID) {
  id_ = ID;
  type_ = type;
  data_ = all_motors[id_];
}

int16_t motor::read_angle() {
  _update();
  return data_.angle;
}

int16_t motor::read_speed() {
  _update();
  return data_.speed;
}

int16_t motor::read_torque() {
  _update();
  return data_.torque;
}

uint8_t motor::read_temp() {
  _update();
  return data_.temp;
}


void motor::print() {
  std::stringstream ss;
  switch (type_) {
    case C620:
    case C610:
    case M3508:
      ss << "M3508: ";
      break;

    case GM6020:
      ss << "GM6020: ";
      break;
  }
  ss << read_angle() << " : " << read_speed() << " : " << read_torque() << " : " << (int) read_temp();
  std::cout << ss.str() << std::endl;
}
