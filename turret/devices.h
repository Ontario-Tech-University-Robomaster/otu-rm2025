#include "motor_info.h"
#ifndef DEVICES_H
#define DEVICES_H

motor motor1(M3508, 0);
motor motor2(M3508, 1);
motor motor3(M3508, 2);
motor motor4(M3508, 3);

<<<<<<< Updated upstream
motor tilt(GM6020, 1);

PID mtilt(1, 0, 10);//just for the 6020 tilt motor / controlled by Velocity
// PID m2(1, 0, 10);
// PID m3(1, 0, 10);
// PID m4(1, 0, 10);
=======
motor pan(GM6020, 1);

PID m1(1, 0, 10);
PID m2(1, 0, 10);
PID m3(1, 0, 10);
PID m4(1, 0, 10);
>>>>>>> Stashed changes

//oridigl is PD_0 and PD_1
STM32_CAN Can1(PD_0, PD_1);  //by PinName. Finds matching peripheral automatically

//                          RX   TX
<<<<<<< Updated upstream
HardwareSerial SerialInput(PG9, PG14);

// //Rx and Tx pins from Turret to Chassis
// HardwareSerial ChassisInput(PG9, PG14);//UART port

=======
HardwareSerial SerialInput(PB7, PB6);

<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
// HardwareSerial ChassisToTurret(PG9, PG14);//UART port from chassis to turret

>>>>>>> Stashed changes
>>>>>>> Stashed changes
#endif // DEVICES_H