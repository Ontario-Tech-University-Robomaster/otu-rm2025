#include "hal_conf_extra.h"
#include "pid.h"
#include <vector>
#include "motor_info.h"
#include "CanConf.h"
#include "dr16.h"
// #include <PID_v1_bc.h>

using namespace std;
motor motor1(M3508, 0);
motor motor2(M3508, 1);
motor motor3(M3508, 2);
motor motor4(M3508, 3);

// PID m1(39.36, 1226.72, 0);
// PID m1(3.936, 122.672, 0);
// PID m1(76, 6400, 0.02);
// PID m1(9, 50, 0.005);
PID m1(1, 0, 10);
PID m2(1, 0, 10);
PID m3(1, 0, 10);
PID m4(1, 0, 10);


CAN_message_t turret = {
  .id = 0x1FE,  // can identifier
  .len = 8,     // length of data
  .buf = { 0 }  // data
};

CAN_message_t motor_msg{
  .id = 0x204  // can identifier
};

CAN_message_t motor_feedback;

bool dataValid = false;

//oridigl is PD_0 and PD_1
STM32_CAN Can1(PD_0, PD_1);  //by PinName. Finds matching peripheral automatically

//                          RX   TX
HardwareSerial SerialInput(PB7, PB6);

vector<uint8_t> readDR16() {
  std::vector<uint8_t> rxData(18, 0);
  if (!SerialInput.available()) return rxData;
  // for (int i = 0; i < 9; ++i) SerialInput.read();  // deal with offset
  for (int index = 0; index < 18; index++) {
    rxData[index] = SerialInput.read();
  }

  return rxData;
}

struct motor_data {
  int m1, m2, m3, m4;
};


CAN_message_t setDrivetrain(struct motor_data motorValues) {
  motorValues.m1 = -motorValues.m1;
  CAN_message_t drivetrain = {
    .id = 0x200,  // can identifier
    .len = 8,     // length of data
    .buf = {
      motorValues.m1 >> 8,
      motorValues.m1,
      motorValues.m2 >> 8,
      motorValues.m2,
      motorValues.m3 >> 8,
      motorValues.m3,
      motorValues.m4 >> 8,
      motorValues.m4,
    }  // data
  };
  return drivetrain;
}

CAN_message_t setTurret(struct motor_data motorValues) {
  CAN_message_t drivetrain = {
    .id = 0x1FE,  // can identifier
    .len = 8,     // length of data
    .buf = {
      motorValues.m1 >> 8,
      motorValues.m1,
      motorValues.m2 >> 8,
      motorValues.m2,
      motorValues.m3 >> 8,
      motorValues.m3,
      motorValues.m4 >> 8,
      motorValues.m4,
    }  // data
  };
  return drivetrain;
}

double input = 0;
double output = 0;
double setpoint = 1000.0;  // Define setpoint
// PID myPID(&input, &output, &setpoint, 1, 5, 70, 1);
// PID myPID(&input, &output, &setpoint, 0, 0, 0, 0);

void setup() {
  // myPID.SetMode(P_ON_M);
  // myPID.SetOutputLimits(-1000, 1000);

  SerialInput.begin(100000, SERIAL_8E1);  //100Kbps

  Can1.setBaudRate(1000000);  //1M
  Can1.begin(false);

  pinMode(PE11, OUTPUT);  //LED R
  pinMode(PF14, OUTPUT);  //LED G
  Serial.begin(115200);
}

uint16_t prev_sp1 = 0;
uint16_t prev_sp2 = 0;
uint16_t prev_sp3 = 0;
uint16_t prev_sp4 = 0;
bool pp = true;
// struct motor_data prevmotorValues{0,0,0,0};
//const int offset = -1024;//ties the mapping so the data recieved means 0
const double skillIssue = 0.01;
const int mapLimit = 10;

// less than or greater than
inline bool ltgt(int lower, int val, int upper) {
  return (lower > val) || (val > upper);
}

uint16_t c0_prev = 1024;
uint16_t c1_prev = 1024;
uint16_t c2_prev = 1024;
uint16_t c3_prev = 1024;
DR16 drop_controller(DR16 in) {
  if (ltgt(364, in.c0, 1684)
      || ltgt(364, in.c1, 1684)
      || ltgt(364, in.c2, 1684)
      || ltgt(364, in.c3, 1684)) {
    Serial.println("killed");

    in.c0 = c0_prev;
    in.c1 = c1_prev;
    in.c2 = c2_prev;
    in.c3 = c3_prev;
    return in;
  }
  c0_prev = in.c0;
  c1_prev = in.c1;
  c2_prev = in.c2;
  c3_prev = in.c3;
  return in;
}

bool body_pan = false;

void loop() {
  std::vector<uint8_t> dr16_raw = readDR16();
  DR16 dr16 = parseDR16(dr16_raw.data());
  dr16 = drop_controller(dr16);


  const int lb = -5000, ub = 5000;

  int rightX = map(dr16.c0, 384, 1684, lb, ub);  // - 1000; Yaw
  int rightY = map(dr16.c1, 384, 1684, lb, ub);  // Not currently used for driving
  int leftX = map(dr16.c2, 384, 1684, lb, ub);   // + testX;// - 1000;
  int leftY = map(dr16.c3, 384, 1684, lb, ub);   // + testY;  // + 1000;


  if (abs(leftY) == 1) leftY = 0;
  if (abs(leftX) == 1) leftX = 0;
  if (abs(rightY) == 1) rightY = 0;
  if (abs(rightX) == 85) rightX = 0;

  auto cm1 = motor1.read_speed();
  auto cm2 = motor2.read_speed();
  auto cm3 = motor3.read_speed();
  auto cm4 = motor4.read_speed();

  struct motor_data drivetrainValues;
  struct motor_data turretValues;

  setpoint = leftY;

  int m1_s = leftY + leftX + (rightX && body_pan);
  int m2_s = leftY - leftX + (rightX && body_pan);
  int m3_s = -leftY - leftX + (rightX && body_pan);
  int m4_s = -leftY + leftX + (rightX && body_pan);

  int t_spin = rightX && !body_pan;


  if (dr16.s1 == 0) body_pan = true;
  else body_pan = false;

  int err1 = m1_s - cm1;
  int err2 = m2_s - cm2;
  int err3 = m3_s - cm3;
  int err4 = m4_s - cm4;

  int16_t sp1 = m1.update(err1);
  int16_t sp2 = m2.update(err2);
  int16_t sp3 = m3.update(err3);
  int16_t sp4 = m4.update(err4);


  drivetrainValues.m1 = sp1;
  drivetrainValues.m2 = sp2;
  drivetrainValues.m3 = sp3;
  drivetrainValues.m4 = sp4;


  Serial.print("time:");
  Serial.print(millis());
  Serial.print(",setpoint:");
  Serial.print(setpoint);
  Serial.print(",output:");
  Serial.print(sp1);
  Serial.print(",c3:");
  Serial.print(dr16.c3);
  Serial.print(",speed:");
  Serial.println(motor1.read_speed());


  auto dt = setDrivetrain(drivetrainValues);
  auto dt = setTurret({t_spin, t_spin, t_spin, t_spin});

  Can1.write(dt);


  if (1) pp = !pp;
  digitalWrite(PE11, pp);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PF14, !pp);  // turn the LED on (HIGH is the voltage level)
  delay(10);
}