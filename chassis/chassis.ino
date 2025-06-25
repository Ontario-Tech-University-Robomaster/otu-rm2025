#include "hal_conf_extra.h"
#include "pid.h"
#include <vector>
#include "motor_info.h"
#include "CanConf.h"
#include "dr16.h"
#include "devices.h"

#define PI 3.141592
#define SPIN_CORRECTION -1  // cause I don't wanna actually calculate spin rate

using namespace std;

CAN_message_t motor_feedback;

int16_t turret_zero;

bool dataValid = false;

struct motor_data {
  int m1, m2, m3, m4;
};


CAN_message_t setDrivetrain(struct motor_data motorValues) {
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
    .id = 0x1FF,  // can identifier
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

void setup() {

  SerialInput.begin(100000, SERIAL_8E1);  //100Kbps

<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
  // ChassisToTurret.begin(100000, SERIAL_8E1);  //100Kbps

>>>>>>> Stashed changes
  Can1.setBaudRate(1000000);  //1M
  Can1.begin(false); // automatic retransmission

  pinMode(PE11, OUTPUT);  //LED R
  pinMode(PF14, OUTPUT);  //LED G
  Serial.begin(115200);
  digitalWrite(PE11, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PF14, LOW);  // turn the LED on (HIGH is the voltage level)
}

const double skillIssue = 0.30;
// const double skillIssue = 1;

// less than or greater than
inline bool ltgt(int lower, int val, int upper) {
  return (lower > val) || (val > upper);
}

DR16 drop_controller(DR16 in) {
  // compensate for dropouts to stop robot from jittering
  static uint16_t c0_prev = 1024;
  static uint16_t c1_prev = 1024;
  static uint16_t c2_prev = 1024;
  static uint16_t c3_prev = 1024;
  if (ltgt(364, in.c0, 1684)
      || ltgt(364, in.c1, 1684)
      || ltgt(364, in.c2, 1684)
      || ltgt(364, in.c3, 1684)) {
    // Serial.println("killed");

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

float to_radians(uint16_t num, uint16_t ub) {
  float frac = ((float)num) / ub;
  return frac * 2 * PI;
}

struct vector2 {
  float x, y;
};

// Turns global orthogonal to local orthogonal
struct vector2 rotate_by(struct vector2 in, float angle) {
  float c = cos(angle);
  float s = sin(angle);

  return {
    (in.x * c - in.y * s),
    (in.x * s + in.y * c)
  };
}

int beyblade = 0;
int last1 = 3;
int last2 = 3;

void loop() {
  std::vector<uint8_t> dr16_raw = readDR16();
  DR16 dr16 = parseDR16(dr16_raw.data());
  dr16 = drop_controller(dr16);

  float current_angle = to_radians(pan.read_angle() - turret_zero, GM6020_MAX_ANGLE);  // adjust for correction and turn to radian

  const int lb = -5000, ub = 5000;  // lower and upper bounds

  int rightX = map(dr16.c0, 384, 1684, lb, ub);  // - 1000; Yaw
  int rightY = map(dr16.c1, 384, 1684, lb, ub);  // Not currently used for driving
  int leftX = map(dr16.c2, 384, 1684, lb, ub);   // + testX;// - 1000;
  int leftY = map(dr16.c3, 384, 1684, lb, ub);   // + testY;  // + 1000;

  if (abs(leftY) <= 1) leftY = 0;
  if (abs(leftX) <= 1) leftX = 0;
  if (abs(rightY) <= 1) rightY = 0;
  if (abs(rightX) <= 85) rightX = 0;

  auto cm1 = motor1.read_speed();
  auto cm2 = motor2.read_speed();
  auto cm3 = motor3.read_speed();
  auto cm4 = motor4.read_speed();

  struct motor_data drivetrainValues;



  if (last1 != last2) {
    if (dr16.s2 == 3) beyblade = 0;
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    else if (dr16.s2 == 2) beyblade = 2000;
    else if (dr16.s2 == 1) beyblade = -2000;
=======
    else if (dr16.s2 == 2) beyblade = 10000;//max is 468, assume cause of weight is 400
    else if (dr16.s2 == 1) beyblade = -10000;
>>>>>>> Stashed changes
=======
    else if (dr16.s2 == 2) beyblade = 10000;//max is 468, assume cause of weight is 400
    else if (dr16.s2 == 1) beyblade = -10000;
>>>>>>> Stashed changes
=======
    else if (dr16.s2 == 2) beyblade = 10000;//max is 468, assume cause of weight is 400
    else if (dr16.s2 == 1) beyblade = -10000;
>>>>>>> Stashed changes
  }
  last2 = last1;
  last1 = dr16.s2;

  struct vector2 global_dir = { leftX, leftY };
  struct vector2 local_dir = rotate_by(global_dir, current_angle);

  int m1_s = skillIssue * (local_dir.y + local_dir.x) + beyblade;
  int m2_s = skillIssue * (local_dir.y - local_dir.x) + beyblade;
  int m3_s = skillIssue * (-local_dir.y - local_dir.x) + beyblade;
  int m4_s = skillIssue * (-local_dir.y + local_dir.x) + beyblade;
  int pan_s = beyblade * SPIN_CORRECTION - rightX;
  // Serial.println(pan.read_angle());

  drivetrainValues.m1 = m1.update(m1_s - cm1);
  drivetrainValues.m2 = m2.update(m2_s - cm2);
  drivetrainValues.m3 = m3.update(m3_s - cm3);
  drivetrainValues.m4 = m4.update(m4_s - cm4);

  // Serial.println(drivetrainValues.m1);
  auto chassis = setDrivetrain(drivetrainValues);
  auto turret_pan = setTurret({ pan_s, pan_s, pan_s, pan_s });

  if (!Can1.write(chassis) || !Can1.write(turret_pan)) {
    digitalWrite(PE11, HIGH);
    Serial.println("COULD NOT WRITE CHASSIS");
    Can1.end();
    Can1.begin(false);
  } else {
    Serial.println("CHASSIS");
  }


  // if (1) pp = !pp;
  // digitalWrite(PE11, pp);   // turn the LED on (HIGH is the voltage level)
  // digitalWrite(PF14, !pp);  // turn the LED on (HIGH is the voltage level)
  delay(10);
}