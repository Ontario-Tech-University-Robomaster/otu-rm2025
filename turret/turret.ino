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

int16_t tilt_zero;

bool dataValid = false;

struct motor_data {
  int m1, m2, m3, m4;
};

CAN_message_t setTurret(struct motor_data motorValues) {
  CAN_message_t drivetrain = {
    .id = 0x200,  // can identifier M3508 & C620
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

CAN_message_t setGimbal(struct motor_data motorValues) {
  CAN_message_t gimbal {
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
  return gimbal;
}

void setup() {

  SerialInput.begin(100000, SERIAL_8E1);  //100Kbps
  
  Can1.setBaudRate(1000000);  //1M
  Can1.begin(false);          // automatic retransmission

  // pinMode(PE11, OUTPUT);  //LED R
  // pinMode(PF14, OUTPUT);  //LED G
  Serial.begin(115200);
  // digitalWrite(PE11, LOW);   // turn the LED on (HIGH is the voltage level)
  // digitalWrite(PF14, LOW);  // turn the LED on (HIGH is the voltage level)
}

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

const int TorqueCeling = 7000;//7000mA is soft celing 8000mA is hard celing (motor melting)
int agitator = 0;
int tilt_s = 0;
int flywheelF = 0;  //0x4000;
int flywheelR = 0;  //0xC000;
int last1 = 3;
int last2 = 3;
// int last3 = 3;
// int last4 = 3;

void loop() {
  std::vector<uint8_t> dr16_raw = readDR16();
  DR16 dr16 = parseDR16(dr16_raw.data());
  dr16 = drop_controller(dr16);
  
  // float current_angle = to_radians(tilt.read_angle() - tilt_zero, GM6020_MAX_ANGLE);  // adjust for correction and turn to radian

  const int lb = -5000, ub = 5000;  // lower and upper bounds

  int wheel = map(dr16.wheel, 384, 1684, lb, ub);  // - 1000; Yaw
  int rightY = map(dr16.c1, 384, 1684, lb, ub);  // Turret Tilt
  
  if (abs(wheel) <= 800) wheel = 0;
  if (abs(rightY) <= 1) rightY = 0;

  auto cm1 = tilt.read_speed();


  // if (cm1 >= TorqueCeling) agitator = 0;
    if (last1 != last2) {  //Agitator, on wheel
    if (dr16.s1 == 3) agitator = 0;
    else if (dr16.s1 == 2) agitator = -10000;
    else if (dr16.s1 == 1) agitator = 10000;
  }
  last2 = last1;
  last1 = dr16.s1;

int WHOATHEREBESSY = 0.1;//slow down the tilt motor so not crash

  int m1_s = 0xC000;  //L flywheel
  int m2_s = 0x4000;  //R Flywheel
  int m4_s = agitator;   //Agitator
  int tilt_s = rightY * WHOATHEREBESSY;   //Tilt motor

  // tilt_s = mtilt.update(tilt_s - cm1);

  Serial.print("Agitator Value: ");
  Serial.println(agitator);

  Serial.println("Flywheel speed in RPM: ");
  Serial.println(motor1.read_speed());

  Serial.println("Tilt Motor Angle: ");
  Serial.println(tilt.read_angle());//find 0 angle and TILT and PAN motor
  
  auto turret_fire = setTurret({ m1_s, m2_s, 0, m4_s });
  auto turret_tilt = setGimbal({ tilt_s, tilt_s, tilt_s, tilt_s });

  if (!Can1.write(turret_fire) || !Can1.write(turret_tilt)) {
    digitalWrite(PE11, HIGH);
    Serial.println("COULD NOT WRITE TURRET");
    Can1.end();
    Can1.begin(false);
  } else {
    Serial.println("TURRET IS WORKING");
  }

    
  // if (1) pp = !pp;
  // digitalWrite(PE11, pp);   // turn the LED on (HIGH is the voltage level)
  // digitalWrite(PF14, !pp);  // turn the LED on (HIGH is the voltage level)
  delay(10);
}