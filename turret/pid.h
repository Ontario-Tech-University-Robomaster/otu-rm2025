class PID {
  const double kp;
  const double kd;
  const double ki;
  double toterr = 0;
  double lasterr = 0;
  double lasttime = 0;
  int iters;

public:
  PID(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd) {}

  double update(double error) {
    if (iters++ > 10) {
      iters = 0;
      toterr = 0;
    }

    long P = kp * error;
    long currenttime = millis();
    long delta = currenttime - lasttime;  // this is our change in time
    toterr += error * delta;              // this is for our integral
    lasttime = currenttime;
    long I = ki * toterr;

    // derivative
    long D = kd * (error - lasterr) / delta;  // this is delta error / delta time
    long PID_ret = P + I + D;
    lasterr = error;
    return PID_ret;
  }
};