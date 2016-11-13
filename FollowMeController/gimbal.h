
Servo yawServo;
Servo pitchServo;

int16_t cameraYaw = 0;

void setPitch(float degrees) {
  degrees += -5; // trim
  degrees = constrain(degrees, -90, 60);
  int servoValue = map(degrees, -90, 90, 135, 45);
  pitchServo.write(servoValue);
}

void setYaw(float degrees) {
  degrees *= 0.9;
  degrees += -9; // trim
  degrees = constrain(degrees, -170, 170);
  int servoValue = map(degrees, -180, 180, 135, 45);
  yawServo.write(servoValue);
}

void gimbal_setup() {
  yawServo.attach(5);
  pitchServo.attach(3);
  setPitch(0);
  setYaw(0);
}

void gimbal_loop() {
}
