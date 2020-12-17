#include <Servo.h>
 
Servo servo;
int servoPinNum = 8;
int delayTime = 50;
float deltaTime = delayTime / 1000.0;
float initialDeg = 90.0;
float minDeg = 45.0;
float maxDeg = 135.0;

void setup() {
  Serial.begin(9600);

  servo.attach(servoPinNum);
  servo.write((int)initialDeg);
}

int serialRead() {
  char c[6];
  for (int i = 0; i < 6; i++) {
    c[i] = Serial.read();
    Serial.println(c[i]);
    if (c[i] == '\0') {
      break;
    }
  }
  return atoi(c);
}

void loop() {
  static float deg = initialDeg;
  static int angVel = 0;

  if (Serial.available() > 0) {
    delay(10);
    angVel = serialRead();
    if (angVel < -180) {
      angVel = -180;
    } else if (angVel > 180) {
      angVel = 180;
    }
  }

  deg += (float)angVel * deltaTime;
  if (deg < minDeg) {
    deg = minDeg;
  } else if (deg > maxDeg) {
    deg = maxDeg;
  }
  servo.write((int)deg);

  delay(delayTime);
}
