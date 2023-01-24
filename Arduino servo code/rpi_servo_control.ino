#include <Servo.h>

String data;

int servop1 = 3;
int servop2 = 5;
int servop3 = 6;
int servop4 = 9;
int servop7 = 10;
int servop8 = 11;

Servo servo1, servo2, servo3, servo4, servo7, servo8;

void setup() {
  Serial.begin(9600);
  servo1.attach(servop1);
  servo2.attach(servop2);
  servo3.attach(servop3);
  servo4.attach(servop4);
  servo7.attach(servop7);
  servo8.attach(servop8);
}

void loop() {
  if (Serial.available()) {
    char received = Serial.read();
    data += received;
    if (received == '\n') {
      String servoId = getValue(data, ':', 0);
      String servoAngle = getValue(data, ':', 1);
      if (servoAngle.toInt() >= 0 && servoAngle.toInt() <= 180) {
        switch (servoId.toInt()) {
          case 1:
            servo1.write(servoAngle.toInt());
            break;
          case 2:
            servo2.write(servoAngle.toInt());
            break;
          case 3:
            servo3.write(servoAngle.toInt());
            break;
          case 4:
            servo4.write(servoAngle.toInt());
            break;
          case 7:
            servo7.write(servoAngle.toInt());
            break;
          case 8:
            servo8.write(servoAngle.toInt());
            break;
        }
      }
      data = "";
    }
  }
  delay(50);
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
