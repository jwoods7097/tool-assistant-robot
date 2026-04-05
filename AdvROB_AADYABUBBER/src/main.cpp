#include <Arduino.h>
#include <Servo.h>

// Only the 4 servos we care about now
Servo gripperServo;   // D7
Servo shoulderServo;  // D6
Servo elbowServo;     // D5
Servo baseServo;      // D2

int gripperAngle  = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int baseAngle     = 90;

bool objectFound = false;
bool busy = false;

int objX = 48;
int objY = 48;
int objS = 0;

void moveServoSmooth(Servo &s, int &current, int target, int stepDelay = 10) {
  if (current < target) {
    for (int a = current; a <= target; a++) {
      s.write(a);
      delay(stepDelay);
    }
  } else {
    for (int a = current; a >= target; a--) {
      s.write(a);
      delay(stepDelay);
    }
  }
  current = target;
}

void homePose() {
  moveServoSmooth(gripperServo,  gripperAngle,  90, 8);
  moveServoSmooth(shoulderServo, shoulderAngle, 90, 8);
  moveServoSmooth(elbowServo,    elbowAngle,    90, 8);
  moveServoSmooth(baseServo,     baseAngle,     90, 8);
  Serial.println("HOME");
}

void openGripper() {
  moveServoSmooth(gripperServo, gripperAngle, 70, 10);
  Serial.println("GRIP OPEN");
}

void closeGripper() {
  moveServoSmooth(gripperServo, gripperAngle, 25, 10);
  Serial.println("GRIP CLOSE");
}

void grabObject() {
  busy = true;
  Serial.println("GRAB START");

  openGripper();
  delay(300);

  // come down with shoulder + elbow only
  moveServoSmooth(shoulderServo, shoulderAngle, 120, 12);
  moveServoSmooth(elbowServo,    elbowAngle,     60, 12);
  delay(300);

  closeGripper();
  delay(300);

  // lift back
  moveServoSmooth(elbowServo,    elbowAngle,     90, 12);
  moveServoSmooth(shoulderServo, shoulderAngle,  90, 12);

  Serial.println("GRAB END");
  busy = false;
}

void parseLine(String line) {
  line.trim();

  if (line == "NONE") {
    objectFound = false;
    return;
  }

  int ix = line.indexOf("X:");
  int iy = line.indexOf(",Y:");
  int is = line.indexOf(",S:");

  if (ix == 0 && iy > 0 && is > iy) {
    objX = line.substring(2, iy).toInt();
    objY = line.substring(iy + 3, is).toInt();
    objS = line.substring(is + 3).toInt();
    objectFound = true;
  }
}

void setup() {
  Serial.begin(115200);

  gripperServo.attach(7);
  shoulderServo.attach(6);
  elbowServo.attach(5);
  baseServo.attach(3);

  delay(500);

  homePose();
  openGripper();

  Serial.println("ARM READY");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    parseLine(line);

    if (objectFound) {
      Serial.print("X=");
      Serial.print(objX);
      Serial.print(" Y=");
      Serial.print(objY);
      Serial.print(" S=");
      Serial.println(objS);
    } else {
      Serial.println("NONE");
    }
  }

  if (busy || !objectFound) return;

  // image center ~48 for 96x96 frame
  const int xCenter = 48;
  const int xTol = 8;

  // Stronger base movement
  if (objX < xCenter - xTol) {
    baseAngle -= 5;
    if (baseAngle > 170) baseAngle = 170;
    baseServo.write(baseAngle);
    Serial.print("BASE LEFT -> ");
    Serial.println(baseAngle);
    delay(120);
    return;
  }

  if (objX > xCenter + xTol) {
    baseAngle += 5;
    if (baseAngle < 10) baseAngle = 10;
    baseServo.write(baseAngle);
    Serial.print("BASE RIGHT -> ");
    Serial.println(baseAngle);
    delay(120);
    return;
  }

  // Object centered enough, then grab
  if (objY > 55 || objS > 700) {
    grabObject();
    delay(500);
  }
}