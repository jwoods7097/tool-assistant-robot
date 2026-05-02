#include <Servo.h>
#include <math.h>

#define NUM_SERVOS 5

#define GRIPPER  0
#define WRIST    1
#define ELBOW    2
#define SHOULDER 3
#define BASE     4

Servo servos[NUM_SERVOS];
int pins[NUM_SERVOS] = {7, 6, 5, 4, 3};

int angles[NUM_SERVOS] = {90, 90, 90, 90, 90};
String input = "";

// Arm lengths in mm
float L1 = 58.0;     // shoulder to elbow
float L2 = 110.0;    // elbow to gripper tip approx

// Table/gripper height tuning
float PICK_Z = -30.0;     // more negative = lower down
float ABOVE_Z = 10.0;     // safer above-object height

// Servo calibration
int BASE_CENTER = 90;
int BASE_SIGN = 1;

int SHOULDER_OFFSET = 90;
int SHOULDER_SIGN = 1;

int ELBOW_OFFSET = 90;
int ELBOW_SIGN = 1;

// Wrist and gripper
int WRIST_DOWN = 50;
int WRIST_UP = 90;

int GRIPPER_OPEN = 120;
int GRIPPER_CLOSE = 40;

// Drop
int BASE_DROP = 150;

// Limits
int BASE_MIN = 20;
int BASE_MAX = 160;
int SHOULDER_MIN = 25;
int SHOULDER_MAX = 165;
int ELBOW_MIN = 15;
int ELBOW_MAX = 170;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(pins[i]);
    servos[i].write(angles[i]);
    delay(200);
  }

  Serial.println("READY 3D IK");
  Serial.println("Use: PICK 0 120");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      input.trim();
      processCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void moveServo(int i, int target) {
  target = constrain(target, 0, 180);
  int current = angles[i];

  if (current < target) {
    for (int a = current; a <= target; a++) {
      servos[i].write(a);
      delay(15);
    }
  } else {
    for (int a = current; a >= target; a--) {
      servos[i].write(a);
      delay(15);
    }
  }

  angles[i] = target;
}

void homePosition() {
  moveServo(BASE, 90);
  moveServo(SHOULDER, 90);
  moveServo(ELBOW, 90);
  moveServo(WRIST, 90);
  moveServo(GRIPPER, GRIPPER_OPEN);
}

bool computeIK(float x, float y, float z, int &baseServo, int &shoulderServo, int &elbowServo) {
  // x = left/right, y = forward, z = height

  float baseRad = atan2(x, y);
  float baseDeg = baseRad * 180.0 / PI;

  baseServo = BASE_CENTER + BASE_SIGN * baseDeg;
  baseServo = constrain(baseServo, BASE_MIN, BASE_MAX);

  float d = sqrt(x * x + y * y);       // forward reach after base rotation
  float r = sqrt(d * d + z * z);       // shoulder-to-target distance

  if (r > L1 + L2 || r < abs(L1 - L2)) {
    Serial.println("IK FAILED: unreachable");
    return false;
  }

  float cosElbow = (r*r - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  cosElbow = constrain(cosElbow, -1.0, 1.0);

  float elbowRad = acos(cosElbow);

  float cosShoulder = (L1*L1 + r*r - L2*L2) / (2.0 * L1 * r);
  cosShoulder = constrain(cosShoulder, -1.0, 1.0);

  float shoulderRad = atan2(z, d) + acos(cosShoulder);

  float shoulderDeg = shoulderRad * 180.0 / PI;
  float elbowDeg = elbowRad * 180.0 / PI;

  shoulderServo = SHOULDER_OFFSET + SHOULDER_SIGN * shoulderDeg;
  elbowServo = ELBOW_OFFSET + ELBOW_SIGN * elbowDeg;

  shoulderServo = constrain(shoulderServo, SHOULDER_MIN, SHOULDER_MAX);
  elbowServo = constrain(elbowServo, ELBOW_MIN, ELBOW_MAX);

  Serial.print("IK base=");
  Serial.print(baseServo);
  Serial.print(" shoulder=");
  Serial.print(shoulderServo);
  Serial.print(" elbow=");
  Serial.println(elbowServo);

  return true;
}

void moveArmTo(int x, int y, float z, bool wristDown) {
  int baseAngle, shoulderAngle, elbowAngle;

  bool ok = computeIK(x, y, z, baseAngle, shoulderAngle, elbowAngle);

  if (!ok) return;

  moveServo(BASE, baseAngle);
  delay(300);

  moveServo(SHOULDER, shoulderAngle);
  moveServo(ELBOW, elbowAngle);

  if (wristDown) {
    moveServo(WRIST, WRIST_DOWN);
  } else {
    moveServo(WRIST, WRIST_UP);
  }

  delay(600);
}

void pickObject(int x, int y) {
  Serial.print("PICK x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.println(y);

  moveServo(GRIPPER, GRIPPER_OPEN);
  delay(300);

  // Move above tool first
  moveArmTo(x, y, ABOVE_Z, false);

  // Move down to tool
  moveArmTo(x, y, PICK_Z, true);

  // Grab
  moveServo(GRIPPER, GRIPPER_CLOSE);
  delay(700);

  // Lift
  moveArmTo(x, y, ABOVE_Z, false);

  // Drop
  moveServo(BASE, BASE_DROP);
  delay(500);

  moveServo(GRIPPER, GRIPPER_OPEN);
  delay(500);

  homePosition();

  Serial.println("DONE");
}

void processCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("PICK") || cmd.startsWith("MOVE")) {
    int x, y;

    int matched1 = sscanf(cmd.c_str(), "PICK %d %d", &x, &y);
    int matched2 = sscanf(cmd.c_str(), "MOVE %d %d", &x, &y);

    if (matched1 == 2 || matched2 == 2) {
      pickObject(x, y);
    } else {
      Serial.print("BAD COMMAND: ");
      Serial.println(cmd);
    }
  }

  else if (cmd == "HOME") {
    homePosition();
    Serial.println("HOME DONE");
  }

  else {
    Serial.print("UNKNOWN COMMAND: ");
    Serial.println(cmd);
  }
}