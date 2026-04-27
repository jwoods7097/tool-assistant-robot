#include <Arduino.h>
#include <Servo.h>

#define NUM_SERVOS 5

Servo servos[NUM_SERVOS];

// CHANGE THESE to match your wiring
int servoPins[NUM_SERVOS] = {3, 4, 5, 6, 7};

// Current + target angles
int currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90};
int targetAngles[NUM_SERVOS];

// Serial input
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);

  // Attach servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(currentAngles[i]);
  }

  inputString.reserve(50);
}

// =========================
// SERIAL EVENT
// =========================

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }
}

// =========================
// PARSE INPUT
// =========================

void parseInput(String data) {
  int index = 0;
  int lastIndex = 0;

  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',') {
      targetAngles[index++] = data.substring(lastIndex, i).toInt();
      lastIndex = i + 1;

      if (index >= NUM_SERVOS) break;
    }
  }

  // Last value
  if (index < NUM_SERVOS) {
    targetAngles[index++] = data.substring(lastIndex).toInt();
  }

  // Clamp values
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetAngles[i] = constrain(targetAngles[i], 0, 180);
  }

  // Debug print
  Serial.print("Target: ");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(targetAngles[i]);
    if (i < NUM_SERVOS - 1) Serial.print(", ");
  }
  Serial.println();
}

// =========================
// SMOOTH MOTION FUNCTION
// =========================

void moveSmooth(int target[]) {
  int steps = 20;        // number of interpolation steps
  int delayMs = 20;      // delay between steps

  for (int step = 1; step <= steps; step++) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      int start = currentAngles[i];
      int end = target[i];

      int next = start + (end - start) * step / steps;
      servos[i].write(next);
    }
    delay(delayMs);
  }

  // Update current angles
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = target[i];
  }
}

void loop() {
  if (stringComplete) {
    parseInput(inputString);
    moveSmooth(targetAngles);

    inputString = "";
    stringComplete = false;
  }
}