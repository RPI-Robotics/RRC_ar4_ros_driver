#include <Encoder.h>
#include <AccelStepper.h>

#define J1_PUL_PIN 0
#define J1_DIR_PIN 1
#define J2_PUL_PIN 2
#define J2_DIR_PIN 3
#define J3_PUL_PIN 4
#define J3_DIR_PIN 5

// --- ENCODER PINS ---
#define J1_ENC_BLU 14 // EB+
#define J1_ENC_BRN 15 // EA-

// Initialize the encoder
Encoder encPosJ1(J1_ENC_BLU, J1_ENC_BRN);

// =============================================================================
// CONFIGURATION & CLOSED-LOOP PREDICTION SETTINGS
// =============================================================================
bool enableClosedLoopGoal = true;

// Stop correcting when encoder is within this many ticks of the target goal
// (10 ticks = 2 motor steps of tolerance to completely prevent hunting)
const long goalTolerance = 10;
const long zeroTolerance = 10;

// Default kinematic limits for AccelStepper
float speedLimit = 8000.0; // steps per second
float accelLimit = 8000.0; // steps per second^2

// Define testing mode flags
#define testSerialJog true
#define testOneMotor false
#define testAll false

#define safety false
// =============================================================================

#define NUM_MOTORS 6
int pulPins[NUM_MOTORS] = {J1_PUL_PIN, J2_PUL_PIN, J3_PUL_PIN, -1, -1, -1};
int dirPins[NUM_MOTORS] = {J1_DIR_PIN, J2_DIR_PIN, J3_PUL_PIN, -1, -1, -1};
Encoder* encoders[NUM_MOTORS] = {&encPosJ1, nullptr, nullptr, nullptr, nullptr, nullptr};

// Motor steps * encMult = Encoder ticks (MK4/MK5 = 5.0 for J1-J4, J6)[cite: 4]
const float encMult[NUM_MOTORS] = {5.0, 5.0, 5.0, 5.0, 2.5, 5.0};
const int encDir[NUM_MOTORS] = {-1, 1, 1, 1, 1, 1};

long targetEncGoal[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
bool goalInitialized[NUM_MOTORS] = {false, false, false, false, false, false};

// Initialize AccelStepper instances[cite: 4]
AccelStepper steppers[NUM_MOTORS] = {
  AccelStepper(AccelStepper::DRIVER, J1_PUL_PIN, J1_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, J2_PUL_PIN, J2_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, J3_PUL_PIN, J3_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, -1, -1),
  AccelStepper(AccelStepper::DRIVER, -1, -1),
  AccelStepper(AccelStepper::DRIVER, -1, -1)
};

const int stepsPer90 = 6400; 
int jogSteps = 200;
int activeMotorIdx = 0;

void setup() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (pulPins[i] != -1) {
      steppers[i].setMaxSpeed(speedLimit);
      steppers[i].setAcceleration(accelLimit);
      steppers[i].setMinPulseWidth(10); // Standard for DM320T/DM542T[cite: 4]
    }
    if (encoders[i] != nullptr) {
      targetEncGoal[i] = encoders[i]->read();
      goalInitialized[i] = true;
    }
  }

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("=== Teensy 4.1 Motor & Encoder Serial Jog Ready ===");
  Serial.println("Commands:");
  Serial.println("  + or d : Jog HIGH (Right/Forward) by jogSteps");
  Serial.println("  - or a : Jog LOW  (Left/Reverse)  by jogSteps");
  Serial.println("  1      : Select Motor J1");
  Serial.println("  2      : Select Motor J2");
  Serial.println("  3      : Select Motor J3");
  Serial.println("  s[num] : Set jog step count (e.g., s800)");
  Serial.println("  v[num] : Set Max Speed steps/sec (e.g., v4000)");
  Serial.println("  q[num] : Set Acceleration steps/sec^2 (e.g., q8000)");
  Serial.println("  c      : Toggle closed-loop prediction & goal maintenance");
  Serial.println("  e      : Print current J1 Encoder Count & Goal");
  Serial.println("  r      : Reset J1 Encoder Count & Goal to 0");
  Serial.println("  z      : Smoothly return all active encoders to Zero");
}

long predictEncoderDelta(int motorIdx, int steps, uint8_t direction) {
  int dirSign = (direction == HIGH) ? 1 : -1;
  return lround(steps * encMult[motorIdx]) * dirSign * encDir[motorIdx];
}

// Executes a smooth open-loop move using AccelStepper
void stepMotorSmooth(int motorIdx, int steps, uint8_t direction) {
  if (pulPins[motorIdx] == -1) return;
  
  long stepTarget = (direction == HIGH) ? steps : -steps;
  steppers[motorIdx].move(stepTarget);

  while (steppers[motorIdx].distanceToGo() != 0) {
    steppers[motorIdx].run();
  }
}

// Evaluates error, calculates missing steps, and eases into the correction
void maintainEncoderGoals() {
  if (!enableClosedLoopGoal) return;

  bool movedThisCycle = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (encoders[i] != nullptr && pulPins[i] != -1 && goalInitialized[i]) {
      long currentPos = encoders[i]->read();
      long errorTicks = targetEncGoal[i] - currentPos;

      if (abs(errorTicks) > goalTolerance) {
        long stepsToCorrect = abs(errorTicks) / encMult[i];
        if (stepsToCorrect == 0) stepsToCorrect = 1;

        // Apply corrective sign based on encoder polarity
        if ((errorTicks * encDir[i]) < 0) {
          stepsToCorrect = -stepsToCorrect;
        }

        steppers[i].move(stepsToCorrect);
        movedThisCycle = true;
      }
    }
  }

  if (movedThisCycle) {
    bool stillRunning = true;
    while (stillRunning) {
      stillRunning = false;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (pulPins[i] != -1 && steppers[i].distanceToGo() != 0) {
          steppers[i].run();
          stillRunning = true;
        }
      }
    }
    // Allow mechanical settling before re-evaluating
    delay(50);
  }
}

void moveMotorWithPrediction(int motorIdx, int steps, uint8_t direction) {
  Encoder* enc = encoders[motorIdx];

  if (enc != nullptr) {
    long startEnc = enc->read();
    if (!goalInitialized[motorIdx]) {
      targetEncGoal[motorIdx] = startEnc;
      goalInitialized[motorIdx] = true;
    }

    long predictedDelta = predictEncoderDelta(motorIdx, steps, direction);
    
    if (enableClosedLoopGoal) {
      targetEncGoal[motorIdx] += predictedDelta;
    } else {
      targetEncGoal[motorIdx] = startEnc + predictedDelta;
    }

    Serial.print("-> Start Enc: ");
    Serial.print(startEnc);
    Serial.print(" | Target Goal: ");
    Serial.println(targetEncGoal[motorIdx]);

    stepMotorSmooth(motorIdx, steps, direction);

    long postOpenLoopEnc = enc->read();
    Serial.print("-> Post-Move Enc (Open-Loop): ");
    Serial.print(postOpenLoopEnc);
    Serial.print(" (Error: ");
    Serial.print(targetEncGoal[motorIdx] - postOpenLoopEnc);
    Serial.println(" ticks)");

    if (enableClosedLoopGoal) {
      int attempts = 0;
      while (abs(targetEncGoal[motorIdx] - enc->read()) > goalTolerance && attempts < 5) {
        maintainEncoderGoals();
        attempts++;
      }
      Serial.print("-> Maintained Enc (Closed-Loop): ");
      Serial.println(enc->read());
    }
  } else {
    stepMotorSmooth(motorIdx, steps, direction);
  }
}

void returnToZero() {
  Serial.println("Smoothly returning active encoders to zero...");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (encoders[i] != nullptr) {
      targetEncGoal[i] = 0;
      goalInitialized[i] = true;
    }
  }

  bool movedThisCycle = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (encoders[i] != nullptr && pulPins[i] != -1) {
      long errorTicks = 0 - encoders[i]->read();
      
      if (abs(errorTicks) > zeroTolerance) {
        long stepsNeeded = abs(errorTicks) / encMult[i];
        if (stepsNeeded == 0) stepsNeeded = 1;
        
        if ((errorTicks * encDir[i]) < 0) {
          stepsNeeded = -stepsNeeded;
        }

        steppers[i].move(stepsNeeded);
        movedThisCycle = true;
      }
    }
  }

  if (movedThisCycle) {
    bool stillRunning = true;
    while (stillRunning) {
      stillRunning = false;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (pulPins[i] != -1 && steppers[i].distanceToGo() != 0) {
          steppers[i].run();
          stillRunning = true;
        }
      }
    }
    delay(50);
  }

  Serial.println("All active encoders successfully returned to zero.");
  if (encoders[activeMotorIdx] != nullptr) {
    Serial.print("-> Current Encoder Count: ");
    Serial.println(encoders[activeMotorIdx]->read());
  }
}

void jogMotorSerial() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case '+':
      case 'd':
      case 'D':
        Serial.print("Jogging HIGH: ");
        Serial.print(jogSteps);
        Serial.println(" steps");
        moveMotorWithPrediction(activeMotorIdx, jogSteps, HIGH);
        break;

      case '-':
      case 'a':
      case 'A':
        Serial.print("Jogging LOW: ");
        Serial.print(jogSteps);
        Serial.println(" steps");
        moveMotorWithPrediction(activeMotorIdx, jogSteps, LOW);
        break;

      case '1':
        activeMotorIdx = 0;
        Serial.println("Selected Motor: J1");
        break;

     case '2':
        activeMotorIdx = 1;
        Serial.println("Selected Motor: J2");
        break; 

     case '3':
        activeMotorIdx = 2;
        Serial.println("Selected Motor: J3");
        break; 

      case 's':
      case 'S': {
        int newSteps = Serial.parseInt();
        if (newSteps > 0) {
          jogSteps = newSteps;
          Serial.print("Updated jogSteps to: ");
          Serial.println(jogSteps);
        }
        break;
      }
      
      case 'v':
      case 'V': {
        float newSpeed = Serial.parseFloat();
        if (newSpeed > 0) {
          speedLimit = newSpeed;
          for (int i = 0; i < NUM_MOTORS; i++) {
            if (pulPins[i] != -1) steppers[i].setMaxSpeed(speedLimit);
          }
          Serial.print("Updated Max Speed to: ");
          Serial.println(speedLimit);
        }
        break;
      }

      case 'q':
      case 'Q': {
        float newAccel = Serial.parseFloat();
        if (newAccel > 0) {
          accelLimit = newAccel;
          for (int i = 0; i < NUM_MOTORS; i++) {
            if (pulPins[i] != -1) steppers[i].setAcceleration(accelLimit);
          }
          Serial.print("Updated Acceleration to: ");
          Serial.println(accelLimit);
        }
        break;
      }

      case 'c':
      case 'C':
        enableClosedLoopGoal = !enableClosedLoopGoal;
        if (enableClosedLoopGoal) {
          for (int i = 0; i < NUM_MOTORS; i++) {
            if (encoders[i] != nullptr) targetEncGoal[i] = encoders[i]->read();
          }
        }
        Serial.print("Closed-Loop Goal Maintenance: ");
        Serial.println(enableClosedLoopGoal ? "ENABLED" : "DISABLED");
        break;

      case 'e':
      case 'E':
        if (encoders[activeMotorIdx] != nullptr) {
          Serial.print("Active Motor Encoder Count: ");
          Serial.print(encoders[activeMotorIdx]->read());
          Serial.print(" | Target Goal: ");
          Serial.println(targetEncGoal[activeMotorIdx]);
        } else {
          Serial.println("No encoder on active motor.");
        }
        break;
        
      case 'r':
      case 'R':
        if (encoders[activeMotorIdx] != nullptr) {
          encoders[activeMotorIdx]->write(0);
          targetEncGoal[activeMotorIdx] = 0;
          goalInitialized[activeMotorIdx] = true;
          Serial.println("Active Motor Encoder Count and Target Goal reset to 0.");
        }
        break;

      case 'z':
      case 'Z':
        returnToZero();
        break;

      case '\n':
      case '\r':
      case ' ':
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        break;
    }
  }
}

void testAllMotors(int steps, uint8_t direction) {
  long stepTarget = (direction == HIGH) ? steps : -steps;
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (pulPins[i] != -1) steppers[i].move(stepTarget);
  }

  bool stillRunning = true;
  while (stillRunning) {
    stillRunning = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (pulPins[i] != -1 && steppers[i].distanceToGo() != 0) {
        steppers[i].run();
        stillRunning = true;
      }
    }
  }
}

void loop() {
  while (testSerialJog && !testOneMotor && !testAll) {
    jogMotorSerial();
    maintainEncoderGoals();
  }

  while (testOneMotor && !testAll) {
    moveMotorWithPrediction(0, stepsPer90, LOW);
    delay(500);
    moveMotorWithPrediction(0, stepsPer90, HIGH);
    delay(1000);
    moveMotorWithPrediction(0, stepsPer90, HIGH);
    delay(500);
    moveMotorWithPrediction(0, stepsPer90, LOW);
    delay(1000);
  }

  while (testAll && safety && !testOneMotor) {
    testAllMotors(stepsPer90, LOW);
    delay(500);
    testAllMotors(stepsPer90, HIGH);
    delay(1000);
    testAllMotors(stepsPer90, HIGH);
    delay(500);
    testAllMotors(stepsPer90, LOW);
    delay(1000);
  }
}