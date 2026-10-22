#define J1_PUL_PIN 0
#define J1_DIR_PIN 1
#define J2_PUL_PIN 2
#define J2_DIR_PIN 3

// Define testing mode flags
#define testSerialJog true
#define testOneMotor false
#define testAll false

// CHECK THAT ROBOT IS IN ZERO CONFIG!!! HANDS ON ESTOP!!!!!!!!!!!
#define safety false

#define TEST_DIR_PIN J2_DIR_PIN
#define TEST_PUL_PIN J2_PUL_PIN

// 800 steps per revolution (360 degrees)
const int stepsPer90 = 6400; 

// Default steps per single serial jog command
int jogSteps = 200;

// Adjust to change motor speed (lower is faster)
const int stepDelay = 200; 

// Track currently selected motor for serial jogging (defaults to J2)
int activeDirPin = TEST_DIR_PIN;
int activePulPin = TEST_PUL_PIN;

void setup() {
  pinMode(J1_PUL_PIN, OUTPUT);
  pinMode(J1_DIR_PIN, OUTPUT);
  pinMode(J2_PUL_PIN, OUTPUT);
  pinMode(J2_DIR_PIN, OUTPUT);

  Serial.begin(115200);
  // Wait up to 3 seconds for Teensy native USB Serial to connect
  while (!Serial && millis() < 3000) {}

  Serial.println("=== Teensy 4.1 Motor Serial Jog Ready ===");
  Serial.println("Commands:");
  Serial.println("  + or d : Jog HIGH (Right/Forward) by jogSteps");
  Serial.println("  - or a : Jog LOW  (Left/Reverse)  by jogSteps");
  Serial.println("  1      : Select Motor J1 (Pins 0/1)");
  Serial.println("  2      : Select Motor J2 (Pins 2/3)");
  Serial.println("  s[num] : Set jog step count (e.g., s800)");
}

void testMotor(int dirPin, int pulPin, int steps, uint8_t direction) {
  digitalWrite(dirPin, direction);
  delayMicroseconds(5); // Wait for stepper driver direction setup time (600 MHz MCU)
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

// Serial Jog Function
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
        testMotor(activeDirPin, activePulPin, jogSteps, HIGH);
        break;

      case '-':
      case 'a':
      case 'A':
        Serial.print("Jogging LOW: ");
        Serial.print(jogSteps);
        Serial.println(" steps");
        testMotor(activeDirPin, activePulPin, jogSteps, LOW);
        break;

      case '1':
        activeDirPin = J1_DIR_PIN;
        activePulPin = J1_PUL_PIN;
        Serial.println("Selected Motor: J1");
        break;

      case '2':
        activeDirPin = J2_DIR_PIN;
        activePulPin = J2_PUL_PIN;
        Serial.println("Selected Motor: J2");
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

// ENSURE ALL MOTORS ARE IN ZERO CONFIG BEFORE RUNNING THIS CODE
// HAND ON ESTOP FOR FIRST FEW RUNS
void testAllMotors(int steps, uint8_t direction) {
  digitalWrite(J1_DIR_PIN, direction);
  digitalWrite(J2_DIR_PIN, direction);
  delayMicroseconds(5); // Direction setup time
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(J1_PUL_PIN, HIGH);
    digitalWrite(J2_PUL_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(J1_PUL_PIN, LOW);
    digitalWrite(J2_PUL_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

void loop() {
  while (testSerialJog && !testOneMotor && !testAll) {
    jogMotorSerial();
  }

  while (testOneMotor && !testAll) {
    testMotor(TEST_DIR_PIN, TEST_PUL_PIN, stepsPer90, LOW);
    delay(500);
    testMotor(TEST_DIR_PIN, TEST_PUL_PIN, stepsPer90, HIGH);
    delay(1000);
    testMotor(TEST_DIR_PIN, TEST_PUL_PIN, stepsPer90, HIGH);
    delay(500);
    testMotor(TEST_DIR_PIN, TEST_PUL_PIN, stepsPer90, LOW);
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