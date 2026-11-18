// Smart Ballot Station - ARD_GROUP_22 (physical Arduino UNO version)
// Automation and Computer-Aided Manufacturing 364-1-3321, BGU
// ===================== Libraries =====================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.hpp>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>
// ===================== Pins =====================
#define MANAGER_BUTTON_PIN    2   // INPUT  - manager button, 10k pull-down (pressed = HIGH)
#define FINGERPRINT_TX_PIN    3   // OUTPUT - Arduino sends to the fingerprint sensor
#define FINGERPRINT_RX_PIN    4   // INPUT  - Arduino receives from the fingerprint sensor
#define ULTRASONIC_ECHO_PIN   5   // INPUT  - ultrasonic echo
#define ULTRASONIC_TRIG_PIN   6   // OUTPUT - ultrasonic trigger
#define IR_RECEIVER_PIN       7   // INPUT  - IR receiver signal
#define YELLOW_LED_PIN        8   // OUTPUT - voting in progress
#define SERVO_PIN             9   // OUTPUT - gate servo (PWM)
#define GREEN_LED_PIN        12   // OUTPUT - station ready / entry allowed
// LCD (I2C): SDA -> A4, SCL -> A5
// ===================== Settings =====================
#define LCD_ADDRESS        0x27    // if the screen stays empty, try 0x3F
#define GATE_CLOSED_ANGLE     0
#define GATE_OPEN_ANGLE      90
#define GATE_OPEN_TIME_MS  3000UL  // how long the gate stays open
#define SERVO_MIN_PULSE_US  544    // pulse width for 0 degrees (microseconds)
#define SERVO_MAX_PULSE_US 2400    // pulse width for 180 degrees (microseconds)
#define SERVO_PULSE_COUNT    20    // pulses sent per move (20 x 20 ms = about 0.4 s)
#define VOTER_NEAR_CM        50    // closer than this = voter is at the ballot box
#define VOTER_FAR_CM         60    // farther than this = voter has left (gap prevents flicker)
#define DISTANCE_CONFIRM_COUNT 3   // readings in a row needed before the voter counts as arrived
#define VOTER_LEFT_CONFIRM_MS 1500UL  // the voter must stay away this long before the exit opens
#define NO_ECHO_CM          999    // returned when nothing is in front of the sensor
#define ADMIN_OK_CODE        28    // OK button code of our physical remote (read in Serial Monitor)
#define ADMIN_DENY_CODE      22    // deny button code ("*" on our remote) - check it in Serial Monitor
#define DENIED_SHOW_MS     3000UL  // how long "Access Denied" stays on the screen
#define NO_IR_CODE           -1
#define FINGER_BAUD_RATE  57600    // default speed of the fingerprint sensor
#define FINGER_CHECK_MS     200    // how often to ask the sensor for a finger
#define FINGER_CONFIRM_COUNT  2    // images in a row needed before a finger is checked
#define BUTTON_CONFIRM_COUNT  4    // steady readings in a row needed (about 0.2 s) - filters noise
#define FINGER_NONE           0    // no finger (empty sensor, unreadable image, or same finger as before)
#define FINGER_MATCHED        1    // known finger
#define FINGER_REJECTED       2    // real fingerprint, but not in the sensor's memory
#define MAX_VOTER_ID        128    // fingerprint IDs 0-127 are tracked for "already voted"
// ===================== LCD messages =====================
#define MSG_WAITING           0    // "Waiting for Voter"
#define MSG_PLACE_FINGER      1    // "Please Place Finger"
#define MSG_APPROVED          2    // "Approved"
#define MSG_ADMIN_OVERRIDE    3    // "Admin Override"
#define MSG_VOTING            4    // "Voting in Progress"
#define MSG_NOT_RECOGNIZED    5    // "Not Recognized / Wait for Admin"
#define MSG_ACCESS_DENIED     6    // "Access Denied / Cannot Vote"
#define MSG_ALREADY_VOTED     7    // "Already Voted / Access Denied"
#define MSG_VOTING_DONE       8    // "Voting Finished / Please Exit"
// ===================== States =====================
enum StationState {
  STATE_IDLE,             // State 0: waiting for the manager
  STATE_READY,            // Step 1: waiting for fingerprint or admin override
  STATE_ENTRY_GATE_OPEN,  // Step 2: gate open so the voter can enter
  STATE_WAIT_FOR_VOTER,   // Step 2->3: gate closed, voter walking to the ballot box
  STATE_VOTING,           // Step 3: voter is at the ballot box
  STATE_EXIT_GATE_OPEN,   // Step 4: gate open so the voter can leave
  STATE_ACCESS_DENIED     // Step 2 (rejected): showing "Access Denied", gate stays closed
};
// ===================== Global objects and variables =====================
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
SoftwareSerial fingerSerial(FINGERPRINT_RX_PIN, FINGERPRINT_TX_PIN);  // (receive pin, send pin)
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&fingerSerial);
StationState currentState = STATE_IDLE;
unsigned long gateOpenedAt = 0;      // millis() at the moment the gate opened
unsigned long deniedAt = 0;          // millis() at the moment the manager denied access
unsigned long lastFingerCheckAt = 0; // millis() of the last question to the fingerprint sensor
bool waitForFingerLift = false;      // after a checked finger, wait until the sensor is empty again
int fingerSeenCount = 0;             // how many images in a row showed something on the sensor
int buttonReleasedCount = 0;         // how many readings in a row saw the button released
int buttonPressedCount = 0;          // how many readings in a row saw the button pressed
int distanceConfirmCount = 0;        // how many readings in a row saw the voter near the box
bool voterAway = false;              // true while the readings show the voter away from the box
unsigned long voterAwaySince = 0;    // millis() when the voter was first seen away
int currentMessageId = -1;           // the message now on the LCD, so it can be redrawn
int voterCount = 0;                  // voters who voted
int deniedCount = 0;                 // voters who were denied
int matchedFingerId = -1;            // ID of the last fingerprint that was recognized
bool hasVoted[MAX_VOTER_ID];         // hasVoted[id] = true once that fingerprint voted (starts all false)
// ===================== Function prototypes =====================
void handleIdle();
void handleReady(int irCode);
void handleEntryGateOpen();
void handleWaitForVoter();
void handleVoting();
void handleExitGateOpen();
void handleAccessDenied();
void denyAccess(int messageId);
void enterIdleState();
void openGate();
void closeGate();
void moveGate(int angle);
void showMessage(int messageId);
void refreshLcd();
bool isManagerButtonPressed();
int checkFingerprint();
int readIrCode();
long readDistanceCm();
// ===================== Setup =====================
void setup() {
  Serial.begin(9600);
  pinMode(MANAGER_BUTTON_PIN, INPUT);  // external 10k pull-down keeps the pin LOW until pressed
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  lcd.init();
  lcd.backlight();
  IrReceiver.begin(IR_RECEIVER_PIN);
  finger.begin(FINGER_BAUD_RATE);
  if (finger.verifyPassword()) {
    Serial.println(F("Fingerprint sensor found"));
  } else {
    // The station still works with the IR admin override
    Serial.println(F("Fingerprint sensor NOT found - check wiring, or swap pins 3 and 4"));
  }
  if (digitalRead(MANAGER_BUTTON_PIN) == HIGH) {
    Serial.println(F("Warning: manager button reads PRESSED at startup - check the 10k resistor"));
  }
  Serial.println(F("=== Smart Ballot Station started ==="));  // F() keeps text out of RAM
  closeGate();
  enterIdleState();
}
// ===================== Main loop =====================
void loop() {
  int irCode = readIrCode();  // read every loop so an old press is never "saved for later"
  switch (currentState) {
    case STATE_IDLE:            handleIdle();          break;
    case STATE_READY:           handleReady(irCode);   break;
    case STATE_ENTRY_GATE_OPEN: handleEntryGateOpen(); break;
    case STATE_WAIT_FOR_VOTER:  handleWaitForVoter();  break;
    case STATE_VOTING:          handleVoting();        break;
    case STATE_EXIT_GATE_OPEN:  handleExitGateOpen();  break;
    case STATE_ACCESS_DENIED:   handleAccessDenied();  break;
  }
  delay(50);  // short pause between checks
}
// ===================== State handlers =====================
// State 0: wait for the manager to press the button
void handleIdle() {
  if (isManagerButtonPressed()) {
    Serial.println(F("[Step 1] Manager pressed button -> station ready"));
    digitalWrite(GREEN_LED_PIN, HIGH);
    showMessage(MSG_PLACE_FINGER);
    waitForFingerLift = false;
    fingerSeenCount = 0;
    currentState = STATE_READY;
  }
}
// Step 1 -> 2: wait for a fingerprint match, an admin override, or an admin denial.
// Stays here as long as needed - nothing moves on without one of them.
void handleReady(int irCode) {
  int fingerResult = checkFingerprint();
  if (fingerResult == FINGER_MATCHED && matchedFingerId < MAX_VOTER_ID && hasVoted[matchedFingerId]) {
    Serial.print(F("[Step 2] Fingerprint ID "));
    Serial.print(matchedFingerId);
    Serial.println(F(" already voted -> access denied"));
    denyAccess(MSG_ALREADY_VOTED);
  } else if (fingerResult == FINGER_MATCHED) {
    if (matchedFingerId < MAX_VOTER_ID) {
      hasVoted[matchedFingerId] = true;  // this fingerprint cannot vote again
    }
    Serial.println(F("[Step 2] Fingerprint matched -> Approved"));
    showMessage(MSG_APPROVED);
    openGate();
    currentState = STATE_ENTRY_GATE_OPEN;
  } else if (irCode == ADMIN_OK_CODE) {
    Serial.println(F("[Step 2] IR OK from manager -> Admin Override"));
    showMessage(MSG_ADMIN_OVERRIDE);
    openGate();
    currentState = STATE_ENTRY_GATE_OPEN;
  } else if (irCode == ADMIN_DENY_CODE) {
    Serial.println(F("[Step 2] Manager denied access -> voter cannot vote, gate stays closed"));
    denyAccess(MSG_ACCESS_DENIED);
  } else if (fingerResult == FINGER_REJECTED) {
    Serial.println(F("[Step 2] Not recognized -> waiting for manager OK or deny on the remote"));
    showMessage(MSG_NOT_RECOGNIZED);
  }
}
// Step 2 (rejected): green LED off, show the denial message, gate stays closed
void denyAccess(int messageId) {
  digitalWrite(GREEN_LED_PIN, LOW);
  showMessage(messageId);
  deniedCount++;
  deniedAt = millis();
  currentState = STATE_ACCESS_DENIED;
}
// Step 2 (rejected): show the denial for DENIED_SHOW_MS, then get ready for the next voter
void handleAccessDenied() {
  if (millis() - deniedAt >= DENIED_SHOW_MS) {
    Serial.print(F("[Step 2] Denied voter sent away. Total denied: "));
    Serial.println(deniedCount);
    enterIdleState();
  }
}
// Step 2: keep the gate open for GATE_OPEN_TIME_MS, then close it and turn the green LED off
void handleEntryGateOpen() {
  if (millis() - gateOpenedAt >= GATE_OPEN_TIME_MS) {
    closeGate();
    digitalWrite(GREEN_LED_PIN, LOW);
    Serial.println(F("[Step 2] Green LED off, waiting for voter to reach the ballot box"));
    distanceConfirmCount = 0;
    currentState = STATE_WAIT_FOR_VOTER;
  }
}
// Step 3: wait until the voter is close to the ballot box (several readings in a row)
void handleWaitForVoter() {
  long distance = readDistanceCm();
  if (distance < VOTER_NEAR_CM) {
    distanceConfirmCount++;
  } else {
    distanceConfirmCount = 0;
  }
  if (distanceConfirmCount >= DISTANCE_CONFIRM_COUNT) {
    distanceConfirmCount = 0;
    Serial.print(F("[Step 3] Voter detected at "));
    Serial.print(distance);
    Serial.println(F(" cm -> Voting in progress"));
    digitalWrite(YELLOW_LED_PIN, HIGH);
    showMessage(MSG_VOTING);
    voterAway = false;
    currentState = STATE_VOTING;
  }
}
// Step 4: wait until the voter has stayed away for VOTER_LEFT_CONFIRM_MS, then open the gate.
// One close reading resets the wait, so a few bad readings while the voter is still
// at the box never count as leaving.
void handleVoting() {
  long distance = readDistanceCm();
  if (distance <= VOTER_FAR_CM) {
    voterAway = false;  // the voter is still at the box
    return;
  }
  if (!voterAway) {
    voterAway = true;
    voterAwaySince = millis();
    return;
  }
  if (millis() - voterAwaySince >= VOTER_LEFT_CONFIRM_MS) {
    voterAway = false;
    Serial.print(F("[Step 4] Voter moved away ("));
    if (distance == NO_ECHO_CM) {
      Serial.print(F("nothing in front of sensor"));
    } else {
      Serial.print(distance);
      Serial.print(F(" cm"));
    }
    Serial.println(F(") -> voting finished, gate opens for exit"));
    digitalWrite(YELLOW_LED_PIN, LOW);
    showMessage(MSG_VOTING_DONE);
    openGate();
    currentState = STATE_EXIT_GATE_OPEN;
  }
}
// Step 4: keep the gate open for the exit, then close it and return to State 0
void handleExitGateOpen() {
  if (millis() - gateOpenedAt >= GATE_OPEN_TIME_MS) {
    closeGate();
    voterCount++;
    Serial.print(F("[Step 4] Voter has left. Total voters: "));
    Serial.println(voterCount);
    enterIdleState();
  }
}
// ===================== Helper functions =====================
// State 0: LEDs off, waiting message (the gate is already closed)
void enterIdleState() {
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  showMessage(MSG_WAITING);
  Serial.println(F("[State 0] Waiting for voter"));
  currentState = STATE_IDLE;
}
// Raises the gate and remembers when, so the timer can close it later
void openGate() {
  moveGate(GATE_OPEN_ANGLE);
  refreshLcd();
  gateOpenedAt = millis();
  Serial.println(F("Gate OPEN"));
}
// Lowers the gate
void closeGate() {
  moveGate(GATE_CLOSED_ANGLE);
  refreshLcd();
  Serial.println(F("Gate CLOSED"));
}
// Turns the servo to an angle by sending its control signal directly:
// a HIGH pulse of 0.5-2.4 ms, repeated every 20 ms, sets the angle (0-180 degrees).
// Short and bounded (about 0.4 s), so the sensors are not blocked for long.
void moveGate(int angle) {
  int pulseWidth = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  for (int i = 0; i < SERVO_PULSE_COUNT; i++) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);
    delay(20);
  }
}
// Clears the LCD and shows one of the station messages (16 characters max per line)
void showMessage(int messageId) {
  currentMessageId = messageId;
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (messageId) {
    case MSG_WAITING:
      lcd.print("Waiting for");
      lcd.setCursor(0, 1);
      lcd.print("Voter");
      break;
    case MSG_PLACE_FINGER:
      lcd.print("Please Place");
      lcd.setCursor(0, 1);
      lcd.print("Finger");
      break;
    case MSG_APPROVED:
      lcd.print("Approved");
      break;
    case MSG_ADMIN_OVERRIDE:
      lcd.print("Admin Override");
      break;
    case MSG_VOTING:
      lcd.print("Voting in");
      lcd.setCursor(0, 1);
      lcd.print("Progress");
      break;
    case MSG_NOT_RECOGNIZED:
      lcd.print("Not Recognized");
      lcd.setCursor(0, 1);
      lcd.print("Wait for Admin");
      break;
    case MSG_ACCESS_DENIED:
      lcd.print("Access Denied");
      lcd.setCursor(0, 1);
      lcd.print("Cannot Vote");
      break;
    case MSG_ALREADY_VOTED:
      lcd.print("Already Voted");
      lcd.setCursor(0, 1);
      lcd.print("Access Denied");
      break;
    case MSG_VOTING_DONE:
      lcd.print("Voting Finished");
      lcd.setCursor(0, 1);
      lcd.print("Please Exit");
      break;
  }
}
// The servo draws a burst of current when it moves, which can scramble the LCD
// (random characters and numbers). Restarting the LCD and drawing the current
// message again after every gate move fixes that.
void refreshLcd() {
  lcd.init();
  lcd.backlight();
  showMessage(currentMessageId);
}
// Returns true once per real press of the manager button.
// The button must first be seen released for BUTTON_CONFIRM_COUNT readings in a row,
// and then pressed for BUTTON_CONFIRM_COUNT readings in a row. Short spikes of
// electrical noise last less than that, so they never count as a press.
bool isManagerButtonPressed() {
  if (digitalRead(MANAGER_BUTTON_PIN) == LOW) {
    buttonPressedCount = 0;
    if (buttonReleasedCount < BUTTON_CONFIRM_COUNT) {
      buttonReleasedCount++;
    }
    return false;
  }
  if (buttonReleasedCount < BUTTON_CONFIRM_COUNT) {
    buttonPressedCount = 0;
    buttonReleasedCount = 0;  // not steadily released yet - start counting again
    return false;
  }
  buttonPressedCount++;
  if (buttonPressedCount >= BUTTON_CONFIRM_COUNT) {
    buttonPressedCount = 0;
    buttonReleasedCount = 0;  // the next press needs a release first
    return true;
  }
  return false;
}
// Asks the fingerprint sensor about the finger on it, once per finger placement.
// An empty sensor, or an image with no readable fingerprint, counts as "no finger":
// only a real fingerprint that is not in memory is reported as FINGER_REJECTED.
int checkFingerprint() {
  if (millis() - lastFingerCheckAt < FINGER_CHECK_MS) {
    return FINGER_NONE;
  }
  // Talking to the sensor briefly pauses interrupts, which can spoil an IR signal
  // that is arriving, so the sensor is asked only while the IR receiver is quiet.
  if (!IrReceiver.isIdle()) {
    return FINGER_NONE;
  }
  lastFingerCheckAt = millis();
  if (finger.getImage() != FINGERPRINT_OK) {
    fingerSeenCount = 0;
    waitForFingerLift = false;  // sensor is empty - ready for the next finger
    return FINGER_NONE;
  }
  if (waitForFingerLift) {
    return FINGER_NONE;  // the same finger is still on the sensor
  }
  fingerSeenCount++;
  if (fingerSeenCount < FINGER_CONFIRM_COUNT) {
    return FINGER_NONE;  // wait for a steady finger
  }
  if (finger.image2Tz() != FINGERPRINT_OK) {
    return FINGER_NONE;  // no readable fingerprint in the image - not a finger yet
  }
  waitForFingerLift = true;  // this finger is checked only once
  if (finger.fingerFastSearch() != FINGERPRINT_OK) {
    Serial.println(F("Fingerprint not in memory"));
    return FINGER_REJECTED;
  }
  matchedFingerId = finger.fingerID;
  Serial.print(F("Fingerprint ID "));
  Serial.print(finger.fingerID);
  Serial.print(F(", confidence "));
  Serial.println(finger.confidence);
  return FINGER_MATCHED;
}
// Returns the code of a new IR button press, or NO_IR_CODE if there is none.
// Noise (unknown signals) and repeats from holding a button down are ignored.
int readIrCode() {
  if (!IrReceiver.decode()) {
    return NO_IR_CODE;
  }
  int code = IrReceiver.decodedIRData.command;
  bool isNoise = (IrReceiver.decodedIRData.protocol == UNKNOWN);
  bool isRepeat = (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT);
  IrReceiver.resume();  // get ready for the next press
  if (isNoise || isRepeat) {
    return NO_IR_CODE;
  }
  Serial.print(F("IR code received: "));
  Serial.println(code);
  return code;
}
// Sends an ultrasonic pulse and converts the echo time to centimeters:
// distance = time * speed of sound (0.0343 cm/us) / 2 (the sound goes there and back)
long readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  unsigned long echoTime = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000UL);  // 30 ms timeout (~5 m)
  if (echoTime == 0) {
    return NO_ECHO_CM;
  }
  return echoTime * 0.0343 / 2;
}
