#include <Servo.h>

// ——— Patch the Encoder library for RP2040 ———
// 1) force polling fallback
#define ENCODER_DO_NOT_USE_INTERRUPTS
// 2) stub out the AVR register types/macros it expects
#define IO_REG_TYPE             int
#define PIN_TO_BASEREG(P)       (P)
#define PIN_TO_BITMASK(P)       (P)
#define DIRECT_PIN_READ(PIN,MSK) digitalRead(PIN)

#include <Encoder.h>
#include <Bounce2.h>
// Servo pins.
const int SERVO_LIFT_PIN  = 2;
const int SERVO_LEFT_PIN  = 3;
const int SERVO_RIGHT_PIN = 4;

// Encoder pins.
const int ENCODER_L_PIN1     = 6;
const int ENCODER_L_PIN2     = 7;
const int ENCODER_L_BTN_PIN  = 8;

const int ENCODER_R_PIN1     = 9;
const int ENCODER_R_PIN2     = 10;
const int ENCODER_R_BTN_PIN  = 11;

const int ENCODER_LIFT_PIN1  = 12;
const int ENCODER_LIFT_PIN2  = 13;
const int ENCODER_LIFT_INIT  = 70;  // starting “angle”

// Calibration & geometry constants
const int SERVO_LEFT_FACTOR  = 630;
const int SERVO_RIGHT_FACTOR = 640;
const int Z_OFFSET           = 230;
const int LIFT0              = 1110 + Z_OFFSET;
const int LIFT1              = 925  + Z_OFFSET;
const int LIFT2              = 735  + Z_OFFSET;
const int SERVO_LEFT_NULL    = 1950;
const int SERVO_RIGHT_NULL   = 815;
const int WISHY              = 3;
const int LIFT_SPEED         = 2000;  // delayMicroseconds()

const float L1 = 35.0, L2 = 55.1, L3 = 13.2, L4 = 45.0;
const int   O1X = 24, O1Y = -25, O2X = 49, O2Y = -25;

const volatile double HOME_X = 72.2, HOME_Y = 45.5;
const float SCALE = 0.9;
const int   MOVE_DELAY_MS = 2;

Bounce btnLeft  = Bounce();
Bounce btnRight = Bounce();

Servo   servo_lift, servo_left, servo_right;
Encoder encoderL(ENCODER_L_PIN1,   ENCODER_L_PIN2),
        encoderR(ENCODER_R_PIN1,   ENCODER_R_PIN2),
        encoderLift(ENCODER_LIFT_PIN1, ENCODER_LIFT_PIN2);

int encPosL, encLastPosL = -1;
int encPosR, encLastPosR = -1;
int encPosLift, encLastPosLift = -1;

void setup() {
  Serial.begin(9600);

  int currentMode = 0;

  // Attach servos
  servo_lift .attach(SERVO_LIFT_PIN,  500, 2500);
  servo_left .attach(SERVO_LEFT_PIN,  500, 2500);
  servo_right.attach(SERVO_RIGHT_PIN, 500, 2500);

  // Initialize encoders
  encoderL.write    (HOME_X * 4);
  encoderR.write    (HOME_Y * 4);
  encoderLift.write (ENCODER_LIFT_INIT * 4);

  // Setup Bounce2 buttons
  Bounce btnLeft, btnRight;

  btnLeft .attach(ENCODER_L_BTN_PIN,  INPUT_PULLUP);
  btnLeft .interval(10);
  btnRight.attach(ENCODER_R_BTN_PIN,  INPUT_PULLUP);
  btnRight.interval(10);

  // If not in calibration, home the pen
  if (currentMode != 0) {
    lift(LIFT2);
    drawTo(HOME_X, HOME_Y);
    lift(LIFT0);
  }

  delay(2000);
}

void loop() {
  btnLeft.update();
  btnRight.update();
  handleEncoderBtns();

  if (currentMode == 0) {
    // Calibration wiggle
    drawTo(-3, 29.2);
    delay(500);
    drawTo(74.1, 28);
    delay(500);
  }
  else if (currentMode == 1) {
    // Manual control
    if (updateEncoder(encoderL, encPosL, encLastPosL))
      set_XY(encPosL, encPosR);
    if (updateEncoder(encoderR, encPosR, encLastPosR))
      set_XY(encPosL, encPosR);
    if (updateEncoder(encoderLift, encPosLift, encLastPosLift))
      // Standard Servo.write(angle) expects 0–180°
      servo_lift.write(encPosLift);
  }
  else if (currentMode == 2) {
    // Auto mode still present but you can ignore it
    // drawCurrentTime(SCALE);
    delay(1000);
  }
}

// Exactly your original handleEncoderBtns()
void handleEncoderBtns() {
  if (btnLeft.fell()) {
    currentMode++;
    if (currentMode == 3) currentMode = 0;
    Serial.print("Current mode: ");
    Serial.println(currentMode);
  }
  if (btnRight.fell()) {
    erase();
  }
}

// Exactly your original updateEncoder()
bool updateEncoder(Encoder &encoder, int &encPos, int &encLastPos) {
  encPos = encoder.read() / 4;
  if (encPos != encLastPos) {
    encLastPos = encPos;
    return true;
  }
  return false;
}

// All your original draw/kinematics functions follow unchanged:
void drawTo(double pX, double pY) { /* … */ }
double return_angle(double a, double b, double c) { /* … */ }
void set_XY(double Tx, double Ty) { /* … */ }
void lift(int liftTarget) { /* … */ }
void erase() { /* … */ }
void goHome() { /* … */ }
void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) { /* … */ }
void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) { /* … */ }
void drawNumber(float bx, float by, int num, float scale1) { /* … */ }
void drawColon(float bx, float by, float scale1)  { /* … */ }
void drawTime(int h1, int h2, int m1, int m2, float scale1) { /* … */ }
void drawCurrentTime(float scale1) { /* … */ }
