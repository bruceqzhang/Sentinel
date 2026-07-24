// =============================================================================
// Sentinel Tank
// Bryan Zhang, Bruce Zhang
// June 19, 2026
// 
// A program designed to receive radio signals containing data from the Sentinel
// controller, translating the data into actuator and motor outputs, controlling
// the tank according to user control.
//
// Hardware:
//   - Arduino UNO R3 (U11)
//   - KY-008 Laser Transmitting Module (LD1)
//   - SG90 Servo Motor (M1)
//   - BYJ-48 Stepper Motor (M2)
//   - MG996R Servo Motor (M3)
//   - JGB37-550 37mm DC Motors (M4 & M6)
//   - Nerf Size-130 DC Motors (M5 & M7)
// =============================================================================


#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <AccelStepper.h>


// ── Pin definitions ─────────────────────────
#define NRF_CSN_PIN   4
#define NRF_CE_PIN    8


#define MOTOR_A_IN1   5
#define MOTOR_A_IN2   3
#define MOTOR_B_IN1   9
#define MOTOR_B_IN2   6 
#define FLYWHEEL_PIN  7 
#define LASER_PIN     2


#define SHOVER_PIN    A4
#define BARREL_PIN    A5


#define STEP_PIN1     A0
#define STEP_PIN2     A1
#define STEP_PIN3     A2
#define STEP_PIN4     A3


// ── Buttons ─────────────────────────────
#define BTN_X          (1 << 1)
#define BTN_DPAD_UP    (1 << 6)
#define BTN_DPAD_DOWN  (1 << 7)


// ── Payload ─────────────────────────────
struct Payload {
  int16_t  leftX;
  int16_t  leftY;
  int16_t  rightX;
  int16_t  rightY;
  uint8_t  leftTrig;
  uint8_t  rightTrig;
  uint8_t  buttons;
};


// ── Forward declaration ─────────────────
void handlePacket(const Payload &pkt, unsigned long now);


// ── Radio ───────────────────────────────
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte ADDRESS[6] = "SNTL1";


// ── Servos ──────────────────────────────
Servo shoverServo;
Servo barrelServo;


// ── Stepper ─────────────────────────────
#define STEPS_PER_REV 2048
AccelStepper turretStepper(AccelStepper::HALF4WIRE, STEP_PIN1, STEP_PIN3, STEP_PIN2, STEP_PIN4);




// ── State ───────────────────────────────
bool laserOn = false;
bool flywheelsOn = false;
bool shootActive = false;




int barrelAngle = 15;
const int BARREL_MIN = 0;
const int BARREL_MAX = 30;


long turretTarget = 0;




// ── timing ──────────────────────────────
unsigned long lastPacketTime = 0;
const unsigned long WATCHDOG_MS = 1000;


unsigned long lastShoveTime = 0;
const unsigned long SHOVE_INTERVAL = 500;
bool shoverState = false;




// ── startup safety system ──────────
bool systemReady = false;
uint8_t packetCount = 0;
uint8_t lastButtons = 0;


void setMotor(uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -255, 255);


  if (speed >= 0) {
    // Forward: IN1 handles direction (flat HIGH), IN2 handles speed via inverted PWM
    // This allows Pin 9 (IN1) to act as a pure digital pin
    digitalWrite(in1, HIGH);
    analogWrite(in2, 255 - speed);
  }
  else {
    // Backward: IN1 goes LOW, IN2 handles raw speed PWM
    digitalWrite(in1, LOW);
    analogWrite(in2, -speed);
  }
}


// ── Arcade drive ────────────────────────
void arcadeDrive(int16_t joyY, int16_t joyX, int &leftOut, int &rightOut) {
  int throttle = map((long)joyY, -32767L, 32767L, -255L, 255L);
  int turn     = map((long)joyX, -32767L, 32767L, -255L, 255L);


  leftOut  = constrain(throttle + turn, -255, 255);
  rightOut = constrain(throttle - turn, -255, 255);
}


// ── safe stop ───────────────────────────
void safeStop() {
  setMotor(MOTOR_A_IN1, MOTOR_A_IN2, 0);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, 0);
  analogWrite(FLYWHEEL_PIN, 0);


  shoverServo.write(0);
  digitalWrite(LASER_PIN, LOW);


  laserOn = false;
  flywheelsOn = false;
  shootActive = false;
}


// ── setup ───────────────────────────────
void setup() {


  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);


  pinMode(FLYWHEEL_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);


  // immediate safe servo position ──
  delay(300);
  shoverServo.write(0);
  barrelServo.write(barrelAngle);
  shoverServo.attach(SHOVER_PIN);
  barrelServo.attach(BARREL_PIN);


  safeStop();


  turretStepper.setMaxSpeed(1000);
  turretStepper.setAcceleration(400);


  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();


  delay(1000);  // full power stabilization delay


  systemReady = true;
  lastPacketTime = millis();
}


// ── Loop ────────────────────────────────
void loop() {


  if (!systemReady) return;


  unsigned long now = millis();


  if (now - lastPacketTime > WATCHDOG_MS) {
    safeStop();
  }




  Payload pkt;
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    lastPacketTime = now;
    handlePacket(pkt, now);


  }


  // ── shover ───────────────────────────
  if (shootActive && flywheelsOn) {
    if (now - lastShoveTime > SHOVE_INTERVAL) {
      lastShoveTime = now;
      shoverState = !shoverState;
      shoverServo.write(shoverState ? 180 : 0);
    }
  } else {
    shoverServo.write(0);
    shoverState = false;
  }


  // ── turret ───────────────────────────
 turretStepper.run();
}


// ── packet handler ──────────────────────
void handlePacket(const Payload &pkt, unsigned long now) {


  // ── ignore first packets ───────
  if (packetCount < 10) {
    packetCount++;
    lastButtons = pkt.buttons;
    return;
  }


  uint8_t rising = pkt.buttons & ~lastButtons;


  if (rising & BTN_X) {
    laserOn = !laserOn;
    digitalWrite(LASER_PIN, laserOn);
  }


  // ── barrel control ───────────────────
  if (pkt.buttons & BTN_DPAD_UP)
    barrelAngle = constrain(barrelAngle + 3, BARREL_MIN, BARREL_MAX);


  if (pkt.buttons & BTN_DPAD_DOWN)
    barrelAngle = constrain(barrelAngle - 3, BARREL_MIN, BARREL_MAX);


  barrelServo.write(barrelAngle);


  // ── drive ────────────────────────────
  int leftSpeed, rightSpeed;
  arcadeDrive(pkt.leftY, pkt.leftX, leftSpeed, rightSpeed);


  setMotor(MOTOR_A_IN1, MOTOR_A_IN2, leftSpeed);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, -rightSpeed);


  // ── turret input ─────────────────────
  if (abs(pkt.rightX) > 7500) {
    turretTarget += map(pkt.rightX, -32767, 32767, -20, 20);
  }
  turretStepper.moveTo(turretTarget);


  // ── flywheels ────────────────
  flywheelsOn = (pkt.leftTrig > 10);


  if (flywheelsOn) {
    digitalWrite(FLYWHEEL_PIN, HIGH);
  } else {
    digitalWrite(FLYWHEEL_PIN, LOW);
  }


  shootActive = (flywheelsOn && pkt.rightTrig > 10);


  lastButtons = pkt.buttons;
}


