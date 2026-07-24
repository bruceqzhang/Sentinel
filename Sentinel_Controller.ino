// =============================================================================
// Sentinel Controller
// Bryan Zhang, Bruce Zhang
// June 19, 2026
//
// A program designed to transform user inputs on an Xbox Controller into radio
// information to be sent over a 2.4 GHz channel communication link to the Sentinel, 
// allowing user control of the tank.
//
// Hardware:
//   - Arduino UNO R3 (U2)
//   - USB Host Shield (U3)  → D10 CS, D11 MOSI, D12 MISO, D13 SCK, D2 INT
//   - Xbox One Controller (U5)   → USB Host Shield
//   - nRF24L01+PA+LNA (U6) → D9 CSN, D8 CE, D11 MOSI, D12 MISO, D13 SCK
//
// Control Scheme:
//   Left Joystick Y  → Drive forward / backward
//   Left Joystick X  → Pivot / turn (tank drive)
//   Right Joystick X → Rotate turret
//   Left Trigger     → Rev flywheels (analog speed)
//   Right Trigger    → Shoot dart (only fires if flywheels spinning)
//   X                → Toggle laser
//   D-Pad U/D        → Barrel elevation setpoints (3° increments)
// =============================================================================


#include <XBOXONE.h>
#include <SPI.h>
#include <RF24.h>


// ── nRF24L01 pins ────────────────────────────────────────────────────────────
#define NRF_CE_PIN  8
#define NRF_CSN_PIN 4


// ── Radio ─────────────────────────────────────────────────────────────────────
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte ADDRESS[6] = "SNTL1";   


// ── USB / Xbox ────────────────────────────────────────────────────────────────
USB  Usb;
XBOXONE Xbox(&Usb);


// ── Payload struct (max 32 bytes) ─────────────────────────────────────────────
// Total: 2+2+2+2+1+1+1 = 11 bytes
struct Payload {
  int16_t  leftX;       // -32768 to 32767
  int16_t  leftY;
  int16_t  rightX;
  int16_t  rightY;
  uint8_t  leftTrig;    // 0-255
  uint8_t  rightTrig;   // 0-255
  uint8_t  buttons;    
};


// ── Button bitmask positions ──────────────────────────────────────────────────
#define BTN_X          (1 << 1)
#define BTN_DPAD_UP    (1 << 6)
#define BTN_DPAD_DOWN  (1 << 7)


// ── Deadband ──────────────────────────────────────────────────────────────────
#define DEADBAND 7500


// ── Exponential curve helper ──────────────────────────────────────────────────
// Applies a mild square curve while preserving sign.
// Input/output: -32767 to 32767
int16_t expoCurve(int16_t raw) {
  if (abs(raw) < DEADBAND) return 0;
  float n = (float)raw / 32767.0f;
  // blend: 50% linear + 50% cubic
  float curved = 0.5f * n + 0.5f * n * n * n;
  return (int16_t)(curved * 32767.0f);
}


// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);


  // force both spi devices silent immediately
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH); // De-select USB Host Shield  
  pinMode(NRF_CSN_PIN, OUTPUT);
  digitalWrite(NRF_CSN_PIN, HIGH); // De-select nRF24L01  
  delay(100); 


  // initialize usb shield
  Serial.println(F("Initializing USB Host Shield..."));
  if (Usb.Init() == -1) {
    Serial.println(F("USB Host Shield did not start"));
    while (1);
  }
  digitalWrite(10, HIGH); // Ensure it releases the SPI bus immediately after 
  Serial.println(F("USB Host Shield started"));




  // initialize nrf24l01
  Serial.println(F("Initializing nRF24L01..."));
  if (!radio.begin()) {
    Serial.println(F("nRF24L01 not found"));
    while (1);
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);            
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();            
 
  digitalWrite(NRF_CSN_PIN, HIGH); // Ensure radio releases the bus
  Serial.println(F("nRF24L01 ready — Sentinel Controller online"));
 
  Xbox.setRumbleOff();
}


// ── State for non-blocking transmission timer ───────────────────────────────
unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL_MS = 20;


// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  Usb.Task();


  if (!Xbox.XboxOneConnected) return;


  // ── Read raw joystick values ────────────────────────────────────────────────
  int16_t rawLX = Xbox.getAnalogHat(LeftHatX);
  int16_t rawLY = Xbox.getAnalogHat(LeftHatY);
  int16_t rawRX = Xbox.getAnalogHat(RightHatX);
  int16_t rawRY = Xbox.getAnalogHat(RightHatY);


  // Apply deadband + expo curve
  int16_t lx = expoCurve(rawLX);
  int16_t ly = expoCurve(rawLY);
  int16_t rx = expoCurve(rawRX);
  int16_t ry = expoCurve(rawRY);


 
  // ── Triggers ───────────────────────────────────────────────────────────────
  uint8_t lt = (uint8_t)map(Xbox.getButtonPress(LT), 0, 1023, 0, 255);
  uint8_t rt = (uint8_t)map(Xbox.getButtonPress(RT), 0, 1023, 0, 255);


  // ── Button clicks ───────
  static uint8_t accumulatedButtons = 0;


  // Check clicks continuously on every fast loop pass
  if (Xbox.getButtonClick(X))         accumulatedButtons |= BTN_X;
  if (Xbox.getButtonClick(UP))        accumulatedButtons |= BTN_DPAD_UP;
  if (Xbox.getButtonClick(DOWN))      accumulatedButtons |= BTN_DPAD_DOWN;


  // ── Non-Blocking Transmission Engine ───────────────────────────────────────
  unsigned long now = millis();
  if (now - lastTxTime >= TX_INTERVAL_MS) {
    lastTxTime = now;


    // Pack the payload using our accumulated buttons
    Payload pkt;
    pkt.leftX     = lx;
    pkt.leftY     = ly;
    pkt.rightX    = rx;
    pkt.rightY    = ry;
    pkt.leftTrig  = lt;
    pkt.rightTrig = rt;
    pkt.buttons   = accumulatedButtons;
    // Send data over nRF24
    bool ok = radio.write(&pkt, sizeof(pkt));
    // Clear the accumulator so we can collect fresh clicks for the next packet
    accumulatedButtons = 0;


     }
}
