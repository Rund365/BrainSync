/*
 * I2C check for both IMU buses (BrainSync 2.0 bring-up)
 *
 * Upload to the SENDER ESP32, Serial Monitor at 115200 baud.
 * Every 4 seconds it checks both IMU pin pairs and prints what it finds:
 *   - the idle level of each line (LOW at rest = IMU unpowered or a short)
 *   - every I2C address that answers, with SDA/SCL the expected way round
 *     and swapped (to catch swapped wires)
 *   - whether the device is really a BNO055 (chip ID 0xA0)
 *   - a one-line diagnosis per pin pair
 *
 * The firmware expects one BNO055 per pin pair, at address 0x28 (ADR pin to GND).
 * Pin pairs as the sender sketch uses them: first pin = SDA, second = SCL.
 * After the check, upload step_detection_sketch again.
 */
#include <Wire.h>

struct PinPair {
  const char *name;
  int sda;
  int scl;
};

PinPair pairs[] = {
  {"pins 21 (SDA) / 22 (SCL)", 21, 22},
  {"pins 18 (SDA) / 19 (SCL)", 18, 19},
};

const uint32_t I2C_HZ = 100000;  // same speed as the sender firmware
const uint8_t EXPECTED_ADDR = 0x28;
const uint8_t BNO055_CHIP_ID_REG = 0x00;
const uint8_t BNO055_CHIP_ID = 0xA0;
const int MAX_FOUND = 8;

// Reads both lines with the ESP32's pull-ups on, before any I2C traffic.
// Breakout boards pull SDA/SCL up to their own supply, so an unpowered board pulls them LOW.
bool linesIdleHigh(const PinPair &p) {
  pinMode(p.sda, INPUT_PULLUP);
  pinMode(p.scl, INPUT_PULLUP);
  delay(5);
  int sdaLevel = digitalRead(p.sda);
  int sclLevel = digitalRead(p.scl);
  Serial.printf("  idle levels: GPIO %d = %s, GPIO %d = %s\n",
                p.sda, sdaLevel ? "HIGH" : "LOW", p.scl, sclLevel ? "HIGH" : "LOW");
  return sdaLevel && sclLevel;
}

// Scans all addresses with the given SDA/SCL; returns how many answered
int scan(int sda, int scl, uint8_t *found) {
  Wire.begin(sda, scl, I2C_HZ);
  Wire.setTimeOut(20);
  int n = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0 && n < MAX_FOUND) found[n++] = addr;
  }
  return n;
}

bool readChipId(uint8_t addr, uint8_t &id) {
  Wire.beginTransmission(addr);
  Wire.write(BNO055_CHIP_ID_REG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  id = Wire.read();
  return true;
}

void printFound(const char *label, uint8_t *found, int n) {
  Serial.printf("  %s: ", label);
  if (n == 0) {
    Serial.println("nothing answers");
    return;
  }
  for (int i = 0; i < n; i++) {
    Serial.printf("0x%02X", found[i]);
    uint8_t id;
    if (readChipId(found[i], id)) {
      Serial.printf(id == BNO055_CHIP_ID ? " (BNO055)" : " (chip ID 0x%02X, not a BNO055)", id);
    }
    if (i < n - 1) Serial.print(", ");
  }
  Serial.println();
}

bool contains(uint8_t *found, int n, uint8_t addr) {
  for (int i = 0; i < n; i++) if (found[i] == addr) return true;
  return false;
}

void checkPair(const PinPair &p) {
  Serial.printf("\n[%s]\n", p.name);
  bool idleHigh = linesIdleHigh(p);

  uint8_t normal[MAX_FOUND], swapped[MAX_FOUND];
  int nNormal = scan(p.sda, p.scl, normal);
  printFound("SDA/SCL as expected", normal, nNormal);
  Wire.end();
  int nSwapped = scan(p.scl, p.sda, swapped);
  printFound("SDA/SCL swapped    ", swapped, nSwapped);
  Wire.end();

  Serial.print("  => ");
  if (contains(normal, nNormal, EXPECTED_ADDR)) {
    Serial.println("OK: BNO055 at 0x28, as the firmware expects.");
  } else if (contains(normal, nNormal, 0x29)) {
    Serial.println("BNO055 answers at 0x29: its ADR pin is HIGH. Connect ADR to GND (or leave it unconnected).");
  } else if (nSwapped > 0) {
    Serial.println("Answers only with SDA and SCL swapped: swap the two data wires on this pair.");
  } else if (!idleHigh) {
    Serial.println("Nothing answers and a line is LOW at rest: IMU not powered (check VIN and GND) or a line shorted to GND.");
  } else if (nNormal > 0) {
    Serial.println("Something answers, but not at 0x28/0x29: check this is the right board.");
  } else {
    Serial.println("Nothing answers: check SDA/SCL go to these exact pins, and VIN/GND on the IMU.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // BNO055 needs ~650 ms after power-up
  Serial.println("\n=== I2C check, both IMU buses ===");
}

void loop() {
  for (const PinPair &p : pairs) checkPair(p);
  Serial.println("\n--- checking again in 4 s ---");
  delay(4000);
}
