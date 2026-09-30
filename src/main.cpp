// Waveshare ESP32-S3-ETH-8DI-8RO - grid relay chase
//
// Behaviour:
//   loop() plays every pattern in PLAYLIST (bottom of this file) in order,
//   then starts over, forever. Reorder / remove entries there to change the
//   show. Patterns:
//     runFullOnOff                 - all ON (staggered for inrush), hold, all OFF, hold
//     runSingleChase               - relays 1-8 one at a time
//     runCascade(ROWS/COLS)        - groups ON one after another, then OFF in the same order
//     runFillDrain(ROWS/COLS)      - groups ON one after another, then OFF in reverse order
//     runCheckerboard              - two alternating checkerboard halves of the grid
//     runRowBounce                 - one row at a time, top -> bottom -> top
//     runRandom                    - random relay mix, random hold times
//     runMiddleRowOnOuterAlternate - top/bottom rows swap, middle row drops out
//                                    for the last MIDDLE_OFF_PERCENT of each step
//
//   Rows/columns come from the ROWS / COLS grid below, which matches the
//   physical socket layout.
//
// Hardware note:
//   The 8 relays are NOT wired to ESP32 GPIOs directly. They are driven by
//   a TCA9554PWR I2C GPIO expander (EXIO1..EXIO8 = relay 1..8), which sits
//   on the same I2C bus as the onboard RTC:
//     SDA = GPIO42 (RTC_SDA)
//     SCL = GPIO41 (RTC_SCL)
//     TCA9554 I2C address = 0x20   (confirmed in Waveshare's own FAQ)
//   Per Waveshare's own demo code, driving an EXIO pin HIGH turns that
//   relay ON. If your board turns out to behave the other way round,
//   flip RELAY_ACTIVE_HIGH below.
//
// Source for the pin/interface map: official Waveshare wiki page for
// "ESP32-S3-ETH-8DI-8RO" (waveshare.com/wiki/ESP32-S3-ETH-8DI-8RO).

#include <Arduino.h>
#include <Wire.h>

// Set to true if your board turns out to behave the other way round (i.e.
// driving an EXIO pin LOW turns the relay ON instead of HIGH).
static constexpr bool RELAY_ACTIVE_HIGH = true; // measured on real hardware: EXIO LOW = relay energised (opposite of Waveshare's demo comment)

// ---------- Board pin map ----------
static constexpr uint8_t I2C_SDA_PIN   = 42;
static constexpr uint8_t I2C_SCL_PIN   = 41;
static constexpr uint8_t TCA9554_ADDR  = 0x20;
static constexpr uint8_t RGB_LED_PIN   = 38; // onboard WS2812, used here purely as a "is it alive" indicator

// TCA9554 registers
static constexpr uint8_t REG_INPUT     = 0x00;
static constexpr uint8_t REG_OUTPUT    = 0x01;
static constexpr uint8_t REG_POLARITY  = 0x02;
static constexpr uint8_t REG_CONFIG    = 0x03; // bit=0 -> that pin is an output

// ---------- Chase configuration ----------
static constexpr uint8_t  NUM_RELAYS       = 8;
static constexpr uint32_t STEP_INTERVAL_MS = 1000; // gap between each group switching
static constexpr uint32_t PHASE_PAUSE_MS   = 10; // pause between "all on" and "start turning off", and before restarting

// 3x3 grid in reading order, relay indices 0..7 (1-based relay = index+1),
// bottom-right cell (would be relay 9) missing:
//   0 1 2
//   3 4 5
//   6 7 .
struct RelayGroup {
  const uint8_t *indices;
  uint8_t count;
};

// Count is taken from the array itself, so rows/columns can be rewired freely.
#define GROUP(arr) {arr, sizeof(arr) / sizeof(arr[0])}

// Rows follow the physical socket layout (re-plugged; middle row has only 2):
//   top:    relays 1, 2, 3
//   middle: relays 7, 5
//   bottom: relays 4, 6, 8
static const uint8_t ROW0[] = {0, 1, 2};
static const uint8_t ROW1[] = {6, 4}; //9//// only 2 relays
static const uint8_t ROW2[] = {3, 5, 7};
static const RelayGroup ROWS[] = {GROUP(ROW0), GROUP(ROW1), GROUP(ROW2)};
static constexpr uint8_t NUM_ROWS = sizeof(ROWS) / sizeof(ROWS[0]);

static const uint8_t COL0[] = {0, 3, 6};
static const uint8_t COL1[] = {1, 4, 7};
static const uint8_t COL2[] = {2, 5}; //9// bottom-right missing
static const RelayGroup COLS[] = {GROUP(COL0), GROUP(COL1), GROUP(COL2)};
static constexpr uint8_t NUM_COLS = sizeof(COLS) / sizeof(COLS[0]);

// How many times top/bottom swap in runMiddleRowOnOuterAlternate().
static constexpr uint8_t SIDE_SWAPS = 6;
// Middle row switches off for this last part of each step, before the swap.
static constexpr uint32_t MIDDLE_OFF_PERCENT = 20;

// runFullOnOff() timing. Relays switch on INRUSH_STAGGER_MS apart so the fans
// don't all start at the same instant (0 = all in one I2C write).
static constexpr uint32_t FULL_ON_MS        = 10000;
static constexpr uint32_t FULL_OFF_MS       = 5000;
static constexpr uint32_t INRUSH_STAGGER_MS = 50;

// Repeat counts for the playlist patterns.
static constexpr uint8_t CHECKER_SWAPS  = 8;
static constexpr uint8_t BOUNCE_PASSES  = 3;
static constexpr uint8_t RANDOM_STEPS   = 10;
static constexpr uint32_t RANDOM_MIN_MS = 500;
static constexpr uint32_t RANDOM_MAX_MS = 3000;

static uint8_t relayBits = 0x00; // current shadow copy of the TCA9554 output register

// Write one byte to a TCA9554 register. Returns true on success.
static bool tcaWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("TCA9554 I2C write failed (reg 0x%02X, err %u)\n", reg, err);
    return false;
  }
  return true;
}

// index: 0..7 -> relay 1..8. Only updates the shadow byte - call writeRelays()
// to actually push it out over I2C (lets a whole group change atomically).
static void setRelayBit(uint8_t index, bool on) {
  bool physicalHigh = RELAY_ACTIVE_HIGH ? on : !on;
  if (physicalHigh) {
    relayBits |= (1u << index);
  } else {
    relayBits &= ~(1u << index);
  }
}

static void writeRelays() {
  tcaWrite(REG_OUTPUT, relayBits);
}

static void allRelaysOff() {
  relayBits = RELAY_ACTIVE_HIGH ? 0x00 : 0xFF;
  writeRelays();
}

// Set every relay listed in `group` to `on` in one I2C write.
static void setGroup(const RelayGroup &group, bool on) {
  for (uint8_t k = 0; k < group.count; k++) {
    setRelayBit(group.indices[k], on);
  }
  writeRelays();
}

// Set all 8 relays from one logical mask (bit i = relay i+1 ON), one I2C write.
static void setMask(uint8_t mask) {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    setRelayBit(i, mask & (1u << i));
  }
  writeRelays();
}

// Logical mask (bit i = relay i+1) of every relay in `group`.
static uint8_t groupMask(const RelayGroup &group) {
  uint8_t mask = 0;
  for (uint8_t k = 0; k < group.count; k++) {
    mask |= 1u << group.indices[k];
  }
  return mask;
}

// BLUE LED blink + wait `ms` in total. Safe for ms < 100.
static void pulseAndWait(uint32_t ms) {
  rgbLedWrite(RGB_LED_PIN, 0, 0, 32); // BLUE pulse = loop() is alive and iterating
  delay(100);
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
  if (ms > 100) delay(ms - 100);
}

// One blink + one STEP_INTERVAL_MS wait, shared by the patterns below.
static void heartbeatAndWait() {
  pulseAndWait(STEP_INTERVAL_MS);
}

// Switch each group in `groups` ON one after another (STEP_INTERVAL_MS
// apart), then (after a short pause) OFF in the same order.
static void runCascade(const RelayGroup *groups, uint8_t numGroups, const char *label) {
  for (uint8_t g = 0; g < numGroups; g++) {
    setGroup(groups[g], true);
    Serial.printf("%s %u ON\n", label, g + 1);
    heartbeatAndWait();
  }

  delay(PHASE_PAUSE_MS);

  for (uint8_t g = 0; g < numGroups; g++) {
    setGroup(groups[g], false);
    Serial.printf("%s %u OFF\n", label, g + 1);
    heartbeatAndWait();
  }

  delay(PHASE_PAUSE_MS);
}

// Like runCascade(), but switches the groups OFF in reverse order, so the
// grid fills up and then drains back down.
static void runFillDrain(const RelayGroup *groups, uint8_t numGroups, const char *label) {
  for (uint8_t g = 0; g < numGroups; g++) {
    setGroup(groups[g], true);
    Serial.printf("%s %u ON\n", label, g + 1);
    heartbeatAndWait();
  }

  delay(PHASE_PAUSE_MS);

  for (int8_t g = numGroups - 1; g >= 0; g--) {
    setGroup(groups[g], false);
    Serial.printf("%s %u OFF\n", label, g + 1);
    heartbeatAndWait();
  }

  delay(PHASE_PAUSE_MS);
}

// Every relay ON, hold FULL_ON_MS, every relay OFF, hold FULL_OFF_MS.
// Switch-on is staggered by INRUSH_STAGGER_MS to spread the fans' start-up current.
static void runFullOnOff() {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    setRelayBit(i, true);
    if (INRUSH_STAGGER_MS) {
      writeRelays();
      delay(INRUSH_STAGGER_MS);
    }
  }
  writeRelays();
  Serial.println("ALL ON");
  pulseAndWait(FULL_ON_MS);

  allRelaysOff();
  Serial.println("ALL OFF");
  pulseAndWait(FULL_OFF_MS);
}

// Two checkerboard halves of the grid take turns, CHECKER_SWAPS times.
// Positions come from ROWS, so this follows the physical socket layout.
static void runCheckerboard() {
  uint8_t even = 0;
  for (uint8_t r = 0; r < NUM_ROWS; r++) {
    for (uint8_t k = 0; k < ROWS[r].count; k++) {
      if ((r + k) % 2 == 0) even |= 1u << ROWS[r].indices[k];
    }
  }

  for (uint8_t s = 0; s < CHECKER_SWAPS; s++) {
    setMask(s % 2 ? (uint8_t)~even : even);
    Serial.printf("Checker %s\n", s % 2 ? "B" : "A");
    heartbeatAndWait();
  }

  allRelaysOff();
  delay(PHASE_PAUSE_MS);
}

// One row ON at a time: top -> middle -> bottom -> middle -> ..., BOUNCE_PASSES
// times. Each step is a single I2C write, so rows never overlap.
static void runRowBounce() {
  static const uint8_t order[] = {0, 1, 2, 1};
  static constexpr uint8_t steps = sizeof(order) / sizeof(order[0]);

  for (uint8_t p = 0; p < BOUNCE_PASSES * steps; p++) {
    uint8_t row = order[p % steps];
    setMask(groupMask(ROWS[row]));
    Serial.printf("Row %u only\n", row + 1);
    heartbeatAndWait();
  }

  allRelaysOff();
  delay(PHASE_PAUSE_MS);
}

// RANDOM_STEPS random relay mixes, each held for a random time.
static void runRandom() {
  for (uint8_t s = 0; s < RANDOM_STEPS; s++) {
    uint8_t mask = esp_random() & 0xFF;
    setMask(mask);
    Serial.printf("Random 0x%02X\n", mask);
    pulseAndWait(random(RANDOM_MIN_MS, RANDOM_MAX_MS));
  }

  allRelaysOff();
  delay(PHASE_PAUSE_MS);
}

// Simple 1->8 single-relay chase: each relay switches on for STEP_INTERVAL_MS,
// then off, before the next relay in line switches on.

static void runSingleChase() {
  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    setRelayBit(i, true);
    writeRelays();
    Serial.printf("Relay %u ON\n", i + 1);
    heartbeatAndWait();
    setRelayBit(i, false);
    writeRelays();
    Serial.printf("Relay %u OFF\n", i + 1);
  }
}

// Top row (1,2,3) and bottom row (4,6,8) take turns, SIDE_SWAPS times.
// The middle row (7,5) switches ON together with each new outer row, and
// OFF again for the last MIDDLE_OFF_PERCENT of every step - so it drops out
// just before the outer rows swap, then comes back in with the next one.
// Everything is switched off again at the end.
static void runMiddleRowOnOuterAlternate() {
  const RelayGroup &top = ROWS[0];
  const RelayGroup &middle = ROWS[1];
  const RelayGroup &bottom = ROWS[2];

  const uint32_t middleOffMs = STEP_INTERVAL_MS * MIDDLE_OFF_PERCENT / 100;
  const uint32_t middleOnMs  = STEP_INTERVAL_MS - middleOffMs;

  for (uint8_t s = 0; s < SIDE_SWAPS; s++) {
    bool topOn = (s % 2 == 0);
    for (uint8_t k = 0; k < top.count; k++)    setRelayBit(top.indices[k], topOn);
    for (uint8_t k = 0; k < bottom.count; k++) setRelayBit(bottom.indices[k], !topOn);
    for (uint8_t k = 0; k < middle.count; k++) setRelayBit(middle.indices[k], true);
    writeRelays(); // outer rows swap and middle comes on in the same I2C write
    Serial.println(topOn ? "Top + Middle ON, Bottom OFF" : "Bottom + Middle ON, Top OFF");

    pulseAndWait(middleOnMs);

    setGroup(middle, false);
    Serial.println("Middle OFF");
    delay(middleOffMs);
  }

  allRelaysOff();
  Serial.println("All rows OFF");
  delay(PHASE_PAUSE_MS);
}

// Quick bus scan so we can see in the Serial Monitor whether the TCA9554
// (expected at 0x20) is actually answering on these pins.
static void scanI2CBus() {
  Serial.println("Scanning I2C bus...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  no I2C devices found at all - check wiring/pins/power");
  } else {
    Serial.printf("  %u device(s) found\n", found);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Heartbeat / diagnostic: RED = "reached setup(), about to scan I2C".
  // If it never advances past red, execution is hanging inside the I2C
  // scan below (e.g. TCA9554 not responding / bus stuck) - independent of
  // whether Serial is actually visible on your monitor or not.
  rgbLedWrite(RGB_LED_PIN, 32, 0, 0);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  scanI2CBus();

  // YELLOW = survived the I2C scan.
  rgbLedWrite(RGB_LED_PIN, 32, 32, 0);

  // Write "all off" BEFORE switching the EXIO pins to outputs. The TCA9554
  // powers up with its output register defaulted to 0xFF; if we set the
  // pins to outputs first, every relay would briefly energise before our
  // "off" write lands a moment later.
  allRelaysOff();
  tcaWrite(REG_CONFIG, 0x00); // all 8 EXIO pins -> outputs

  Serial.println("ESP32-S3-ETH-8DI-8RO relay chase - starting");

  // GREEN = setup() fully completed, entering loop().
  rgbLedWrite(RGB_LED_PIN, 0, 32, 0);
  delay(500);
}

// Patterns played in order by loop(), then repeated. Comment out / reorder
// entries to change the show.
using Pattern = void (*)();
static const Pattern PLAYLIST[] = {
 // runFullOnOff,
 // runSingleChase,
 // [] { runCascade(ROWS, NUM_ROWS, "Row"); },
 // [] { runCascade(COLS, NUM_COLS, "Col"); },
 // [] { runFillDrain(ROWS, NUM_ROWS, "Row"); },
 // [] { runFillDrain(COLS, NUM_COLS, "Col"); },
 // runCheckerboard,
 // runRowBounce,
 // runRandom,
  runMiddleRowOnOuterAlternate,
};

void loop() {
  for (Pattern pattern : PLAYLIST) {
    pattern();
  }
}
