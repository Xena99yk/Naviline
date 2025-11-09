
#include <SPI.h>
#include <LoRa.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// ================== Device Identity ==================
#define DEVICE_ID 3      // <-- Set uniquely to 1, 2, or 3 on each board

// ================== LoRa Pin Definitions (XIAO ESP32-C3) ==================
#define LORA_SS    D7   // GPIO7 (NSS)
#define LORA_RST   D3   // GPIO3 (RST)
#define LORA_DIO0  D1   // GPIO1 (DIO0)

// SPI pins explicitly (XIAO ESP32-C3)
#define SPI_SCK    D4   // GPIO4
#define SPI_MISO   D5   // GPIO5
#define SPI_MOSI   D6   // GPIO6

// ================== Simple LED + Button ==================
#define SIMPLE_LED_PIN  D9       // LED you want to light on command
#define BUTTON_PIN      D8       // Button to GND, uses INPUT_PULLUP

// ================== Optional NeoPixel for distance feedback ==================
#define LED_PIN   D2
#define NUM_LEDS  6
Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ================== LoRa Radio Settings ==================
#define LORA_FREQ          433E6
#define LORA_SF            7
#define LORA_BW            125E3
#define LORA_CR            5
#define LORA_SYNC_WORD     0x33

// ================== Beacon / Protocol ==================
const unsigned long BASE_INTERVAL_MS = 800; // base beacon interval
// Add an ID-based offset to de-sync transmit times (reduce collisions)
const unsigned long ID_OFFSET_MS = (DEVICE_ID % 4) * 120;

// ================== RSSI / Distance ==================
const int RSSI_WINDOW_SIZE = 10;
struct Peer {
  int id;
  int rssiBuf[RSSI_WINDOW_SIZE];
  int idx = 0;
  bool filled = false;
  float lastDistanceM = NAN;
  unsigned long lastHeardMs = 0;
};
Peer peers[2]; // track the two other devices

// Tunables for RSSI->distance (calibrate these!)
float RSSI_AT_1M = -55.0;  // dBm at 1 meter
float PATH_LOSS_N = 2.7;   // path-loss exponent

// ================== LED Timer ==================
unsigned long simpleLedOffAt = 0; // millis when to turn off D9

// ================== Beacon Seq ==================
unsigned long lastBeaconMs = 0;
uint32_t beaconSeq = 0;

// ================== Helpers ==================
void setNeoPixel(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(r, g, b));
  leds.show();
}

float estimateDistance(float avgRssi) {
  // d = 10^((RSSI_1m - RSSI) / (10 * n))
  float exponent = (RSSI_AT_1M - avgRssi) / (10.0f * PATH_LOSS_N);
  return powf(10.0f, exponent);
}

float avgRSSI(const Peer& p) {
  int count = p.filled ? RSSI_WINDOW_SIZE : p.idx;
  if (count <= 0) return NAN;
  long sum = 0;
  for (int i = 0; i < count; i++) sum += p.rssiBuf[i];
  return (float)sum / (float)count;
}

Peer* getPeerById(int id) {
  for (int i = 0; i < 2; i++) if (peers[i].id == id) return &peers[i];
  return nullptr;
}

void pushRSSI(Peer& p, int rssi) {
  p.rssiBuf[p.idx] = rssi;
  p.idx = (p.idx + 1) % RSSI_WINDOW_SIZE;
  if (p.idx == 0) p.filled = true;
  float r = avgRSSI(p);
  if (!isnan(r)) p.lastDistanceM = estimateDistance(r);
  p.lastHeardMs = millis();
}

void tryInitPeers() {
  // Fill peer IDs as the two IDs != DEVICE_ID (assuming IDs 1..3 used)
  int ids[3] = {1, 2, 3};
  int j = 0;
  for (int i = 0; i < 3; i++) {
    if (ids[i] != DEVICE_ID) {
      peers[j].id = ids[i];
      peers[j].idx = 0;
      peers[j].filled = false;
      peers[j].lastDistanceM = NAN;
      peers[j].lastHeardMs = 0;
      j++;
      if (j >= 2) break;
    }
  }
}

int nearestPeerId() {
  // returns the peer ID with the smallest known distance; -1 if none
  float best = 1e9;
  int bestId = -1;
  for (int i = 0; i < 2; i++) {
    float d = peers[i].lastDistanceM;
    if (!isnan(d) && d < best) {
      best = d;
      bestId = peers[i].id;
    }
  }
  return bestId;
}

void sendBeacon() {
  LoRa.beginPacket();
  // Format: B,<id>,<seq>
  LoRa.print("B,");
  LoRa.print(DEVICE_ID);
  LoRa.print(",");
  LoRa.print(beaconSeq++);
  LoRa.endPacket();
  delay(5); // small gap helps under RF noise
}

void sendLedCommand(int targetId, uint16_t onMs) {
  // Format: L,<from>,<to>,<ms>
  LoRa.beginPacket();
  LoRa.print("L,");
  LoRa.print(DEVICE_ID);
  LoRa.print(",");
  LoRa.print(targetId);
  LoRa.print(",");
  LoRa.print(onMs);
  LoRa.endPacket();
  delay(5);
}

// Basic (very light) debounce
bool buttonPressed() {
  static uint8_t stable = HIGH;
  static unsigned long lastChange = 0;
  uint8_t raw = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  if (raw != stable && (now - lastChange) > 20) {
    stable = raw;
    lastChange = now;
    if (stable == LOW) return true; // active LOW
  }
  return false;
}

void showNearestOnNeoPixel() {
  // Color by nearest distance
  int id = nearestPeerId();
  if (id < 0) { setNeoPixel(0, 0, 0); return; }

  Peer* p = getPeerById(id);
  float d = p ? p->lastDistanceM : NAN;
  if (isnan(d)) { setNeoPixel(0, 0, 0); return; }

  if (d <= 1.0f) setNeoPixel(0, 255, 0);         // <= 1 m : green
  else if (d <= 2.0f) setNeoPixel(0, 0, 255);     // <= 2 m : blue
  else setNeoPixel(255, 0, 0);                    // > 2 m : red
}

// ================== Setup / Loop ==================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nThree-Node LoRa Mesh-Lite (nearest LED) starting...");

  pinMode(SIMPLE_LED_PIN, OUTPUT);
  digitalWrite(SIMPLE_LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // button to GND

  leds.begin();
  leds.show(); // off

  tryInitPeers();

  // Init LoRa
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  while (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed. Retrying...");
    delay(1000);
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC_WORD);

  Serial.println("LoRa Initialized.");
}

void loop() {
  unsigned long now = millis();

  // 1) Periodic beacon (staggered by ID)
  if (now - lastBeaconMs >= (BASE_INTERVAL_MS + ID_OFFSET_MS)) {
    lastBeaconMs = now;
    sendBeacon();
  }

  // 2) Handle received packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String s;
    while (LoRa.available()) s += (char)LoRa.read();
    int rssi = LoRa.packetRssi();

    // Parse: either "B,<id>,<seq>" or "L,<from>,<to>,<ms>"
    if (s.startsWith("B,")) {
      // Beacon
      int c1 = s.indexOf(',', 2);  // after "B,"
      if (c1 > 2) {
        int senderId = s.substring(2, c1).toInt();
        if (senderId != DEVICE_ID) {
          Peer* p = getPeerById(senderId);
          if (p) {
            pushRSSI(*p, rssi);
            float r = avgRSSI(*p);
            float d = isnan(r) ? NAN : estimateDistance(r);
            Serial.print("Beacon from ID ");
            Serial.print(senderId);
            Serial.print(" | RSSI avg: ");
            Serial.print(r, 1);
            Serial.print(" dBm | d≈ ");
            if (isnan(d)) Serial.print("?");
            else Serial.print(d * 100.0f, 0);
            Serial.println(" cm");
          }
        }
      }
    } else if (s.startsWith("L,")) {
      // LED command: "L,<from>,<to>,<ms>"
      int c1 = s.indexOf(',', 2);
      int c2 = (c1 > 0) ? s.indexOf(',', c1 + 1) : -1;
      if (c1 > 0 && c2 > c1) {
        int fromId = s.substring(2, c1).toInt();
        int toId   = s.substring(c1 + 1, c2).toInt();
        int ms     = s.substring(c2 + 1).toInt(); // read to end (no 4th comma)

        if (toId == DEVICE_ID) {
          digitalWrite(SIMPLE_LED_PIN, HIGH);
          simpleLedOffAt = millis() + (unsigned long)ms;
          Serial.print("LED command from ID ");
          Serial.print(fromId);
          Serial.print(" -> me: ON for ");
          Serial.print(ms);
          Serial.println(" ms");
        }
      } else {
        Serial.print("Bad L packet: ");
        Serial.println(s);
      }
    } else {
      // Other data (ignore or print)
      Serial.print("RX: ");
      Serial.print(s);
      Serial.print(" | RSSI ");
      Serial.println(rssi);
    }
  }

  // 3) Button -> light self + nearest peer
  if (buttonPressed()) {
    // Light our own LED for 1500 ms
    digitalWrite(SIMPLE_LED_PIN, HIGH);
    simpleLedOffAt = now + 1500;

    // Determine nearest peer and command it
    int target = nearestPeerId();
    if (target > 0) {
      sendLedCommand(target, 1500); // ask nearest to light for 1.5s
      Serial.print("Button pressed. Nearest = ID ");
      Serial.println(target);
    } else {
      Serial.println("Button pressed, but no peer distance yet.");
    }
  }

  // 4) Auto-off for the simple LED
  if (simpleLedOffAt && now >= simpleLedOffAt) {
    digitalWrite(SIMPLE_LED_PIN, LOW);
    simpleLedOffAt = 0;
  }

  // 5) NeoPixel feedback for nearest peer
  showNearestOnNeoPixel();
}