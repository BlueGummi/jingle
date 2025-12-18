#include <Wire.h>
#include <FastLED.h>
#include <L3G.h> 

#define LED_PIN     6
#define NUM_LEDS    180
#define BRIGHTNESS 120
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
uint16_t offset = 0;

L3G gyro;

const int16_t IDLE_THRESHOLD = 600;
const unsigned long IDLE_DELAY_MS = 2000;

unsigned long lastMovementTime = 0;
bool isIdle = false;

void setup() {
  Serial.begin(115200);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  Wire.begin();

  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro!");
    while (1) { delay(10); }
  }
  gyro.enableDefault();
  Serial.println("L3G gyro initialized!");

  lastMovementTime = millis();
}

void loop() {
  gyro.read();
  int16_t gx = gyro.g.x;
  int16_t gy = gyro.g.y;
  int16_t gz = gyro.g.z;

  unsigned long now = millis();
  bool movementDetected = (abs(gx) >= IDLE_THRESHOLD || abs(gy) >= IDLE_THRESHOLD || abs(gz) >= IDLE_THRESHOLD);

  if (movementDetected) {
    lastMovementTime = now;
    isIdle = false;
  }

  if (!isIdle && now - lastMovementTime >= IDLE_DELAY_MS) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    isIdle = true;
  }

  if (!isIdle) {
    int speed = map(abs(gx), 0, 150, 1, 12);
    offset += speed;

    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ((i + offset) % 4 < 2) ? CRGB::Red : CRGB::Green;
    }

    uint8_t twinkleChance = map(abs(gy), 0, 150, 10, 100);
    if (random8() < twinkleChance) {
      leds[random16(NUM_LEDS)] = CRGB::White;
    }

    FastLED.show();
  }

  delay(60);
}
