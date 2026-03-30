#include <Arduino.h>
#include <unity.h>

#include <Adafruit_BMP085.h>
#include <RotaryEncoder.h>

void setUp(void) {}
void tearDown(void) {}

void test_dependencies_are_available(void) {
  RotaryEncoder encoder(18, 19, RotaryEncoder::LatchMode::TWO03);
  Adafruit_BMP085 bmp;

  (void)encoder;
  (void)bmp;
  TEST_ASSERT_TRUE(true);
}

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_dependencies_are_available);
  UNITY_END();
}

void loop() {}
