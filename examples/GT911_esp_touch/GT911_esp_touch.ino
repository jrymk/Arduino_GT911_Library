#include <Wire.h>
#include "Goodix.h"

// D5 D6 pins are defined on boards like WeMOS, You can redefine it to numeric values
#define INT_PIN 34
#define RST_PIN -1
#define LILYGO_LILYPI true

Goodix touch = Goodix();

void handleTouch(int8_t contacts, GTPoint* points) {
  Serial.printf("Contacts: %d\n", contacts);
  for (uint8_t i = 0; i < contacts; i++) {
    Serial.printf("C%d: %d %d \n", i, points[i].x, points[i].y);
    yield();
  }
}

void touchStart() {
  unsigned short configInfo;
  
  touch.begin(INT_PIN, RST_PIN, GOODIX_I2C_ADDR_BA);
  Serial.print("Check ACK on addr request on 0x");
  Serial.print(touch.i2cAddr, HEX);
  Wire.beginTransmission(touch.i2cAddr);
  if (!Wire.endTransmission()) {
    Serial.println(": SUCCESS");
  } else {
    Serial.print(": ERROR!");
  }
  configInfo = touch.configCheck(LILYGO_LILYPI);
  if (!configInfo)
  {
    Serial.println("Config is OK");
  }
  else
  {
    Serial.print("Config ERROR: ");
    Serial.println(configInfo);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nGoodix GT911x touch driver");

  Wire.setClock(400000);
  Wire.begin();
  delay(300);

  touch.setHandler(handleTouch);
  touchStart();
  //touch.configUpdate();
}

void loop() {
  touch.loop();
  delay(10);
}
