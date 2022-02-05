# Arduino GT911 Library
Arduino GT911 touch screen driver

Basic usage
```cpp
#include <GT911.h>

GT911 touch = GT911();

void handleTouch(int8_t count, GTPoint *points) {
  for (int i = 0; i < count; i++) 
    Serial.printf("Point %d at %d, %d  size: %d  track ID: %d\n", i, points[i].x, points[i].y, points[i].size, points[i].trackId);
}

void touchStart() {
  touch.begin(INT_PIN, RST_PIN, GT911_I2C_ADDR_BA);
  Wire.beginTransmission(touch.getI2CAddress());
  touch.readInfo(); // you can get product id and other information with:
  Serial.println(getInfo().productId);

  touch.readConfig();
  // you can change configs with:
  touch.getConfig().xResolution = 128;
  // and save it with:
  touch.writeConfig();
}

void begin() {
  Wire.setClock(400000);
  Wire.begin();
  delay(300);
  touch.setHandler(handleTouch);
  touchStart();
}

void loop() {
  touch.loop();
}
```

Prototype of arduino-based library for GT911 touchscreen driver chip (tested on Teensy 4.1, 320*480 resolution).

Based on https://github.com/nik-sharky/arduino-goodix