#ifndef _GOODIX_H_
#define _GOODIX_H_

#include <Arduino.h>
#include "GoodixStructs.h"
#include "GoodixFw.h"

#define GOODIX_OK   0

// 0x28/0x29 (0x14 7bit)
#define GOODIX_I2C_ADDR_28  0x14
// 0xBA/0xBB (0x5D 7bit)
#define GOODIX_I2C_ADDR_BA  0x5D

#define GOODIX_CONTACT_SIZE   8
#define GOODIX_MAX_CONTACTS   5

/* Register defines */
#define GT_REG_CFG  0x8047
#define GT_REG_DATA 0x8140

// Write only registers
#define GOODIX_REG_COMMAND        0x8040

// Read/write registers
// The version number of the configuration file
#define GOODIX_REG_CONFIG_DATA  0x8047

#define GOODIX_REG_CONFIG_MIDDLE	0x80A2
#define GOODIX_REG_CONFIG_END		0x80FE

// ReadOnly registers (device and coordinates info)
// Product ID (LSB 4 bytes, GT9110: 0x06 0x00 0x00 0x09)
#define GOODIX_REG_ID           0x8140

#define GOODIX_READ_COORD_ADDR  0x814E


class Goodix {
  public:
    uint8_t i2cAddr;
    struct GTConfig config;
    struct GTInfo info;
    struct GTPoint points[GOODIX_MAX_CONTACTS]; //processed points

    Goodix();

    void setHandler(void (*handler)(int8_t, GTPoint*));

    bool begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr=GOODIX_I2C_ADDR_BA);
    bool reset();
    uint8_t test();
    void loop();

    bool write(uint16_t reg, uint8_t value);
    bool writeBytes(uint16_t reg, uint8_t *data, int nbytes);
    bool readBytes(uint16_t reg, uint8_t *data, int nbytes);

    uint8_t calcChecksum(uint8_t* buf, uint8_t len);
    uint8_t readChecksum();
	
    void fwResolution(uint16_t maxX, uint16_t maxY);
    void configUpdate();
    uint8_t configCheck(bool isLilyPi);
    
    GTConfig* readConfig();
    GTInfo* readInfo();

    uint8_t productID(char *buf);

    int16_t readInput(uint8_t *data);

  //--- Private routines ---
  private:
    uint8_t intPin, rstPin;
    void (*touchHandler)(int8_t, GTPoint*);

    void debugPin(uint8_t level);
    void armIRQ();
    void onIRQ();

    //--- utils ---
    void usSleep(uint16_t microseconds);
    void msSleep(uint16_t milliseconds);

    void pinIn(uint8_t pin);
    void pinOut(uint8_t pin);
    void pinSet(uint8_t pin, uint8_t level);

    // Used with pulled-up lines, set pin mode to out, write LOW
    void pinHold(uint8_t pin);

    // Check pin level
    bool pinCheck(uint8_t pin, uint8_t level);

    void i2cStart(uint16_t reg);
    void i2cRestart();
    uint8_t i2cStop();
};

#endif
