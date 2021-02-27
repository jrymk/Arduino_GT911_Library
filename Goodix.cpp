#include "Goodix.h"
#include "Wire.h"

// Interrupt handling
volatile bool goodixIRQ = false;

#if defined(ESP8266)
void ICACHE_RAM_ATTR _goodix_irq_handler() {
  noInterrupts();
  goodixIRQ = true;
  interrupts();
}
#elif defined(ESP32)
void IRAM_ATTR _goodix_irq_handler() {
  noInterrupts();
  goodixIRQ = true;
  interrupts();
}
#else
void _goodix_irq_handler() {
  noInterrupts();
  goodixIRQ = true;
  interrupts();
}
#endif


// Implementation
Goodix::Goodix() {

}

void Goodix::setHandler(void (*handler)(int8_t, GTPoint*)) {
  touchHandler = handler;
}

bool Goodix::begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr) {
  intPin = interruptPin;
  rstPin = resetPin;
  i2cAddr = addr;

  // Take chip some time to start
  msSleep(300);
  bool result = reset();
  msSleep(200);

  return result;
}


bool Goodix::reset() {
  msSleep(1);

  if (rstPin >= 0)
  {
  	pinMode(rstPin, OUTPUT);
  	digitalWrite(rstPin, LOW);
  	/* T2: > 10ms */
  	msSleep(11);
  	/* T3: > 100us */
  	usSleep(110);
  	pinMode(rstPin, INPUT);
  	/* T4: > 5ms */
  	msSleep(6);
  	/* T5: 50ms */
  	msSleep(51);
  }
  pinMode(intPin, INPUT); // INT pin has no pullups so simple set to floating input

  attachInterrupt(intPin, _goodix_irq_handler, RISING);

  return true;
}

/**
   Read goodix touchscreen version
   set 4 chars + zero productID to target
*/
uint8_t Goodix::productID(char *target) {
  uint8_t success;
  uint8_t buf[4];
  

  success = readBytes(GOODIX_REG_ID, buf, 4);
  readBytes(GT_REG_DATA, (uint8_t *) &info, sizeof(info));
  if (!success) {
    return success;
  }

  memcpy(target, buf, 4);
  target[4] = 0;
  return 0;
}

/**
   goodix_i2c_test - I2C test function to check if the device answers.
*/
uint8_t Goodix::test() {
  uint8_t testByte;
  return readBytes(GOODIX_REG_CONFIG_DATA,  &testByte, 1);
}

uint8_t Goodix::calcChecksum(uint8_t* buf, uint8_t len) {
  uint8_t ccsum = 0;
  for (uint8_t i = 0; i < len; i++) {
    ccsum += buf[i];
  }

  ccsum = (~ccsum) + 1;
  return ccsum;
}

uint8_t Goodix::readChecksum() {
	
	uint8_t len1 = GOODIX_REG_CONFIG_MIDDLE - GOODIX_REG_CONFIG_DATA +1;
	uint8_t len2 = GOODIX_REG_CONFIG_END - GOODIX_REG_CONFIG_MIDDLE;
	uint8_t buf1[len1];
	uint8_t buf2[len2];
	uint8_t buf[len1+len2];
	readBytes(GOODIX_REG_CONFIG_DATA, buf1, len1);
    readBytes(GOODIX_REG_CONFIG_MIDDLE+1, buf2, len2);
    memcpy(buf, buf1, sizeof(buf1));
	memcpy(buf+sizeof(buf1), buf2, sizeof(buf2));	
  	
	return calcChecksum(buf, len1+len2);
}

uint8_t Goodix::configCheck(bool configVersion) {
	
	uint8_t len1 = GOODIX_REG_CONFIG_MIDDLE - GOODIX_REG_CONFIG_DATA +1;
	uint8_t len2 = GOODIX_REG_CONFIG_END - GOODIX_REG_CONFIG_MIDDLE;
	uint8_t buf1[len1];
	uint8_t buf2[len2];
	uint8_t buf[len1+len2];
	uint8_t diff = 0;
	uint8_t calc_check_sum;
	uint8_t read_check_sum[1];
	char prodID[5];
	
	uint8_t config0[] = {
        0x5F, 0x40, 0x01, 0xE0, 0x01, 0x05, 0x35, 0x00, 0x01, 0x08,
        0x1E, 0x0F, 0x50, 0x32, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00,
        0x22, 0x22, 0x00, 0x18, 0x1B, 0x1E, 0x14, 0x87, 0x27, 0x0A,
        0x3C, 0x3E, 0x0C, 0x08, 0x00, 0x00, 0x00, 0x9B, 0x02, 0x1C,
        0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x09, 0x11, 0x00,
        0x00, 0x28, 0x6E, 0x94, 0xC5, 0x02, 0x00, 0x00, 0x00, 0x04,
        0xAB, 0x2C, 0x00, 0x8D, 0x36, 0x00, 0x75, 0x42, 0x00, 0x61,
        0x51, 0x00, 0x51, 0x63, 0x00, 0x51, 0x00, 0x00, 0x00, 0x00,
        0xF0, 0x4A, 0x3A, 0xFF, 0xFF, 0x27, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08, 0x06,
        0x04, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x24,
        0x22, 0x21, 0x20, 0x1F, 0x1E, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xBE, 0x01
    };

    write(GOODIX_REG_COMMAND, 0);
    
    productID(prodID);
    if (prodID[0] != '9')
    {
	    return (prodID[0]);
    }
    readBytes(GOODIX_REG_CONFIG_DATA, buf1, len1);
    readBytes(GOODIX_REG_CONFIG_MIDDLE+1, buf2, len2);
    memcpy(buf, buf1, sizeof(buf1));
	memcpy(buf+sizeof(buf1), buf2, sizeof(buf2));	
	calc_check_sum = calcChecksum(buf, len1+len2);
	readBytes(GOODIX_REG_CONFIG_END+1, read_check_sum, 1);
	
	if (configVersion)
	{
		
		for (uint8_t i=0; i<(len1+len2); i++) {
			if (config0[i] != buf[i])
				{
					diff++;
				}
		}
	}
	if (read_check_sum[0] != calc_check_sum)
		{
			diff++;
		}
	return (diff);
}

void Goodix::configUpdate() {
	
	uint8_t len1 = GOODIX_REG_CONFIG_MIDDLE - GOODIX_REG_CONFIG_DATA +1;
	uint8_t len2 = GOODIX_REG_CONFIG_END - GOODIX_REG_CONFIG_MIDDLE;
	uint8_t buf[2];
	char prodID[5];

	uint8_t config0[] = {
        0x5F, 0x40, 0x01, 0xE0, 0x01, 0x05, 0x35, 0x00, 0x01, 0x08,
        0x1E, 0x0F, 0x50, 0x32, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00,
        0x22, 0x22, 0x00, 0x18, 0x1B, 0x1E, 0x14, 0x87, 0x27, 0x0A,
        0x3C, 0x3E, 0x0C, 0x08, 0x00, 0x00, 0x00, 0x9B, 0x02, 0x1C,
        0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x09, 0x11, 0x00,
        0x00, 0x28, 0x6E, 0x94, 0xC5, 0x02, 0x00, 0x00, 0x00, 0x04,
        0xAB, 0x2C, 0x00, 0x8D, 0x36, 0x00, 0x75, 0x42, 0x00, 0x61,
        0x51, 0x00, 0x51, 0x63, 0x00, 0x51, 0x00, 0x00, 0x00, 0x00,
        0xF0, 0x4A, 0x3A, 0xFF, 0xFF, 0x27, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08, 0x06,
        0x04, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x24,
        0x22, 0x21, 0x20, 0x1F, 0x1E, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xBE, 0x01
    };
    
    buf[0] = calcChecksum(config0, len1+len2);
    buf[1] = 0x01;
    write(GOODIX_REG_COMMAND, 0);
    productID(prodID);
    if (prodID[0] != '9')
    {
	    return;
    }
    writeBytes(GOODIX_REG_CONFIG_DATA, config0, len1+len2);
    writeBytes(GOODIX_REG_CONFIG_END+1, buf, 2);
}

GTConfig* Goodix::readConfig() {
  readBytes(GT_REG_CFG, (uint8_t *) &config, sizeof(config));
  return &config;
}

GTInfo* Goodix::readInfo() {
  readBytes(GT_REG_DATA, (uint8_t *) &info, sizeof(info));
  return &info;
}

void Goodix::onIRQ() {
  int16_t contacts;
  uint8_t rawdata[GOODIX_MAX_CONTACTS * GOODIX_CONTACT_SIZE]; //points buffer

  contacts = readInput(rawdata);
  
  	if (contacts < 0)
  	{
	  	return;
  	}
  	
    if (contacts > 0) {
    
	finaldata[0].trackId = rawdata[1];	    
    finaldata[0].x = ((uint16_t)rawdata[3] << 8) + rawdata[2];
    finaldata[0].y = ((uint16_t)rawdata[5] << 8) + rawdata[4];
    finaldata[0].area = ((uint16_t)rawdata[7] << 8) + rawdata[6];
    
    finaldata[1].trackId = rawdata[9];
    finaldata[1].x = ((uint16_t)rawdata[11] << 8) + rawdata[10];
    finaldata[1].y = ((uint16_t)rawdata[13] << 8) + rawdata[12];
    finaldata[1].area = ((uint16_t)rawdata[15] << 8) + rawdata[14];

    finaldata[2].trackId = rawdata[17];
    finaldata[2].x = ((uint16_t)rawdata[19] << 8) + rawdata[18];
    finaldata[2].y = ((uint16_t)rawdata[21] << 8) + rawdata[20];
	finaldata[2].area = ((uint16_t)rawdata[23] << 8) + rawdata[22];
    
    finaldata[3].trackId = rawdata[25];
    finaldata[3].x = ((uint16_t)rawdata[27] << 8) + rawdata[26];
    finaldata[3].y = ((uint16_t)rawdata[29] << 8) + rawdata[28];
    finaldata[3].area = ((uint16_t)rawdata[31] << 8) + rawdata[30];

    finaldata[4].trackId = rawdata[33];
    finaldata[4].x = ((uint16_t)rawdata[35] << 8) + rawdata[34];
    finaldata[4].y = ((uint16_t)rawdata[37] << 8) + rawdata[36]; 
    finaldata[4].area = ((uint16_t)rawdata[39] << 8) + rawdata[38];
    
    touchHandler(contacts, finaldata);
	}
	write(GOODIX_READ_COORD_ADDR, 0);
}

void Goodix::loop() {
  noInterrupts();
  bool irq = goodixIRQ;
  goodixIRQ = false;
  interrupts();

  if (irq) {
    onIRQ();
  }
}

#define EAGAIN 100 			// Try again error
#define I2C_READ_ERROR 155 // I2C read error

int16_t Goodix::readInput(uint8_t *regState) {
  int touch_num;
  int error;

  error = readBytes(GOODIX_READ_COORD_ADDR, regState, GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS);
  touch_num = regState[0] & 0xF;

  if (!error) {
    return -I2C_READ_ERROR;
  }

  if (!(regState[0] & 0x80))
  {	  
    return -EAGAIN;
  }

  return touch_num;
}

//----- Utils -----
void Goodix::i2cStart(uint16_t reg) {
	Wire.beginTransmission(i2cAddr);
    Wire.write(reg >> 8);
    Wire.write(reg & 0xFF);
}

bool Goodix::write(uint16_t reg, uint8_t buf) {
  i2cStart(reg);
  Wire.write(buf);
  return (Wire.endTransmission() != 0);
}

bool Goodix::writeBytes(uint16_t reg, uint8_t *data, int nbytes)
{
	i2cStart(reg);
    for (int i = 0; i < nbytes; i++) {
        Wire.write(data[i]);
    }
    return (Wire.endTransmission() != 0);
}

bool Goodix::readBytes(uint16_t reg, uint8_t *data, int nbytes)
{
	i2cStart(reg);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddr, (uint8_t )nbytes);
    int index = 0;
    while (Wire.available())
    {
        data[index++] = Wire.read();
    }
    return (nbytes == index);
}

void Goodix::msSleep(uint16_t milliseconds) {
  delay(milliseconds);
}

void Goodix::usSleep(uint16_t microseconds) {
  delayMicroseconds(microseconds);
}