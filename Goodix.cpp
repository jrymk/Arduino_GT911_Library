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

void Goodix::fwResolution(uint16_t maxX, uint16_t maxY) {
	uint8_t len1 = GOODIX_REG_CONFIG_MIDDLE - GOODIX_REG_CONFIG_DATA +1;
	uint8_t len2 = GOODIX_REG_CONFIG_END - GOODIX_REG_CONFIG_MIDDLE;
	uint8_t buf1[len1];
	uint8_t buf2[len2];
	uint8_t buf3[2];
	uint8_t buf[len1+len2];
	
	readBytes(GOODIX_REG_CONFIG_DATA, buf1, len1);
    readBytes(GOODIX_REG_CONFIG_MIDDLE+1, buf2, len2);
    memcpy(buf, buf1, sizeof(buf1));
	memcpy(buf+sizeof(buf1), buf2, sizeof(buf2));

  	buf[0]++;
	buf[1] = (maxX & 0xff);
	buf[2] = (maxX >> 8);
	buf[3] = (maxY & 0xff);
	buf[4] = (maxY >> 8);
	buf3[0] = calcChecksum(buf, len1+len2);
    buf3[1] = 0x01;

	writeBytes(GOODIX_REG_CONFIG_DATA, buf, len1+len2);
    writeBytes(GOODIX_REG_CONFIG_END+1, buf3, 2);
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
			if (LilyPi_config[i] != buf[i])
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

    buf[0] = calcChecksum(LilyPi_config, len1+len2);
    buf[1] = 0x01;
    write(GOODIX_REG_COMMAND, 0);
    productID(prodID);
    if (prodID[0] != '9')
    {
	    return;
    }
    writeBytes(GOODIX_REG_CONFIG_DATA, LilyPi_config, len1+len2);
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
    
	points[0].trackId = rawdata[1];	    
    points[0].x = ((uint16_t)rawdata[3] << 8) + rawdata[2];
    points[0].y = ((uint16_t)rawdata[5] << 8) + rawdata[4];
    points[0].area = ((uint16_t)rawdata[7] << 8) + rawdata[6];
    
    points[1].trackId = rawdata[9];
    points[1].x = ((uint16_t)rawdata[11] << 8) + rawdata[10];
    points[1].y = ((uint16_t)rawdata[13] << 8) + rawdata[12];
    points[1].area = ((uint16_t)rawdata[15] << 8) + rawdata[14];

    points[2].trackId = rawdata[17];
    points[2].x = ((uint16_t)rawdata[19] << 8) + rawdata[18];
    points[2].y = ((uint16_t)rawdata[21] << 8) + rawdata[20];
	points[2].area = ((uint16_t)rawdata[23] << 8) + rawdata[22];
    
    points[3].trackId = rawdata[25];
    points[3].x = ((uint16_t)rawdata[27] << 8) + rawdata[26];
    points[3].y = ((uint16_t)rawdata[29] << 8) + rawdata[28];
    points[3].area = ((uint16_t)rawdata[31] << 8) + rawdata[30];

    points[4].trackId = rawdata[33];
    points[4].x = ((uint16_t)rawdata[35] << 8) + rawdata[34];
    points[4].y = ((uint16_t)rawdata[37] << 8) + rawdata[36]; 
    points[4].area = ((uint16_t)rawdata[39] << 8) + rawdata[38];
    
    touchHandler(contacts, points);
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