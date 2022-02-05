#include "GT911.h"
#include "Wire.h"

#define EAGAIN 100         // Try again error
#define I2C_READ_ERROR 155 // I2C read error

volatile bool GT911IRQ = false;

GT911::GT911()
{
}

uint8_t GT911::getI2CAddress()
{
  return i2cAddr;
}

void _GT911_irq_handler()
{
  noInterrupts();
  GT911IRQ = true;
  interrupts();
}

void GT911::setHandler(void (*handler)(int8_t, GTPoint *))
{
  touchHandler = handler;
}

bool GT911::begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr)
{
  intPin = interruptPin;
  rstPin = resetPin;
  i2cAddr = addr;

  delay(300);
  bool result = reset();
  delay(200);

  return result;
}

bool GT911::reset()
{
  delay(1);

  pinMode(intPin, OUTPUT);
  pinMode(rstPin, OUTPUT);

  digitalWrite(intPin, LOW);
  digitalWrite(rstPin, LOW);

  delay(11);
  digitalWrite(intPin, i2cAddr == GT911_I2C_ADDR_28);

  delayMicroseconds(110);
  pinMode(rstPin, INPUT);

  delay(6);
  digitalWrite(intPin, LOW);

  delay(51);
  pinMode(intPin, INPUT);

  attachInterrupt(intPin, _GT911_irq_handler, RISING);

  return true;
}

void GT911::onIRQ()
{
  uint8_t data[GT911_POINT_DATA_SIZE * GT911_POINT_COUNT + 1]; // point data buffer
  int16_t count = readInput(data);

  switch (count)
  {
  case 5:
    points[4].trackId = data[33];
    points[4].x = ((uint16_t)data[35] << 8) + data[34];
    points[4].y = ((uint16_t)data[37] << 8) + data[36];
    points[4].size = ((uint16_t)data[39] << 8) + data[38];

  case 4:
    points[3].trackId = data[25];
    points[3].x = ((uint16_t)data[27] << 8) + data[26];
    points[3].y = ((uint16_t)data[29] << 8) + data[28];
    points[3].size = ((uint16_t)data[31] << 8) + data[30];

  case 3:
    points[2].trackId = data[17];
    points[2].x = ((uint16_t)data[19] << 8) + data[18];
    points[2].y = ((uint16_t)data[21] << 8) + data[20];
    points[2].size = ((uint16_t)data[23] << 8) + data[22];

  case 2:
    points[1].trackId = data[9];
    points[1].x = ((uint16_t)data[11] << 8) + data[10];
    points[1].y = ((uint16_t)data[13] << 8) + data[12];
    points[1].size = ((uint16_t)data[15] << 8) + data[14];

  case 1:
    points[0].trackId = data[1];
    points[0].x = ((uint16_t)data[3] << 8) + data[2];
    points[0].y = ((uint16_t)data[5] << 8) + data[4];
    points[0].size = ((uint16_t)data[7] << 8) + data[6];

    touchHandler(count, points);
  }
  return;
}

void GT911::loop()
{
  noInterrupts();
  bool irq = GT911IRQ;
  GT911IRQ = false;
  interrupts();

  if (irq)
  {
    onIRQ();
  }
}

uint8_t GT911::calcChecksum(uint8_t *buf, uint8_t len)
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++)
    sum += buf[i];

  sum = (~sum) + 1;
  return sum;
}

uint8_t GT911::readChecksum()
{
  uint16_t len = GT911_CHKSUM - GT911_CFG;
  uint8_t buf[len];
  readBytes(GT911_CFG, buf, len);
  return calcChecksum(buf, len);
}

uint8_t GT911::configCheck()
{
  uint8_t calc_check_sum;
  uint8_t read_check_sum[1];
  char prodID[5];

  write(GT911_CMD, 0);

  calc_check_sum = calcChecksum((uint8_t *)&config, sizeof(config));
  readBytes(GT911_CHKSUM, read_check_sum, 1);

  return read_check_sum[0] == calc_check_sum;
}

uint16_t GT911::swapByte(uint16_t in)
{
  return ((in & 0x00FF) << 8) + ((in & 0xFF00) >> 8);
}

GTInfo &GT911::getInfo()
{
  return info;
}

bool GT911::readInfo()
{
  return readBytes(GT911_INFO, (uint8_t *)&info, sizeof(info));
}

GTConfig &GT911::getConfig()
{
  return config;
}

bool GT911::readConfig()
{
  config.configVersion = 0;
  uint8_t len1 = GT911_CFG_MID - GT911_CFG;
  uint8_t len2 = GT911_CHKSUM - GT911_CFG_MID;
  uint8_t buf1[len1];
  uint8_t buf2[len2];
  if (!readBytes(GT911_CFG, buf1, len1))
    return false;
  if (!readBytes(GT911_CFG_MID, buf2, len2))
    return false;
  memcpy((uint8_t *)&config, buf1, sizeof(buf1));
  memcpy((uint8_t *)&config + sizeof(buf1), buf2, sizeof(buf2));

  config.xResolution = swapByte(config.SWAPPEDxResolution);
  config.yResolution = swapByte(config.SWAPPEDyResolution);
  config.panelBitFreq = swapByte(config.SWAPPEDpanelBitFreq);
  config.panelSensorTime = swapByte(config.SWAPPEDpanelSensorTime);

  return config.configVersion != 0;
}

void GT911::writeConfig()
{
  // config.configVersion++;
  config.SWAPPEDxResolution = swapByte(config.xResolution);
  config.SWAPPEDyResolution = swapByte(config.yResolution);
  config.SWAPPEDpanelBitFreq = swapByte(config.panelBitFreq);
  config.SWAPPEDpanelSensorTime = swapByte(config.panelSensorTime);

  uint16_t len = GT911_CHKSUM - GT911_CFG;
  uint8_t checksum[2];
  checksum[0] = calcChecksum((uint8_t *)&config, len);
  checksum[1] = 0x01;

  writeBytes(GT911_CFG, (uint8_t *)&config, len);
  writeBytes(GT911_CHKSUM, checksum, 2);
}

int16_t GT911::readInput(uint8_t *dest)
{
  bool result = readBytes(GT911_DATA, dest, GT911_POINT_DATA_SIZE * GT911_POINT_COUNT + 1);
  int count = dest[0] & 0b00001111;

  if (!result)
    return -I2C_READ_ERROR;
  if (!(dest[0] & 0b10000000)) // buffer status
    return -EAGAIN;

  write(GT911_DATA, 0);
  return count;
}

void GT911::i2cStart(uint16_t reg)
{
  Wire.beginTransmission(i2cAddr);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
}

bool GT911::write(uint16_t reg, uint8_t buf)
{
  i2cStart(reg);
  Wire.write(buf);
  return (Wire.endTransmission() != 0);
}

bool GT911::writeBytes(uint16_t reg, uint8_t *data, int nbytes)
{
  i2cStart(reg);
  for (int i = 0; i < nbytes; i++)
  {
    Wire.write(data[i]);
  }
  return (Wire.endTransmission() != 0);
}

bool GT911::readBytes(uint16_t reg, uint8_t *data, int nbytes)
{
  i2cStart(reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddr, (uint8_t)nbytes);
  int index = 0;
  while (Wire.available())
    data[index++] = Wire.read();

  return (nbytes == index);
}