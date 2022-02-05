#ifndef _GT911_H_
#define _GT911_H_

#include <Arduino.h>

#define GT911_I2C_ADDR_28 0x14
#define GT911_I2C_ADDR_BA 0x5D

#define GT911_CMD 0x8040
#define GT911_CFG 0x8047
#define GT911_CFG_MID 0x80A3 // for some reason reading config needs to be splited in half
#define GT911_CHKSUM 0x80FF

#define GT911_INFO 0x8140
#define GT911_DATA 0x814E
#define GT911_POINT_COUNT 5
#define GT911_POINT_DATA_SIZE 8

struct GTInfo
{
  char productId[4];
  uint16_t firmwareVersion;
  uint16_t xCoordResolution;
  uint16_t yCoordResolution;
  uint8_t vendorId;
};

struct GTPoint
{
  uint8_t trackId;
  uint16_t x;
  uint16_t y;
  uint16_t size;
};

struct GTConfig
{
  uint8_t configVersion;              // 0x8047
  uint16_t SWAPPEDxResolution;        // 0x8048
  uint16_t SWAPPEDyResolution;        // 0x804A
  uint8_t touchCount;                 // 0x804C
  uint8_t moduleSwitch1;              // 0x804D
  uint8_t moduleSwitch2;              // 0x804E
  uint8_t shakeCount;                 // 0x804F
  uint8_t filter;                     // 0x8050
  uint8_t largeTouch;                 // 0x8051
  uint8_t noiseReduction;             // 0x8052
  uint8_t touchThreshold;             // 0x8053
  uint8_t releaseThreshold;           // 0x8054
  uint8_t lowPowerControl;            // 0x8055
  uint8_t refreshRate;                // 0x8056
  uint8_t xThreshold;                 // 0x8057
  uint8_t yThreshold;                 // 0x8058
  uint8_t xSpeedLimit;                // 0x8059
  uint8_t ySpeedLimit;                // 0x805A
  uint8_t marginV;                    // 0x805B
  uint8_t marginH;                    // 0x805C
  uint8_t miniFilter;                 // 0x805D
  uint8_t stretchR0;                  // 0x805E
  uint8_t stretchR1;                  // 0x805F
  uint8_t stretchR2;                  // 0x8060
  uint8_t stretchRM;                  // 0x8061
  uint8_t drvGroupANum;               // 0x8062
  uint8_t drvGroupBNum;               // 0x8063
  uint8_t sensorNum;                  // 0x8064
  uint8_t freqAFactor;                // 0x8065
  uint8_t freqBFactor;                // 0x8066
  uint16_t SWAPPEDpanelBitFreq;       // 0x8067
  uint16_t SWAPPEDpanelSensorTime;    // 0x8069
  uint8_t panelTxGain;                // 0x806B
  uint8_t panelRxGain;                // 0x806C
  uint8_t panelDumpShift;             // 0x806D
  uint8_t drvFrameControl;            // 0x806E
  uint8_t chargeLevelUp;              // 0x806F
  uint8_t moduleSwitch3;              // 0x8070
  uint8_t gestureDistance;            // 0x8071
  uint8_t gestureLongPress;           // 0x8072
  uint8_t gestureSlopeAdjust;         // 0x8073
  uint8_t gestureControl;             // 0x8074
  uint8_t gestureSwitch1;             // 0x8075
  uint8_t gestureSwitch2;             // 0x8076
  uint8_t gestureRefreshRate;         // 0x8077
  uint8_t gestureTouchThreshold;      // 0x8078 (50)
  uint8_t gestureNewgreenWakeUpLevel; // 0x8079
  uint8_t freqHoppingStart;           // 0x807A
  uint8_t freqHoppingEnd;             // 0x807B
  uint8_t noiseDetectTime;            // 0x807C
  uint8_t hoppingFlag;                // 0x807D
  uint8_t hoppingThreshold;           // 0x807E
  uint8_t noiseThreshold;             // 0x807F
  uint8_t noiseMinThreshold;          // 0x8080
  uint8_t nc1;                        // 0x8081
  uint8_t hoppingSensorGroup;         // 0x8082
  uint8_t hoppingSeg1Normalize;       // 0x8083
  uint8_t hoppingSeg1Factor;          // 0x8084
  uint8_t mainClockAdjust;            // 0x8085
  uint8_t hoppingSeg2Normalize;       // 0x8086
  uint8_t hoppingSeg2Factor;          // 0x8087
  uint8_t nc2;                        // 0x8088
  uint8_t hoppingSeg3Normalize;       // 0x8089
  uint8_t hoppingSeg3Factor;          // 0x808A
  uint8_t nc3;                        // 0x808B
  uint8_t hoppingSeg4Normalize;       // 0x808C
  uint8_t hoppingSeg4Factor;          // 0x808D
  uint8_t nc4;                        // 0x808E
  uint8_t hoppingSeg5Normalize;       // 0x808F
  uint8_t hoppingSeg5Factor;          // 0x8090
  uint8_t nc5;                        // 0x8091
  uint8_t hoppingSeg6Normalize;       // 0x8092
  uint8_t key1Address;                // 0x8093
  uint8_t key2Address;                // 0x8094
  uint8_t key3Address;                // 0x8095
  uint8_t key4Address;                // 0x8096
  uint8_t keyArea;                    // 0x8097
  uint8_t keyTouchThreshold;          // 0x8098
  uint8_t keyReleaseThreshold;        // 0x8099
  uint8_t keySensitivity12;           // 0x809A
  uint8_t keySensitivity34;           // 0x809B
  uint8_t keyRestrain;                // 0x809C
  uint8_t keyRestrainTime;            // 0x809D
  uint8_t gestureLargeTouch;          // 0x809E
  uint8_t nc6;                        // 0x809F
  uint8_t nc7;                        // 0x80A0
  uint8_t hotKnotNoiseMap;            // 0x80A1
  uint8_t linkThreshold;              // 0x80A2
  uint8_t pxyThreshold;               // 0x80A3
  uint8_t ghotDumpShift;              // 0x80A4
  uint8_t ghotRxGain;                 // 0x80A5
  uint8_t freqGain0;                  // 0x80A6
  uint8_t freqGain1;                  // 0x80A7
  uint8_t freqGain2;                  // 0x80A8
  uint8_t freqGain3;                  // 0x80A9 (99)
  uint8_t ncg1[9];                    // 0x80AA -  0x80B2 (108)
  uint8_t combineDis;                 // 0x80B3 (109)
  uint8_t splitSet;                   // 0x80B4 (110)
  uint8_t nc8;                        // 0x80B5 (111)
  uint8_t nc9;                        // 0x80B6 (112)
  uint8_t sensor[14];                 // 0x80B7 - 0x80C4 (126)
  uint8_t ncg2[16];                   // 0x80C5 - 0x80D4 (142)
  uint8_t driver[26];                 // 0x80D5 - 0x80EE (168)
  uint8_t ncg3[16];                   // 0x80EF - 0x80FE (184)
  uint16_t xResolution;
  uint16_t yResolution;
  uint16_t panelBitFreq;
  uint16_t panelSensorTime;
};

class GT911
{
  uint8_t i2cAddr;
  GTInfo info;
  GTPoint points[GT911_POINT_COUNT];
  GTConfig config;

  bool write(uint16_t reg, uint8_t value);
  bool writeBytes(uint16_t reg, uint8_t *data, int nbytes);
  bool readBytes(uint16_t reg, uint8_t *data, int nbytes);
  int16_t readInput(uint8_t *data);

  uint8_t intPin, rstPin;
  void (*touchHandler)(int8_t, GTPoint *);
  void onIRQ();
  void i2cStart(uint16_t reg);

  uint8_t calcChecksum(uint8_t *buf, uint8_t len);
  uint8_t readChecksum();
  uint16_t swapByte(uint16_t in);

public:
  GT911();

  void setHandler(void (*handler)(int8_t, GTPoint *));

  uint8_t getI2CAddress();
  bool begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr = GT911_I2C_ADDR_BA);
  bool reset();
  void loop();

  // checks if local config checksum matches device checksum. true is good
  uint8_t configCheck();

  GTInfo &getInfo();
  bool readInfo();

  GTConfig &getConfig();
  bool readConfig();
  void writeConfig();
};

#endif
