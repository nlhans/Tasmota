/*
  xsns_102_soil.ino - Remote Soil Sensor driver

*/

#ifdef USE_I2C
#ifdef USE_SOILSENSE
/*********************************************************************************************\
 * 
 * The soil humidity sensor is connected to:
 * - IO26: Sensor I2C SCL (I2C clock line)
 * - IO27: Sensor I2C SDA (I2C data line)
 * 
 * The device address is on 0x55. The registerspace used for soil sensor functions is on 0x10 base
 * 
 * Register map:
 * - 0x10 SR: Status register; indicates last measurement config and busy flag
 * - 0x11 CR: Control register; set up new measurement and start flag
 * - 0x12 DATA: Soil sensor data, reads 3 bytes (16-bit decimal + 8-bit fraction)
 *              It's transmitted as: LH.F
 * 
 * The register set 0xF0 is for management functions:
 * - 0xFF: Nop (does nothing)
 * - 0xFE: Ping-pong handshake, reads 0x55
 * - 0xF0: Get capabilities: returns a list of supported peripherals in register set
 * - 0xF1: Get capabilities peripheral: returns descriptor of peripheral base address
 * 
 * The ping-pong handshake is used for detect
\*********************************************************************************************/

#define XSNS_102                          102
#define XI2C_92                          92    // See I2CDEVICES.md

#define SOILSENS_ADDR                         0x55

#define SOILSENS_ADDR_W                       0xAA
#define SOILSENS_ADDR_R                       0xAB

class SoftWire {
  public:
  SoftWire(int sda, int scl) : _sda(sda), _scl(scl) {
    _sclHigh();
    _sdaHigh();

    llStart(0);
    stop();
  }
    // Functions which take raw addresses (ie address passed must
    // already indicate read/write mode)
    uint8_t llStart(uint8_t rawAddr);
    uint8_t stop(bool allowClockStretch = true);

    uint8_t llWrite(uint8_t data);
    uint8_t llRead(uint8_t &data, bool sendAck = true);

    void _sclLow(void);
    void _sclHigh(void);
    void _sdaLow(void);
    void _sdaHigh(void);
    uint8_t _readScl();
    uint8_t _readSda();

  private:
    uint8_t _sda = 27;
    uint8_t _scl = 26;
    bool sSda = true;
    bool sScl = true;
    uint8_t _delay_us = 10;
    uint16_t _timeout_ms = 100;
};

void SoftWire::_sdaLow(void)
{
  if (sSda) {
    digitalWrite(_sda, LOW);
    pinMode(_sda, OUTPUT);
  }
  sSda = false;
}


void SoftWire::_sdaHigh(void)
{
  if (!sSda) {
    pinMode(_sda, INPUT);
  }
  sSda = true;
}


void SoftWire::_sclLow(void)
{
  if (sScl) {
    digitalWrite(_scl, LOW);
    pinMode(_scl, OUTPUT);
  }
  sScl = false;
}


void SoftWire::_sclHigh(void)
{
  if (!sScl) {
    pinMode(_scl, INPUT);
  }
  // while (!_readScl());
  sScl = true;
}


// Read SDA (for data read)
uint8_t SoftWire::_readSda()
{
  return digitalRead(_sda);
}


// Read SCL (to detect clock-stretching)
uint8_t SoftWire::_readScl()
{
  return digitalRead(_scl);
}

uint8_t SoftWire::stop(bool allowClockStretch) {
  // Force SCL low
  _sclLow();
  delayMicroseconds(_delay_us);

  // Force SDA low
  _sdaLow();
  delayMicroseconds(_delay_us);

  // Release SCL
  _sclHigh();
  delayMicroseconds(_delay_us);

  // Release SDA
  _sdaHigh();
  delayMicroseconds(_delay_us);

  return 0;
}


uint8_t SoftWire::llStart(uint8_t rawAddr) {
  // Force SDA low
  _sdaLow();
  delayMicroseconds(_delay_us);

  // Force SCL low
  _sclLow();
  delayMicroseconds(_delay_us);

  return llWrite(rawAddr);
}


uint8_t SoftWire::llWrite(uint8_t data) {
  for (uint8_t i = 8; i; --i) {
    // Force SCL low
    _sclLow();

    if (data & 0x80) {
      // Release SDA
      _sdaHigh();
    }
    else {
      // Force SDA low
      _sdaLow();
    }
    delayMicroseconds(_delay_us);

    // Release SCL
    _sclHigh();

    delayMicroseconds(_delay_us);

    data <<= 1;
  }

  // Get ACK
  // Force SCL low
  _sclLow();

  // Release SDA
  _sdaHigh();

  delayMicroseconds(_delay_us);

  // Release SCL
  _sclHigh();
  
  uint8_t res = (_readSda() == LOW ? 0 : 1);

  delayMicroseconds(_delay_us);

  // Keep SCL low between bytes
  _sclLow();

  return res;
}


uint8_t SoftWire::llRead(uint8_t &data, bool sendAck)
{
  data = 0;

  for (uint8_t i = 8; i; --i) {
    data <<= 1;

    // Force SCL low
    _sclLow();

    // Release SDA (from previous ACK)
    _sdaHigh();
    delayMicroseconds(_delay_us);

    // Release SCL
    _sclHigh();
    delayMicroseconds(_delay_us);

    if (_readSda())
      data |= 1;
  }

  // Put ACK/NACK
  // Force SCL low
  _sclLow();
  if (sendAck) {
    // Force SDA low
    _sdaLow();
  }
  else {
    // Release SDA
    _sdaHigh();
  }

  delayMicroseconds(_delay_us);

  // Release SCL
  _sclHigh();
  delayMicroseconds(_delay_us);

  // Wait for SCL to return high
  while (_readScl() == LOW);
  delayMicroseconds(_delay_us);

  // Keep SCL low between bytes
  _sclLow();

  return 0;
}




















static SoftWire ExtSensI2c = SoftWire(27, 26);
struct SOILSENSOR {
  float   pF_measure = NAN;
  float   pF_min = NAN;
  float   pF_max = NAN;
  float   soil_perc = NAN;
  uint8_t valid = 0;
  char    name[11] = "SoilSensor";
} SensorObj;

bool uw_cmdRW(uint8_t* txBf, size_t txSz, uint8_t* rxBf, size_t rxSz) {
  uint8_t sts;

  bool performWr = txSz > 0;
  bool performRd = rxSz > 0;

  if (performWr) {
    sts = ExtSensI2c.llStart(SOILSENS_ADDR_W);
    // AddLog(LOG_LEVEL_INFO, "I2c::Start W %X : sts %d", SOILSENS_ADDR_W, sts);
    // if (sts != 0) goto stop;

    for (size_t i = 0; i < txSz; i++) {
      sts = ExtSensI2c.llWrite(txBf[i]);
      // AddLog(LOG_LEVEL_INFO, "I2c::Tx %X : sts %d", txBf[i], sts);
      // if (sts != 0) goto stop;
    }
    ExtSensI2c.stop();
    delayMicroseconds(150);
    if (!performRd) goto end;
  }

  if (performRd) {
    sts = ExtSensI2c.llStart(SOILSENS_ADDR_R);
    // AddLog(LOG_LEVEL_INFO, "I2c::Start R %X : sts %d", SOILSENS_ADDR_R, sts);
    // if (sts != 0) goto stop;

    for (size_t i = 0; i < rxSz; i++) {
      sts = ExtSensI2c.llRead(rxBf[i], i+1 != rxSz);
      // AddLog(LOG_LEVEL_INFO, "I2c::Rx %X : sts %d", rxBf[i], sts);
      // if (sts != 0) goto stop;
    }
  }
stop:
  ExtSensI2c.stop();
    delayMicroseconds(150);
end:
  return sts == 0;
}

/********************************************************************************************/
void SoilSensorConvert(void) {
  uint8_t cmdCR[2] = {0x11, 0x11 }; // WR CR (addr 11h) with val 11h (01h = start, 10h = 8MHz sense)
  uw_cmdRW(cmdCR, sizeof(cmdCR), nullptr, 0);

  delay(10);

  uint8_t cmdData[1] = { 0x12 };
  uint8_t pf[3];
  uw_cmdRW(cmdData, sizeof(cmdData), pf, sizeof(pf));


  uint32_t pfdec = (((uint16_t)pf[1]) << 8) | pf[0];

  SensorObj.pF_measure = pfdec*1.0f + pf[2]/256.0f;
}

void SoilSensorDetect(void)
{
  uint8_t tx[1] = {0xFE};
  uint8_t rx[1] = {0};

  bool ok = uw_cmdRW(tx, sizeof(tx), rx, sizeof(rx));

  if (rx[0] == 0x55) {
    SensorObj.valid = 1;
    AddLog(LOG_LEVEL_INFO, S_LOG_I2C_FOUND_AT, SensorObj.name, SOILSENS_ADDR);
  }
}

void SoilSensorShow(bool json)
{
  if (SensorObj.valid) {
    if (json) {
      ResponseAppend_P(PSTR(",\"%s\":{\"pF\": %f"), SensorObj.name, SensorObj.pF_measure);
      ResponseJsonEnd();
    } else {
      WSContentSend_PD(HTTP_SNS_CAP_PF, SensorObj.name, &SensorObj.pF_measure);
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns102(uint8_t function)
{
  // if (!I2cEnabled(XI2C_91)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    SoilSensorDetect();
  }
  else if (SensorObj.valid) {
    switch (function) {
    case FUNC_EVERY_SECOND:
      SoilSensorConvert();
      break;
    case FUNC_JSON_APPEND:
      SoilSensorShow(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      SoilSensorShow(0);
      break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_SOILSENSE
#endif  // USE_I2C
