/*
  xsns_101_shtc3.ino - Onboard SHTC3 for Plant Automation project

*/

#ifdef USE_I2C
#ifdef USE_SHTC3
/*********************************************************************************************\
 * 
 * The onboard sensors are connected to:
 * - IO23: Sensor power (GPIO High = ON, Low = OFF)
 * - IO22: Sensor I2C SCL (I2C clock line)
 * - IO21: Sensor I2C SDA (I2C data line)
 * 
 * The bus is powered via IO23. It must be turned high before any transactions can take place.
 * The following devices are present on the bus (7-bit I2C address, in brackets 8-bit for W/nR):
 * - 0x70 SHTC3: temperature & humiditry sensor (W: 0xE0, R: 0xE1)
 * - 0x29 BH1730FVC: light (lux) sensor (W: 0x52, R: 0x53)
\*********************************************************************************************/

#define XSNS_101                          101
#define XI2C_91                          91    // See I2CDEVICES.md

#define SHTC3_ADDR                       0x70

struct SHTC3 {
  float   temperature = NAN;
  float   humidity = NAN;
  uint8_t valid = 0;
  uint8_t count = 0;
  char    name[6] = "SHTC3";
} Shtc3;

bool Shtc3Read(void)
{
  if (Shtc3.valid) { Shtc3.valid--; }

  const uint16_t convertAndReadT = 0x7866;
  const uint8_t convertAndReadMsb = convertAndReadT >> 8;
  const uint8_t convertAndReadLsb = convertAndReadT & 0xFF;
  
  const uint16_t wakeup = 0x3517;
  const uint8_t wakeupMsb = wakeup >> 8;
  const uint8_t wakeupLsb = wakeup & 0xFF;
  Wire.beginTransmission(SHTC3_ADDR); // Write on address 0x70 , Wake the sensor
  Wire.write(wakeupMsb);
  Wire.write(wakeupLsb);
  if (Wire.endTransmission() != 0) { return false; }

  Wire.beginTransmission(SHTC3_ADDR); // Write on address 0x70 , Start conversion
  Wire.write(convertAndReadMsb);
  Wire.write(convertAndReadLsb);
  if (Wire.endTransmission() != 0) { return false; }

  delay(50);

  Wire.requestFrom(SHTC3_ADDR, 6);
  delay(5);
  uint8_t tempMsb = Wire.read();
  uint8_t tempLsb = Wire.read();
  uint8_t tempChk = Wire.read();
  uint8_t humidMsb = Wire.read();
  uint8_t humidLsb = Wire.read();
  uint8_t humidChk = Wire.read();

  float temperature = -45 + 175 * ((((uint16_t)tempMsb) << 8) | (tempLsb)) * 1.0f / (1<<16);
  float humidity = ((((uint16_t)humidMsb) << 8) | (humidLsb)) * 100.0f / (1<<16);

  Shtc3.humidity    = ConvertHumidity(humidity);
  Shtc3.temperature = ConvertTemp(temperature);

  if (isnan(Shtc3.temperature) || isnan(Shtc3.humidity)) { return false; }

  Shtc3.valid = SENSOR_MAX_MISS;
  return true;
}

/********************************************************************************************/

void Shtc3Detect(void)
{
  if (!I2cSetDevice(SHTC3_ADDR)) { return; }

  if (Shtc3Read()) {
    I2cSetActiveFound(SHTC3_ADDR, Shtc3.name);
    Shtc3.count = 1;
  }
}

void Shtc3EverySecond(void)
{
  if ((TasmotaGlobal.uptime % 10) == 0) {
    // SHTC3: 11mS
    if (!Shtc3Read()) {
      AddLogMissed(Shtc3.name, Shtc3.valid);
    }
  }
}

void Shtc3Show(bool json)
{
  if (Shtc3.valid) {
    TempHumDewShow(json, (0 == TasmotaGlobal.tele_period), Shtc3.name, Shtc3.temperature, Shtc3.humidity);
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns101(uint8_t function)
{
  // if (!I2cEnabled(XI2C_91)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    Shtc3Detect();
  }
  else if (Shtc3.count) {
    switch (function) {
    case FUNC_EVERY_SECOND:
      Shtc3EverySecond();
      break;
    case FUNC_JSON_APPEND:
      Shtc3Show(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      Shtc3Show(0);
      break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_SHTC3
#endif  // USE_I2C
