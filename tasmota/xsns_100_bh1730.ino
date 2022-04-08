/*
  xsns_101_shtc3.ino - Onboard BH1730 for Plant Automation project

*/

#ifdef USE_I2C
#ifdef USE_BH1730

/*********************************************************************************************\
 * 
 * The onboard sensors are connected to:
 * - IO23: Sensor power (GPIO High = ON, Low = OFF)
 * - IO22: Sensor I2C SCL (I2C clock line)
 * - IO21: Sensor I2C SDA (I2C data line)
 * 
 * The bus is powered via IO23. It must be turned high before any transactions can take place.
 * The following devices are present on the bus (7-bit I2C address, in brackets 8-bit for W/nR):
 * - 0x70 BH1730: temperature & humiditry sensor (W: 0xE0, R: 0xE1)
 * - 0x29 BH1730FVC: light (lux) sensor (W: 0x52, R: 0x53)
\*********************************************************************************************/

#define XSNS_100                          100
#define XI2C_90                          90    // See I2CDEVICES.md

#define BH1730_ADDR                       0x29

struct BH1730 {
  float   lx = NAN;
  uint8_t valid = 0;
  uint8_t count = 0;
  char    name[7] = "BH1730";
} Bh1730;

float convertLight(int gain, int itime) {
  Wire.beginTransmission(BH1730_ADDR); // Write on address 0x70 , Wake the sensor
  Wire.write(0x80|0x07); // write COMMAND,  GAIN register
  Wire.write(gain); // GAIN
  Wire.endTransmission();
  
  Wire.beginTransmission(BH1730_ADDR); // Write on address 0x70 , Wake the sensor
  Wire.write(0x80|0x01); // write COMMAND,  TIMING register
  Wire.write(256-itime); // TIMING
  Wire.endTransmission();
  
  Wire.beginTransmission(BH1730_ADDR); // Write on address 0x70 , Wake the sensor
  Wire.write(0x80); // write COMMAND,  control register
  Wire.write(0x0B); // POWER + ADC one-time + convert
  Wire.endTransmission();

  // Convert:
  bool rdy = false;
  uint8_t timeout = 50;
  do { 
    Wire.beginTransmission(BH1730_ADDR); // Write on address 0x70 , Wake the sensor
    Wire.write(0x80); // read COMMAND,  control register
    Wire.endTransmission();
    Wire.requestFrom(BH1730_ADDR, 1);
    uint8_t rdyRaw = Wire.read();
    Wire.endTransmission();
    
    rdy = (rdyRaw & (1<<4)) ? true :false;
    timeout--;
    delay(2);
  }
  while(!rdy && timeout > 0);

  // read data0
  Wire.beginTransmission(BH1730_ADDR); // Write on address 0x70 , Wake the sensor
  Wire.write(0x80 | 0x14); // read COMAND,  DATA0LOW register
    Wire.endTransmission();
  Wire.requestFrom(BH1730_ADDR, 2);
  uint16_t data0 = Wire.read();
  data0 |= ((uint16_t)Wire.read()) << 8;
  Wire.endTransmission();
  
  Wire.beginTransmission(BH1730_ADDR); // Write on address 0x70 , Wake the sensor
  Wire.write(0x80 | 0x16); // read COMMAND,  DATA1LOW register
  Wire.endTransmission();
  Wire.requestFrom(BH1730_ADDR, 2);
  uint16_t data1 = Wire.read();
  data1 |= ((uint16_t)Wire.read()) << 8;
  Wire.endTransmission();

  float lx = 0;
  if(data0 > 0) {
    const float BH1730_T_INT = 2.8;
    float BH1730_ITIME_MS = ((BH1730_T_INT/1000.0) * 964.0 * itime);
    
    float div = data1*1.0f/data0;
    int iGain;
    switch(gain) {
      case 0:
        iGain = 1;
      break;

      case 1:
        iGain = 2;
      break;

      case 2:
        iGain = 64;
      break;

      case 3:
        iGain = 128;
      break;
      
      default:
        iGain = 1;
      break;
    }
    if(div < 0.26) {
      lx = ((1.29 * data0) - (2.733 * data1)) / iGain * 102.6 / BH1730_ITIME_MS;
    }else if(div < 0.55) {
      lx = ((0.795 * data0) - (0.859 * data1)) / iGain * 102.6 / BH1730_ITIME_MS;
    }else if(div < 1.09) {
      lx = ((0.51 * data0) - (0.345 * data1)) / iGain * 102.6 / BH1730_ITIME_MS;
    }else if(div < 2.13) {
      lx = ((0.276 * data0) - (0.13 * data1)) / iGain * 102.6 / BH1730_ITIME_MS;
    } else {
      lx = 0;
    }
  }
  return lx;
}

bool Bh1730Read(void)
{
  if (Bh1730.valid) { Bh1730.valid--; }

  // First probe....
  // GAIN setting 0 = 1x
  // Timing of 4 results in integration period of approx 10.8ms 4*964*2.8us
  float luxProbe = convertLight(0, 8);

  int iTime;
  int gain;

  // Max light intensity of each gain setting under ITIME=0 and DATA1=0:
  // GAIN=1: 84.5 klux
  // GAIN=2: 32.8 klux
  // GAIN=64: 1024 lux
  // GAIN=128: 512 lux
  // Let's use order of magnitude lower:

  if (luxProbe < 50) {
    gain = 3; // 128x
//    iTime = 38; // 410.4ms
  } else if (luxProbe < 100) {
    gain = 2; // 64
//    iTime = 38;
  } else if (luxProbe < 3280) {
    gain = 1; // 2x
//    iTime = 38;
  } else if (luxProbe < 8450) {
    gain = 0; // 1x
//    iTime = 38;
  } else {
    gain = 0; // 1x
  }

  // Dynamic range:
  iTime = 38;
  if (luxProbe > 2048) {
    iTime = 38 - luxProbe / 4096; // @ 100klux, reduces iTime by 24 
  }

  Bh1730.lx = convertLight(gain, iTime);
  
  if (isnan(Bh1730.lx)) { return false; }

  Bh1730.valid = SENSOR_MAX_MISS;
  return true;
}

/********************************************************************************************/

void Bh1730Detect(void)
{
  if (!I2cSetDevice(BH1730_ADDR)) { return; }

  if (Bh1730Read()) {
    I2cSetActiveFound(BH1730_ADDR, Bh1730.name);
    Bh1730.count = 1;
  }
}

void Bh1730EverySecond(void)
{
  if (TasmotaGlobal.uptime &1) {
    // BH1730: 11mS
    if (!Bh1730Read()) {
      AddLogMissed(Bh1730.name, Bh1730.valid);
    }
  }
}

void Bh1730Show(bool json)
{
  if (Bh1730.valid) { 

    if (json) {
      ResponseAppend_P(PSTR(",\"%s\":{\"lux\": %*_f}"), Bh1730.name, Bh1730.lx);
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_PD(HTTP_SNS_ILLUMINANCE, Bh1730.name, 2, &Bh1730.lx);
#endif  // USE_WEBSERVER
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns100(uint8_t function)
{
  // if (!I2cEnabled(XI2C_90)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    Bh1730Detect();
  }
  else if (Bh1730.count) {
    switch (function) {
    case FUNC_EVERY_SECOND:
      Bh1730EverySecond();
      break;
    case FUNC_JSON_APPEND:
      Bh1730Show(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      Bh1730Show(0);
      break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_SHCT3
#endif  // USE_I2C
