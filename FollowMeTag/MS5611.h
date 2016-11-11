
#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum
{
  MS5611_ULTRA_HIGH_RES   = 0x08,
  MS5611_HIGH_RES         = 0x06,
  MS5611_STANDARD         = 0x04,
  MS5611_LOW_POWER        = 0x02,
  MS5611_ULTRA_LOW_POWER  = 0x00
} 
ms5611_osr_t;


// MS5611, Standard address 0x77
#define MS5611_ADDR             0x77

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

#define BAROMETER_NEEDS_SAMPLES      0
#define BAROMETER_NEEDS_CALCULATION  1
#define BAROMETER_NEEDS_PROCESSING   2

#define BARO_SAMPLE_COUNT_MAX   48
#define BARO_SAMPLE_COUNT 21
#define PRESSURE_SAMPLE_COUNT 20

uint32_t ms5611_ut;  // static result of temperature measurement
uint32_t ms5611_up;  // static result of pressure measurement
uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;
static uint8_t ms5611_ct = CMD_ADC_4096;
int32_t baroPressure = 0;
int32_t baroTemperature = 0;
int32_t BaroAlt = 0;

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 0;
uint32_t baroPressureSum = 0;

void i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
  Wire.beginTransmission(addr_);
  Wire.write(reg_);
  for (int i = 0; i < len_; i++)
    Wire.write(data[i]);
  Wire.endTransmission();
}

void i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
  i2cWriteBuffer(addr_, reg_, 1, &data);
}

void i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf)
{
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.beginTransmission(addr_);
  Wire.requestFrom(addr_, len);
  while(!Wire.available()) {
  };
  for (int i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  Wire.endTransmission();
}

// ---------------------------------------------------------------

void ms5611_reset()
{
  Wire.beginTransmission(MS5611_ADDRESS);

#if ARDUINO >= 100
  Wire.write(MS5611_CMD_RESET);
#else
  Wire.send(MS5611_CMD_RESET);
#endif

  Wire.endTransmission();
}

void ms5611_setOversampling(ms5611_osr_t osr)
{
  switch (osr)
  {
  case MS5611_ULTRA_LOW_POWER:
    ms5611_ct = 1;
    break;
  case MS5611_LOW_POWER:
    ms5611_ct = 2;
    break;
  case MS5611_STANDARD:
    ms5611_ct = 3;
    break;
  case MS5611_HIGH_RES:
    ms5611_ct = 5;
    break;
  case MS5611_ULTRA_HIGH_RES:
    ms5611_ct = 10;
    break;
  }

  ms5611_osr = osr;
}

uint16_t ms5611_prom(int8_t coef_num)
{
  uint8_t rxbuf[2] = { 
    0, 0           };
  i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command
  return rxbuf[0] << 8 | rxbuf[1];
}

uint32_t ms5611_readRegister24(uint8_t reg)
{
  uint32_t value;
  Wire.beginTransmission(MS5611_ADDRESS);
#if ARDUINO >= 100
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.requestFrom(MS5611_ADDRESS, 3);
  while(!Wire.available()) {
  };
#if ARDUINO >= 100
  uint8_t vxa = Wire.read();
  uint8_t vha = Wire.read();
  uint8_t vla = Wire.read();
#else
  uint8_t vxa = Wire.receive();
  uint8_t vha = Wire.receive();
  uint8_t vla = Wire.receive();
#endif
  Wire.endTransmission();

  value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

  return value;
}

uint32_t ms5611_readRawTemperature(void)
{
  Wire.beginTransmission(MS5611_ADDRESS);

#if ARDUINO >= 100
  Wire.write(MS5611_CMD_CONV_D2 + ms5611_osr);
#else
  Wire.send(MS5611_CMD_CONV_D2 + ms5611_osr);
#endif

  Wire.endTransmission();

  delay(ms5611_ct);

  return ms5611_readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t ms5611_read_adc(void)
{
  uint8_t rxbuf[3];
  i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); // read ADC
  return ((int32_t)rxbuf[0] << 8) | ((int32_t)rxbuf[1] << 8) | rxbuf[2];
}

void ms5611_start_ut(void)
{
  //i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!

  Wire.beginTransmission(MS5611_ADDRESS);

#if ARDUINO >= 100
  Wire.write(MS5611_CMD_CONV_D2 + ms5611_osr);
#else
  Wire.send(MS5611_CMD_CONV_D2 + ms5611_osr);
#endif

  Wire.endTransmission();
}

void ms5611_get_ut(void)
{
  //ms5611_ut = ms5611_read_adc();
  ms5611_ut = ms5611_readRegister24(MS5611_CMD_ADC_READ);
}

void ms5611_start_up(void)
{
  //i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!

  Wire.beginTransmission(MS5611_ADDRESS);

#if ARDUINO >= 100
  Wire.write(MS5611_CMD_CONV_D1 + ms5611_osr);
#else
  Wire.send(MS5611_CMD_CONV_D1 + ms5611_osr);
#endif

  Wire.endTransmission();
}

void ms5611_get_up(void)
{
  ms5611_up = ms5611_readRegister24(MS5611_CMD_ADC_READ);
}

void ms5611_calculate(int32_t *pressure, int32_t *temperature)
{
  uint32_t press;
  int64_t temp;
  int64_t delt;
  int64_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
  int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
  int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
  temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

  if (temp < 2000) { // temperature lower than 20degC
    delt = temp - 2000;
    delt = 5 * delt * delt;
    off -= delt >> 1;
    sens -= delt >> 2;
    if (temp < -1500) { // temperature lower than -15degC
      delt = temp + 1500;
      delt = delt * delt;
      off -= 7 * delt;
      sens -= (11 * delt) >> 1;
    }
    temp -= ((dT * dT) >> 31);
  }
  press = ((((int64_t)ms5611_up * sens) >> BARO_SAMPLE_COUNT) - off) >> 15;


  if (pressure)
    *pressure = press;
  if (temperature)
    *temperature = temp;
}

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMP_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }

int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    QMP_COPY(p, v, 3);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
    return p[1];
}

#define PRESSURE_SAMPLES_MEDIAN 3

static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;
    
    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;
    
    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}


static uint32_t recalculateBarometerTotal(uint8_t baroSampleCount, uint32_t pressureTotal, int32_t newPressureReading)
{
  static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX];
  static int currentSampleIndex = 0;
  int nextSampleIndex;

  // store current pressure in barometerSamples
  nextSampleIndex = (currentSampleIndex + 1);
  if (nextSampleIndex == baroSampleCount) {
    nextSampleIndex = 0;
    //baroReady = true;
  }
  barometerSamples[currentSampleIndex] = applyBarometerMedianFilter(newPressureReading);

  // recalculate pressure total
  // Note, the pressure total is made up of baroSampleCount - 1 samples - See PRESSURE_SAMPLE_COUNT
  pressureTotal += barometerSamples[currentSampleIndex];
  pressureTotal -= barometerSamples[nextSampleIndex];

  currentSampleIndex = nextSampleIndex;

  return pressureTotal;
}

void baroUpdate(uint32_t currentTime)
{
  static uint32_t baroDeadline = 0;
  static int state = BAROMETER_NEEDS_SAMPLES;

  if ((int32_t)(currentTime - baroDeadline) < 0)
    return;

  baroDeadline = 0;
  switch (state) {
  case BAROMETER_NEEDS_SAMPLES:
    ms5611_get_ut();
    ms5611_start_up();
    state = BAROMETER_NEEDS_CALCULATION;
    baroDeadline += 10000;
    break;

  case BAROMETER_NEEDS_CALCULATION:
    ms5611_get_up();
    ms5611_start_ut();
    baroDeadline += 10000;
    ms5611_calculate(&baroPressure, &baroTemperature);
    state = BAROMETER_NEEDS_PROCESSING;
    break;

  case BAROMETER_NEEDS_PROCESSING:
    state = BAROMETER_NEEDS_SAMPLES;
    baroPressureSum = recalculateBarometerTotal(BARO_SAMPLE_COUNT, baroPressureSum, baroPressure);
    break;
  }

  baroDeadline += micros();        // make sure deadline is set after calling baro callbacks
}

int32_t baroCalculateAltitude(void)
{
  int32_t BaroAlt_tmp;

  // calculates height from ground via baro readings
  // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
  BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / PRESSURE_SAMPLE_COUNT) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
  BaroAlt_tmp -= baroGroundAltitude;
  BaroAlt = lrintf((float)BaroAlt * 0.6f + (float)BaroAlt_tmp * (1.0f - 0.6f)); // additional LPF to reduce baro noise

  return BaroAlt;
}

void performBaroCalibrationCycle(void)
{
    baroGroundPressure -= baroGroundPressure / 8;
    baroGroundPressure += baroPressureSum / PRESSURE_SAMPLE_COUNT;
    baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

    //calibratingB--;
}

void baro_setup() 
{
  //Serial.print("Baro...");
  
  ms5611_reset();
  ms5611_setOversampling(MS5611_ULTRA_HIGH_RES);
  delay(100);

  // read all coefficients
  for (int i = 0; i < PROM_NB; i++)
    ms5611_c[i] = ms5611_prom(i);

  //Serial.println("done.");
}

void baro_loop(unsigned long now)
{
  baroUpdate(now);
}






