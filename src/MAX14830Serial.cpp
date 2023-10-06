/*
  MAX14830Serial.cpp
  Arduino MAX14830Serial library.
  MAX14830Serial.cpp (C) 2023 Hyobok Ahn

  A Maxim Integrated MAX14830 external USART/UART communication library for
  Arduino, built to be source code compatible with the Serial library, etc.
*/

#include <MAX14830Serial.h>

// Constructor. 
MAX14830Serial::MAX14830Serial(uint8_t port)
{
  if (port < 0 || port >= 4) {
    return;
  }
  _port = port;

  switch (port)
  {
      case 0 : _address = MAX14830_CH0; break;
      case 1 : _address = MAX14830_CH1; break;
      case 2 : _address = MAX14830_CH2; break;
      case 3 : _address = MAX14830_CH3; break;
  }
  Serial.print("port : "); Serial.print(_port);
  Serial.print(", address : 0x"); Serial.println(_address, HEX);
}

// Destructor
MAX14830Serial::~MAX14830Serial()
{
  end();
}

// Set the communication rate.
void MAX14830Serial::begin(uint32_t speed)
{
  //Wire.begin();

  // Reset UART
  startup(_address);

  // Set Reference Clock
  setRefClock(_address, REF_FREQ, 1);

  // Set TermIos
  setTermIos(_address);

  // Set Baud Rate
  setBaudRate(_address, speed);

  // Set GPIO
  setGpio();

}

void MAX14830Serial::end()
{
}

int MAX14830Serial::read()
{
  Serial.print("read() port : "); Serial.print(_port);
  Serial.print(", read() address : 0x"); Serial.println(_address, HEX);
  // RHR()
  uint8_t data = readRegister(_address, MAX14830_RHR_REG);
  return data;
}

int MAX14830Serial::available()
{
  Serial.print("available() port : "); Serial.print(_port);
  Serial.print(", available() address : 0x"); Serial.println(_address, HEX);
  // LSR()
  uint8_t lsr = readRegister(_address, MAX14830_LSR_IRQSTS_REG);
  return (lsr & 0x01) ? 1 : 0;  // TBC
}

int MAX14830Serial::_busy()
{
#if 0
  SPI.beginTransaction(spiSet);
  digitalWrite(_chipSelectPin, LOW);
  uint16_t conf = SPI.transfer16(MAX14830_CMD_READ_CONF);
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();
  return (!(conf & MAX14830_CONF_T)); // T flag is not set
#endif
  return 0;
}

size_t MAX14830Serial::write(uint8_t byte)
{
  Serial.print("write() port : "); Serial.print(_port);
  Serial.print(", write() address : 0x"); Serial.print(_address, HEX);
  Serial.print(", write() data : "); Serial.println(byte);
  // THR()
  writeRegister(_address, MAX14830_THR_REG, byte);

  return 0;
}

void MAX14830Serial::flush()
{
  // There is no buffer.  Wait for the transmit register to empty.
  while (_busy()) {}
}

int MAX14830Serial::peek()
{
  // This is not currently implemented in the hardware.  It could be implemented
  // with a one byte software buffer, but this would prevent /RM interrupts from
  // firing.
  return -1;
}

void MAX14830Serial::startup(uint8_t address) {

  // Reset UART
  writeRegister(address, MAX14830_MODE2_REG, MAX14830_MODE2_RST_BIT);
  writeRegister(address, MAX14830_MODE2_REG, ~(MAX14830_MODE2_RST_BIT) & readRegister(address, MAX14830_MODE2_REG));

  // Reset FIFO
  writeRegister(address, MAX14830_MODE2_REG, MAX14830_MODE2_FIFORST_BIT);
  writeRegister(address, MAX14830_MODE2_REG, ~(MAX14830_MODE2_FIFORST_BIT) & readRegister(address, MAX14830_MODE2_REG));

}

void MAX14830Serial::fifoClear(void) {

  // Reset FIFO
  writeRegister(_address, MAX14830_MODE2_REG, MAX14830_MODE2_FIFORST_BIT);
  writeRegister(_address, MAX14830_MODE2_REG, ~(MAX14830_MODE2_FIFORST_BIT) & readRegister(_address, MAX14830_MODE2_REG));

}

int32_t MAX14830Serial::updateBestErr(uint32_t f, int32_t *bestErr) {
#if (REF_BAUDRATE == 9600)
  int32_t err = f % (38400 * 16);
#else 
  // Use baudRate 115200 for calculate error
  int32_t err = f % (460800 * 16);
#endif

  if ((*bestErr < 0) || (*bestErr > err)) {
    *bestErr = err;
    return 0;
  }

  return 1;
}

void MAX14830Serial::setRefClock(uint8_t address, uint32_t freq, bool xtal) {

  uint32_t div, clksrc, pllcfg = 0;
  int32_t bestErr = -1;
  uint32_t fdiv, fmul, bestfreq = freq;

  // Reset Port

  // First, update error without PLL
  updateBestErr(freq, &bestErr);

  // Try all possible PLL dividers
  for (div = 1; (div <= 63) && bestErr; div++) {
    fdiv = DIV_ROUND_CLOSEST(freq, div);

    // Try multiplier 6
    fmul = fdiv * 6;
    if ((fdiv >= 500000) && (fdiv <= 800000))
      if (!updateBestErr(fmul, &bestErr)) {
        pllcfg = (0 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 48 
    fmul = fdiv * 48;
    if ((fdiv >= 850000) && (fdiv <= 1200000))
      if (!updateBestErr(fmul, &bestErr)) {
        pllcfg = (1 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 96
    fmul = fdiv * 96;
    if ((fdiv >= 425000) && (fdiv <= 1000000))
      if (!updateBestErr(fmul, &bestErr)) {
        pllcfg = (2 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 144
    fmul = fdiv * 144;
    if ((fdiv >= 390000) && (fdiv <= 667000))
      if (!updateBestErr(fmul, &bestErr)) {
        pllcfg = (3 << 6) | div;
        bestfreq = fmul;
      }
  }

  // Configure Clock Source
  clksrc = MAX14830_CLKSRC_EXTCLK_BIT | (xtal ? MAX14830_CLKSRC_CRYST_BIT : 0);

  // Configure PLL
  if (pllcfg) {
    clksrc |= MAX14830_CLKSRC_PLL_BIT;
    writeRegister(address, MAX14830_PLLCFG_REG, pllcfg);
  } else {
    clksrc |= MAX14830_CLKSRC_PLLBYP_BIT;
  }

  writeRegister(address, MAX14830_CLKSRC_REG, clksrc);

  // Wait for crystal
  if(xtal) {
    uint32_t val;
    delay(10); // msec
    val = readRegister(address, MAX14830_STS_IRQSTS_REG);
    if(!(val & MAX14830_STS_CLKREADY_BIT)) {
        Serial.println("clock is not stable yet");
    }
  }

}

void MAX14830Serial::setTermIos(uint8_t address) {

  uint8_t lcr = 0, flow = 0;

  // Set LCR Register
  lcr = MAX14830_LCR_LENGTH1_BIT | MAX14830_LCR_LENGTH0_BIT;  // 8 data length
  lcr &= ~(MAX14830_LCR_PARITY_BIT);  // no parity
  lcr &= ~(MAX14830_LCR_STOPLEN_BIT); // 1 stop bit

  writeRegister(address, MAX14830_LCR_REG, lcr);

  // Flow Control

  // Update Timeout

}

void MAX14830Serial::setBaudRate(uint8_t address, uint32_t baudRate) {
#ifdef DEBUG
  Serial.print("setBaudRate() port : "); Serial.print(_port);
  Serial.print(", setBaudRate() address : 0x"); Serial.println(address, HEX);
#endif
  // Divisor Latch Access (CLK Disable)
  writeRegister(address, MAX14830_BRGCFG_REG, 0x40 | readRegister(address, MAX14830_BRGCFG_REG));

  // Divisor 
  uint16_t div_factor = 0, div_mode = 0, frac = 0, frac_factor = 0;
  uint16_t divisor = REF_FREQ / baudRate;
  if (divisor < 8) {
    div_factor = 4;
    div_mode = MAX14830_BRGCFG_4XMODE_BIT;
  } else if (divisor < 16) {
    div_factor = 8;
    div_mode = MAX14830_BRGCFG_2XMODE_BIT;
  } else {
    div_factor = 16;
  }

  divisor /= div_factor;
  frac_factor = div_factor*baudRate;

  if(divisor > 0) {
    frac = (16*(REF_FREQ % frac_factor)) / frac_factor;
  } else {
    divisor = 1;
  }

#ifdef DEBUG
  Serial.print("actual baudRate : "); Serial.println(16*(REF_FREQ) / (div_factor*(16*divisor + frac)));
#endif

  // Divisor Latch 
  writeRegister(address, MAX14830_BRGDIVLSB_REG, divisor & 0xFF);
  writeRegister(address, MAX14830_BRGDIVMSB_REG, (divisor >> 8) & 0xFF);
  writeRegister(address, MAX14830_BRGCFG_REG, frac | div_mode);

  // Divisor Latch Access (CLK Enable) 
  writeRegister(address, MAX14830_BRGCFG_REG, ~0x40 & readRegister(address, MAX14830_BRGCFG_REG));

}

void MAX14830Serial::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
#ifdef DEBUG
  Serial.print("writeRegister() port : "); Serial.print(_port);
  Serial.print(", writeRegister() address : 0x"); Serial.println(address, HEX);
#endif

  Wire.beginTransmission(address);
  // Wire.write(address << 1);  // TBC
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();

}

uint8_t MAX14830Serial::readRegister(uint8_t address, uint8_t reg) {
#ifdef DEBUG
  Serial.print("readRegister() port : "); Serial.print(_port);
  Serial.print(", readRegister() address : 0x"); Serial.println(address, HEX);
#endif
  Wire.beginTransmission(address);
  // Wire.write(address << 1);  // TBC
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

uint16_t MAX14830Serial::writeData(uint8_t* buf, uint16_t length) {

  uint16_t ret = length;

  if(length > MAX_TX_BUF_SIZE) {
    return 0;
  }

  while(length--) {
    Wire.beginTransmission(_address);
    // Wire.write(address << 1);  // TBC
    Wire.write(MAX14830_THR_REG);
    Wire.write(*buf);
    Wire.endTransmission();
    buf++;
  }

  return ret;
}

uint16_t MAX14830Serial::readData(uint8_t* buf, uint16_t length) {

  uint16_t ret = length;

  if(length > MAX_RX_BUF_SIZE) {
    return 0;
  }

  Wire.beginTransmission(_address);
  // Wire.write(address << 1);  // TBC
  Wire.write(MAX14830_RHR_REG);
  Wire.endTransmission();

  Wire.requestFrom(_address, (uint8_t) length);

  while(length) {
    if (Wire.available()) {
      *buf = Wire.read();
      buf++;
      length--;
    }
  }

  return ret;    
}

void MAX14830Serial::setGpio(void) {

    writeRegister(_address, MAX14830_GPIOCFG_REG, 0x08);
    writeRegister(_address, MAX14830_GPIODATA_REG, 0x08);
}