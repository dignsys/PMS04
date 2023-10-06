/*
  SC16IS752Serial.cpp
  Arduino SC16IS752Serial library.
  SC16IS752Serial.cpp (C) 2023 Hyobok Ahn

  A NXP SC16IS752 external USART/UART communication library for
  Arduino, built to be source code compatible with the Serial library, etc.
*/

#include <SC16IS752Serial.h>

// Constructor. 
SC16IS752Serial::SC16IS752Serial(uint8_t port)
{
  if (port < 0 || port >= 4) {
    return;
  }
  _port = port;

  switch (port)
  {
      case 0 : _address = SC16IS752_SADDR0; _channel = SC16IS752_CHANNEL_A; break;
      case 1 : _address = SC16IS752_SADDR0; _channel = SC16IS752_CHANNEL_B; break;
      case 2 : _address = SC16IS752_SADDR1; _channel = SC16IS752_CHANNEL_A; break;
      case 3 : _address = SC16IS752_SADDR1; _channel = SC16IS752_CHANNEL_B; break;
  }
  Serial.print("port : "); Serial.print(_port);
  Serial.print(", address : 0x"); Serial.print(_address, HEX);
  Serial.print(",channel : "); Serial.println(_channel, DEC);
}

// Destructor
SC16IS752Serial::~SC16IS752Serial()
{
  end();
}

// Set the communication rate.
void SC16IS752Serial::begin(uint32_t speed)
{
  //Wire.begin();

  // Reset UART
  startup(_address, _channel);

  // Set Reference Clock
  setRefClock(_address, _channel, SC_REF_FREQ, 1);

  // Set Baud Rate
  setBaudRate(_address, _channel, speed);

  // Set TermIos
  setTermIos(_address, _channel);

  // Set GPIO
  setGpio();

}

void SC16IS752Serial::end()
{
}

int SC16IS752Serial::read()
{
  Serial.print("read() port : "); Serial.print(_port);
  Serial.print(", read() address : 0x"); Serial.println(_address, HEX);
  // RHR()
  uint8_t data = readRegister(_address, _channel, SC16IS7XX_RHR_REG);
  return data;
}

int SC16IS752Serial::available()
{
  //Serial.print("available() port : "); Serial.print(_port);
  //Serial.print(", available() address : 0x"); Serial.println(_address, HEX);
  // LSR()
  uint8_t lsr = readRegister(_address, _channel, SC16IS7XX_LSR_REG);
  return (lsr & 0x01) ? 1 : 0;  // TBC
}

int SC16IS752Serial::_busy()
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

size_t SC16IS752Serial::write(uint8_t byte)
{
  Serial.print("write() port : "); Serial.print(_port);
  Serial.print(", write() address : 0x"); Serial.print(_address, HEX);
  Serial.print(", write() data : "); Serial.println(byte);
  // THR()
  writeRegister(_address, _channel, SC16IS7XX_THR_REG, byte);

  return 0;
}

void SC16IS752Serial::flush()
{
  // There is no buffer.  Wait for the transmit register to empty.
  while (_busy()) {}
}

int SC16IS752Serial::peek()
{
  // This is not currently implemented in the hardware.  It could be implemented
  // with a one byte software buffer, but this would prevent /RM interrupts from
  // firing.
  return -1;
}

void SC16IS752Serial::startup(uint8_t address, uint8_t channel) {

  // Reset UART
  if(channel == SC16IS752_CHANNEL_A) {
    writeRegister(address, channel, SC16IS7XX_IOCONTROL_REG, 0x08 | readRegister(address, channel, SC16IS7XX_IOCONTROL_REG));
  }

  // Reset FIFO
  fifoClear();

}

void SC16IS752Serial::fifoClear(void) {

  uint8_t rtmp;
  rtmp = readRegister(_address, _channel, SC16IS7XX_FCR_REG) & 0xf0;

  // Reset FIFO
  writeRegister(_address, _channel, SC16IS7XX_FCR_REG, rtmp | SC16IS7XX_FCR_RXRESET_BIT | SC16IS7XX_FCR_TXRESET_BIT);
  writeRegister(_address, _channel, SC16IS7XX_FCR_REG, rtmp | SC16IS7XX_FCR_FIFO_BIT);

}

int32_t SC16IS752Serial::updateBestErr(uint32_t f, int32_t *bestErr) {
#if (SC_REF_BAUDRATE == 9600)
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

void SC16IS752Serial::setRefClock(uint8_t address, uint8_t channel, uint32_t freq, bool xtal) {

}

void SC16IS752Serial::setTermIos(uint8_t address, uint8_t channel) {

  uint8_t data_length = 8;
  uint8_t parity_select = 0;
  uint8_t stop_length = 1;
  uint8_t temp_lcr;

  temp_lcr = readRegister(address, channel, SC16IS7XX_LCR_REG);
  temp_lcr &= 0xC0; // Clear the lower six bit of LCR (LCR[0] to LCR[5]

  switch (data_length) { // data length settings
    case 5:
      break;
    case 6:
      temp_lcr |= 0x01;
      break;
    case 7:
      temp_lcr |= 0x02;
      break;
    case 8:
      temp_lcr |= 0x03;
      break;
    default:
      temp_lcr |= 0x03;
      break;
  }

  if (stop_length == 2) {
    temp_lcr |= 0x04;
  }

  switch (parity_select) { // parity selection length settings
    case 0:                // no parity
      break;
    case 1:                // odd parity
      temp_lcr |= 0x08;
      break;
    case 2:                // even parity
      temp_lcr |= 0x18;
      break;
    case 3:                // force '1' parity
      temp_lcr |= 0x03;
      break;
    case 4:                // force '0' parity
      break;
    default:
      break;
  }

  writeRegister(address, channel, SC16IS7XX_LCR_REG, temp_lcr);

}

void SC16IS752Serial::setBaudRate(uint8_t address, uint8_t channel, uint32_t baudRate) {
#ifdef DEBUG
  Serial.print("setBaudRate() port : "); Serial.print(_port);
  Serial.print(", setBaudRate() address : 0x"); Serial.println(address, HEX);
#endif
  uint16_t divisor;
  uint8_t  prescaler;
  uint32_t actual_baudrate;
  int16_t  error;
  uint8_t  temp_lcr;

  if ((readRegister(address, channel, SC16IS7XX_MCR_REG) & 0x80) == 0) { // if prescaler==1
    prescaler = 1;
  } else {
    prescaler = 4;
  }

  divisor = (SC_REF_FREQ / prescaler) / (baudRate * 16);

  temp_lcr  = readRegister(address, channel, SC16IS7XX_LCR_REG);
  temp_lcr |= 0x80;
  writeRegister(address, channel, SC16IS7XX_LCR_REG, temp_lcr);

  // write to DLL
  writeRegister(address, channel, SC16IS7XX_DLL_REG, (uint8_t)divisor);

  // write to DLH
  writeRegister(address, channel, SC16IS7XX_DLH_REG, (uint8_t)(divisor >> 8));
  temp_lcr &= 0x7F;
  writeRegister(address, channel, SC16IS7XX_LCR_REG, temp_lcr);


  actual_baudrate = (SC_REF_FREQ / prescaler) / (16 * divisor);
  error           = ((float)actual_baudrate - baudRate) * 1000 / baudRate;
#ifdef  DEBUG
  Serial.print("Desired baudrate: ");
  Serial.println(baudRate, DEC);
  Serial.print("Calculated divisor: ");
  Serial.println(divisor, DEC);
  Serial.print("Actual baudrate: ");
  Serial.println(actual_baudrate, DEC);
  Serial.print("Baudrate error: ");
  Serial.println(error, DEC);
#endif

}

void SC16IS752Serial::writeRegister(uint8_t address, uint8_t channel, uint8_t reg, uint8_t value) {
#ifdef DEBUG
  Serial.print("writeRegister() port : "); Serial.print(_port);
  Serial.print(", writeRegister() address : 0x"); Serial.println(address, HEX);
#endif

  Wire.beginTransmission(address);
  Wire.write((reg<<3)|(channel<<1));
  Wire.write(value);
  Wire.endTransmission();

}

uint8_t SC16IS752Serial::readRegister(uint8_t address, uint8_t channel, uint8_t reg) {
#ifdef DEBUG
  Serial.print("readRegister() port : "); Serial.print(_port);
  Serial.print(", readRegister() address : 0x"); Serial.println(address, HEX);
#endif
  Wire.beginTransmission(address);
  Wire.write((reg<<3)|(channel<<1));
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

uint16_t SC16IS752Serial::writeData(uint8_t* buf, uint16_t length) {

  uint16_t ret = length;
  uint16_t cnt = 0;

  if(length > SC_TX_BUF_SIZE) {
    return 0;
  }

  while(length--) {
#ifdef DEBUG
    Serial.print("wdata["); Serial.print(cnt); Serial.print("] = "); Serial.println(*buf, HEX);
    cnt++;
#endif
    Wire.beginTransmission(_address);
    Wire.write((SC16IS7XX_THR_REG<<3)|(_channel<<1));
    Wire.write(*buf);
    Wire.endTransmission();
    buf++;
  }

  return ret;
}

uint16_t SC16IS752Serial::readData(uint8_t* buf, uint16_t length) {

  uint16_t ret = length;
  uint16_t cnt = 0;

  if(length > SC_RX_BUF_SIZE) {
    return 0;
  }

  Wire.beginTransmission(_address);
  Wire.write((SC16IS7XX_RHR_REG<<3)|(_channel<<1));
  Wire.endTransmission();

  Wire.requestFrom(_address, (uint8_t) length);

  while(length) {
    if (Wire.available()) {
      *buf = Wire.read();
#ifdef DEBUG
      Serial.print("rdata["); Serial.print(cnt); Serial.print("] = "); Serial.println(*buf, HEX);
      cnt++;
#endif
      buf++;
      length--;
    }
  }

  return ret;    
}

void SC16IS752Serial::setGpio(void) {

  uint8_t rtmp;

  if(_channel == SC16IS752_CHANNEL_B) {
    return;
  }

  // Set IO Mode
  rtmp = readRegister(_address, _channel, SC16IS7XX_IOCONTROL_REG);
  writeRegister(_address, _channel, SC16IS7XX_IOCONTROL_REG, rtmp & 0xf9);

  // Set GPIO Output Direction
  rtmp = PIN_GPIO_CHA_RX_BIT | PIN_GPIO_CHA_TX_BIT | PIN_GPIO_CHB_RX_BIT | PIN_GPIO_CHB_TX_BIT;
  writeRegister(_address, _channel, SC16IS7XX_IODIR_REG, rtmp);

}

// e.g.) setLed(PIN_GPIO_CHA_RX_BIT, GPIO_VALUE_HIGH)
void SC16IS752Serial::setLed(uint8_t gpio, uint8_t value) {

  uint8_t rtmp;

  rtmp = readRegister(_address, _channel, SC16IS7XX_IOSTATE_REG);
  if(value) {
    writeRegister(_address, _channel, SC16IS7XX_IOSTATE_REG, rtmp | gpio);
  } else {
    writeRegister(_address, _channel, SC16IS7XX_IOSTATE_REG, rtmp & (~gpio));
  }
}