/*
  $Id: SC16IS752Serial.h
  Arduino SC16IS752Serial library.
  SC16IS752Serial.h (C) 2023 Hyobok Ahn
*/

#ifndef SC16IS752SERIAL_H
#define SC16IS752SERIAL_H


#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

// SC16IS7XX register definitions
#define SC16IS7XX_RHR_REG		      (0x00) // Receive Holding Register - RX FIFO
#define SC16IS7XX_THR_REG		      (0x00) // Transmit Holding Register - TX FIFO
#define SC16IS7XX_IER_REG		      (0x01) // Interrupt Enable Register - Interrupt enable
#define SC16IS7XX_IIR_REG		      (0x02) // Interrupt Identification Register - Interrupt Identification
#define SC16IS7XX_FCR_REG		      (0x02) // FIFO Control Register - FIFO control
#define SC16IS7XX_LCR_REG		      (0x03) // Line Control Register - Line Control
#define SC16IS7XX_MCR_REG		      (0x04) // Modem Control Register - Modem Control
#define SC16IS7XX_LSR_REG		      (0x05) // Line Status Register - Line Status
#define SC16IS7XX_MSR_REG		      (0x06) // Modem Status Register - Modem Status
#define SC16IS7XX_SPR_REG		      (0x07) // Scratchpad Register - Scratch Pad
#define SC16IS7XX_TXLVL_REG		    (0x08) // Transmit FIFO Level Register - TX FIFO level
#define SC16IS7XX_RXLVL_REG		    (0x09) // Receive FIFO Level Register - RX FIFO level
#define SC16IS7XX_IODIR_REG		    (0x0a) // I/O pin Direction Register - I/O Direction
#define SC16IS7XX_IOSTATE_REG		  (0x0b) // I/O pin States Register - I/O State
#define SC16IS7XX_IOINTENA_REG		(0x0c) // I/O Interrupt Enable Register - I/O Interrupt Enable
#define SC16IS7XX_IOCONTROL_REG		(0x0e) // I/O pins Contol Register - I/O Control
#define SC16IS7XX_EFCR_REG		    (0x0f) // Extra Features Register - Extra Features Control

// TCR/TLR Register set: Only if ((MCR[2] == 1) && (EFR[4] == 1))
#define SC16IS7XX_TCR_REG		      (0x06) // Transmission Control Register - Transmit control
#define SC16IS7XX_TLR_REG		      (0x07) // Trigger Level Register - Trigger level

// Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF))
#define SC16IS7XX_DLL_REG		      (0x00) // divisor latch LSB - Divisor Latch Low
#define SC16IS7XX_DLH_REG		      (0x01) // divisor latch MSB - Divisor Latch High

// Enhanced Register set: Only if (LCR == 0xBF)
#define SC16IS7XX_EFR_REG		      (0x02) // Enhanced Feature Register - Enhanced Features
#define SC16IS7XX_XON1_REG		    (0x04) // Xon1 word
#define SC16IS7XX_XON2_REG		    (0x05) // Xon2 word
#define SC16IS7XX_XOFF1_REG		    (0x06) // Xoff1 word
#define SC16IS7XX_XOFF2_REG		    (0x07) // Xoff2 word

// IER register bits
#define SC16IS7XX_IER_RDI_BIT		  (1 << 0) // Enable RX data interrupt
#define SC16IS7XX_IER_THRI_BIT		(1 << 1) // Enable TX holding register
#define SC16IS7XX_IER_RLSI_BIT		(1 << 2) // Enable RX line status
#define SC16IS7XX_IER_MSI_BIT		  (1 << 3) // Enable Modem status

// IER register bits - write only if (EFR[4] == 1)
#define SC16IS7XX_IER_SLEEP_BIT		(1 << 4) // Enable Sleep mode
#define SC16IS7XX_IER_XOFFI_BIT		(1 << 5) // Enable Xoff interrupt
#define SC16IS7XX_IER_RTSI_BIT		(1 << 6) // Enable nRTS interrupt
#define SC16IS7XX_IER_CTSI_BIT		(1 << 7) // Enable nCTS interrupt

// FCR register bits
#define SC16IS7XX_FCR_FIFO_BIT		(1 << 0) // Enable FIFO
#define SC16IS7XX_FCR_RXRESET_BIT	(1 << 1) // Reset RX FIFO
#define SC16IS7XX_FCR_TXRESET_BIT	(1 << 2) // Reset TX FIFO
#define SC16IS7XX_FCR_RXLVLL_BIT	(1 << 6) // RX Trigger level LSB
#define SC16IS7XX_FCR_RXLVLH_BIT	(1 << 7) // RX Trigger level MSB

// FCR register bits - write only if (EFR[4] == 1)
#define SC16IS7XX_FCR_TXLVLL_BIT	(1 << 4) // TX Trigger level LSB
#define SC16IS7XX_FCR_TXLVLH_BIT	(1 << 5) // TX Trigger level MSB

// Reference Frequency
#define SC_REF_FREQ        1843200  // 1,843,200 (1.84320MHz)
#define SC_REF_BAUDRATE    9600

// A1:A0 = VDD:VDD  (0x90 - 1001 000X)
// A1:A0 = VSS:VSS  (0x9A - 1001 101X)
#define SC16IS752_SADDR0        0x4D //Device 0
#define SC16IS752_SADDR1        0x48 //Device 1

#define SC16IS752_CHANNEL_A     0x00
#define SC16IS752_CHANNEL_B     0x01

#define SC_RX_BUF_SIZE   128
#define SC_TX_BUF_SIZE   128

#define PIN_SC16IS752_RST    15

#define PIN_GPIO_CHA_RX_BIT   (1 << 4)
#define PIN_GPIO_CHA_TX_BIT   (1 << 5)
#define PIN_GPIO_CHB_RX_BIT   (1 << 0)
#define PIN_GPIO_CHB_TX_BIT   (1 << 1)

#define GPIO_VALUE_HIGH   1
#define GPIO_VALUE_LOW    0

class SC16IS752Serial : public Stream
{
public:
  // public methods
  SC16IS752Serial(uint8_t port);
  ~SC16IS752Serial();
  void begin(uint32_t speed);
  void end();
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual int _busy();
  virtual void flush();

  uint16_t writeData(uint8_t* buf, uint16_t length);
  uint16_t readData(uint8_t* buf, uint16_t length);
  void fifoClear(void);

  using Print::write;

private:
  uint8_t _address;
  uint8_t _port;
  uint8_t _channel;
  void writeRegister(uint8_t address, uint8_t channel, uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t address, uint8_t channel, uint8_t reg);

  int32_t updateBestErr(uint32_t f, int32_t *bestErr);
  void startup(uint8_t address, uint8_t channel);
  void setRefClock(uint8_t address, uint8_t channel, uint32_t freq, bool xtal);
  void setTermIos(uint8_t address, uint8_t channel);
  void setBaudRate(uint8_t address, uint8_t channel, uint32_t baudRate);
  void setGpio(void);
  void setLed(uint8_t gpio, uint8_t value);
};

#endif