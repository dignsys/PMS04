/*
  $Id: MAX14830Serial.h
  Arduino MAX14830Serial library.
  MAX14830Serial.h (C) 2023 Hyobok Ahn
*/

#ifndef MAX14830SERIAL_H
#define MAX14830SERIAL_H


#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

// MAX14830 
#define MAX14830_RHR_REG            0x00 // Receive Hold Register - RX FIFO
#define MAX14830_THR_REG            0x00 // Transmit Hold Register - TX FIFO
#define MAX14830_IRQEN_REG          0x01 // IRQ Enable Register - IRQ enable 
#define MAX14830_IRQSTS_REG         0x02 // Interrupt Status Register - IRQ status
#define MAX14830_LSR_IRQEN_REG      0x03 // Line Status Interrupt Enable Register - LSR IRQ enable
#define MAX14830_LSR_IRQSTS_REG     0x04 // Line Status Register - LSR IRQ status
#define MAX14830_SPCHR_IRQEN_REG    0x05 // Special Character Interrupt Enable Register - Special char IRQ en
#define MAX14830_SPCHR_IRQSTS_REG   0x06 // Special Character Interrupt Register - Special char IRQ status
#define MAX14830_STS_IRQEN_REG      0x07 // STS Interrupt Enable Register - Status IRQ enable
#define MAX14830_STS_IRQSTS_REG     0x08 // Status Interrupt Register - Status IRQ status
#define MAX14830_MODE1_REG          0x09 // MODE1 Register - MODE1
#define MAX14830_MODE2_REG          0x0a // MODE2 Register - MODE2
#define MAX14830_LCR_REG            0x0b // Line Control Register - LCR
#define MAX14830_RXTO_REG           0x0c // Receiver Timeout Register - RX timeout
#define MAX14830_HDPIXDELAY_REG     0x0d // Auto transceiver delays
#define MAX14830_IRDA_REG           0x0e // IRDA settings
#define MAX14830_FLOWLVL_REG        0x0f // Flow Level Register - Flow control levels
#define MAX14830_FIFOTRIGLVL_REG    0x10 // FIFO Interrupt Trigger Level Register - FIFO IRQ trigger levels
#define MAX14830_TXFIFOLVL_REG      0x11 // Transmit FIFO Level Register - TX FIFO level
#define MAX14830_RXFIFOLVL_REG      0x12 // Receive FIFO Level Register - RX FIFO level
#define MAX14830_FLOWCTRL_REG       0x13 // Flow Control Register - Flow control
#define MAX14830_XON1_REG           0x14 // XON1 character
#define MAX14830_XON2_REG           0x15 // XON2 character
#define MAX14830_XOFF1_REG          0x16 // XOFF1 character
#define MAX14830_XOFF2_REG          0x17 // XOFF2 character
#define MAX14830_GPIOCFG_REG        0x18 // GPIO Configuration Register - GPIO config
#define MAX14830_GPIODATA_REG       0x19 // GPIO Data Register - GPIO data
#define MAX14830_PLLCFG_REG         0x1a // PLL Configuration Register - PLL config
#define MAX14830_BRGCFG_REG         0x1b // Baud-Rate Generator Configuration Register - Baud rate generator conf
#define MAX14830_BRGDIVLSB_REG      0x1c // Baud-Rate Generator LSB Divisor Register - Baud rate divisor LSB
#define MAX14830_BRGDIVMSB_REG      0x1d // Baud-Rate Generator MSB Divisor Register - Baud rate divisor MSB
#define MAX14830_CLKSRC_REG         0x1e // Clock Source Register - Clock source
#define MAX14830_REG_1F             0x1f
#define MAX14830_GLOBALIRQ_REG		MAX14830_REG_1F // Global IRQ Register - Global IRQ (RO)
#define MAX14830_GLOBALCMD_REG		MAX14830_REG_1F // Global Command Register - Global Command (WO)
#define MAX14830_TXSYNCH_REG        0x20 // Transmitter Synchronization Register
#define MAX14830_SYNCHDELAY1_REG    0x21 // Synchronization Delay Register 1
#define MAX14830_SYNCHDELAY2_REG    0x22 // Synchronization Delay Register 2
#define MAX14830_TIMER1_REG         0x23 // Timer Register 1
#define MAX14830_TIMER2_REG         0x24 // Timer Register 2
#define MAX14830_REVID_REG          0x25 // Revision Identification Register

// IRQ register bits
#define MAX14830_IRQ_LSR_BIT            (1 << 0) // LSR interrupt
#define MAX14830_IRQ_SPCHR_BIT          (1 << 1) // Special char interrupt
#define MAX14830_IRQ_STS_BIT            (1 << 2) // Status interrupt
#define MAX14830_IRQ_RXFIFO_BIT         (1 << 3) // RX FIFO interrupt
#define MAX14830_IRQ_TXFIFO_BIT         (1 << 4) // TX FIFO interrupt
#define MAX14830_IRQ_TXEMPTY_BIT        (1 << 5) // TX FIFO empty interrupt
#define MAX14830_IRQ_RXEMPTY_BIT        (1 << 6) // RX FIFO empty interrupt
#define MAX14830_IRQ_CTS_BIT            (1 << 7) // CTS interrupt

// LSR register bits
#define MAX14830_LSR_RXTO_BIT           (1 << 0) // RX timeout
#define MAX14830_LSR_RXOVR_BIT          (1 << 1) // RX overrun
#define MAX14830_LSR_RXPAR_BIT          (1 << 2) // RX parity error
#define MAX14830_LSR_FRERR_BIT          (1 << 3) // Frame error
#define MAX14830_LSR_RXBRK_BIT          (1 << 4) // RX break
#define MAX14830_LSR_RXNOISE_BIT        (1 << 5) // RX noise
#define MAX14830_LSR_CTS_BIT            (1 << 7) // CTS pin state

// Special character register bits
#define MAX14830_SPCHR_XON1_BIT         (1 << 0) // XON1 character
#define MAX14830_SPCHR_XON2_BIT         (1 << 1) // XON2 character
#define MAX14830_SPCHR_XOFF1_BIT        (1 << 2) // XOFF1 character
#define MAX14830_SPCHR_XOFF2_BIT        (1 << 3) // XOFF2 character
#define MAX14830_SPCHR_BREAK_BIT        (1 << 4) // RX break
#define MAX14830_SPCHR_MULTIDROP_BIT    (1 << 5) // 9-bit multidrop addr char

// Status register bits
#define MAX14830_STS_GPIO0_BIT          (1 << 0) // GPIO 0 interrupt
#define MAX14830_STS_GPIO1_BIT          (1 << 1) // GPIO 1 interrupt
#define MAX14830_STS_GPIO2_BIT          (1 << 2) // GPIO 2 interrupt
#define MAX14830_STS_GPIO3_BIT          (1 << 3) // GPIO 3 interrupt
#define MAX14830_STS_CLKREADY_BIT       (1 << 5) // Clock ready
#define MAX14830_STS_SLEEP_BIT          (1 << 6) // Sleep interrupt

// MODE1 register bits
#define MAX14830_MODE1_RXDIS_BIT        (1 << 0) // RX disable
#define MAX14830_MODE1_TXDIS_BIT        (1 << 1) // TX disable
#define MAX14830_MODE1_TXHIZ_BIT        (1 << 2) // TX pin three-state
#define MAX14830_MODE1_RTSHIZ_BIT       (1 << 3) // RTS pin three-state
#define MAX14830_MODE1_TRNSCVCTRL_BIT   (1 << 4) // Transceiver ctrl enable
#define MAX14830_MODE1_FORCESLEEP_BIT   (1 << 5) // Force sleep mode
#define MAX14830_MODE1_AUTOSLEEP_BIT    (1 << 6) // Auto sleep enable
#define MAX14830_MODE1_IRQSEL_BIT       (1 << 7) // IRQ pin enable

// MODE2 register bits
#define MAX14830_MODE2_RST_BIT          (1 << 0) // Chip reset
#define MAX14830_MODE2_FIFORST_BIT      (1 << 1) // FIFO reset
#define MAX14830_MODE2_RXTRIGINV_BIT    (1 << 2) // RX FIFO INT invert
#define MAX14830_MODE2_RXEMPTINV_BIT    (1 << 3) // RX FIFO empty INT invert
#define MAX14830_MODE2_SPCHR_BIT        (1 << 4) // Special chr detect enable
#define MAX14830_MODE2_LOOPBACK_BIT     (1 << 5) // Internal loopback enable
#define MAX14830_MODE2_MULTIDROP_BIT    (1 << 6) // 9-bit multidrop enable
#define MAX14830_MODE2_ECHOSUPR_BIT     (1 << 7) // ECHO suppression enable

// LCR register bits
/*
 * Word length bits table:
 * 00 -> 5 bit words
 * 01 -> 6 bit words
 * 10 -> 7 bit words
 * 11 -> 8 bit words
 */
#define MAX14830_LCR_LENGTH0_BIT        (1 << 0) // Word length bit 0
#define MAX14830_LCR_LENGTH1_BIT        (1 << 1) // Word length bit 1
/*
 * STOP length bit table:
 * 0 -> 1 stop bit
 * 1 -> 1-1.5 stop bits if
 *      word length is 5,
 *      2 stop bits otherwise
 */
#define MAX14830_LCR_STOPLEN_BIT        (1 << 2) // STOP length bit
#define MAX14830_LCR_PARITY_BIT         (1 << 3) // Parity bit enable
#define MAX14830_LCR_EVENPARITY_BIT     (1 << 4) // Even parity bit enable
#define MAX14830_LCR_FORCEPARITY_BIT    (1 << 5) // 9-bit multidrop parity
#define MAX14830_LCR_TXBREAK_BIT        (1 << 6) // TX break enable
#define MAX14830_LCR_RTS_BIT            (1 << 7) // RTS pin control

// IRDA register bits
#define MAX14830_IRDA_IRDAEN_BIT        (1 << 0) // IRDA mode enable
#define MAX14830_IRDA_SIR_BIT           (1 << 1) // SIR mode enable

// Flow control trigger level register masks
#define MAX14830_FLOWLVL_HALT_MASK      (0x000f) // Flow control halt level
#define MAX14830_FLOWLVL_RES_MASK       (0x00f0) // Flow control resume level
#define MAX14830_FLOWLVL_HALT(words)    ((words / 8) & 0x0f)
#define MAX14830_FLOWLVL_RES(words)     (((words / 8) & 0x0f) << 4)

// FIFO interrupt trigger level register masks
#define MAX14830_FIFOTRIGLVL_TX_MASK    (0x0f) // TX FIFO trigger level
#define MAX14830_FIFOTRIGLVL_RX_MASK    (0xf0) // RX FIFO trigger level
#define MAX14830_FIFOTRIGLVL_TX(words)  ((words / 8) & 0x0f)
#define MAX14830_FIFOTRIGLVL_RX(words)  (((words / 8) & 0x0f) << 4)

// Flow control register bits
#define MAX14830_FLOWCTRL_AUTORTS_BIT   (1 << 0) // Auto RTS flow ctrl enable
#define MAX14830_FLOWCTRL_AUTOCTS_BIT   (1 << 1) // Auto CTS flow ctrl enable
/* Enables that GPIO inputs
 * are used in conjunction with
 * XOFF2 for definition of
 * special character */
#define MAX14830_FLOWCTRL_GPIADDR_BIT   (1 << 2) 
#define MAX14830_FLOWCTRL_SWFLOWEN_BIT  (1 << 3) // Auto SW flow ctrl enable
/*
 * SWFLOW bits 1 & 0 table:
 * 00 -> no transmitter flow
 *       control
 * 01 -> receiver compares
 *       XON2 and XOFF2
 *       and controls
 *       transmitter
 * 10 -> receiver compares
 *       XON1 and XOFF1
 *       and controls
 *       transmitter
 * 11 -> receiver compares
 *       XON1, XON2, XOFF1 and
 *       XOFF2 and controls
 *       transmitter
 */
#define MAX14830_FLOWCTRL_SWFLOW0_BIT   (1 << 4) // SWFLOW bit 0
#define MAX14830_FLOWCTRL_SWFLOW1_BIT   (1 << 5) // SWFLOW bit 1
/*
 * SWFLOW bits 3 & 2 table:
 * 00 -> no received flow
 *       control
 * 01 -> transmitter generates
 *       XON2 and XOFF2
 * 10 -> transmitter generates
 *       XON1 and XOFF1
 * 11 -> transmitter generates
 *       XON1, XON2, XOFF1 and
 *       XOFF2
 */
#define MAX14830_FLOWCTRL_SWFLOW2_BIT   (1 << 6) // SWFLOW bit 2
#define MAX14830_FLOWCTRL_SWFLOW3_BIT   (1 << 7) // SWFLOW bit 3

// PLL configuration register masks
#define MAX14830_PLLCFG_PREDIV_MASK     (0x3f) // PLL predivision value
#define MAX14830_PLLCFG_PLLFACTOR_MASK  (0xc0) // PLL multiplication factor

// Baud rate generator configuration register bits
#define MAX14830_BRGCFG_2XMODE_BIT      (1 << 4) // Double baud rate
#define MAX14830_BRGCFG_4XMODE_BIT      (1 << 5) // Quadruple baud rate

// Clock source register bits
#define MAX14830_CLKSRC_CRYST_BIT       (1 << 1) // Crystal osc enable
#define MAX14830_CLKSRC_PLL_BIT         (1 << 2) // PLL enable
#define MAX14830_CLKSRC_PLLBYP_BIT      (1 << 3) // PLL bypass
#define MAX14830_CLKSRC_EXTCLK_BIT      (1 << 4) // External clock enable
#define MAX14830_CLKSRC_CLK2RTS_BIT     (1 << 7) // Baud clk to RTS pin

// Reference Frequency
//#define REF_FREQ        28230000  // 28,230,000 (28.23MHz)
#define REF_FREQ        3686400  // 3,686,400 (3.68640MHz)
#define REF_BAUDRATE    9600

// SA1:SA0 = GND:GND
#define MAX14830_CH0 0x6C //Channel 0
#define MAX14830_CH1 0x5C //Channel 1
#define MAX14830_CH2 0x2C //Channel 2
#define MAX14830_CH3 0x1C //Channel 3

#define DIV_ROUND_CLOSEST(x, divisor)(    \
{                                         \
  typeof(x) __x = x;                      \
  typeof(divisor) __d = divisor;          \
    (((typeof(x))-1) > 0 || (__x) > 0) ?  \
    (((__x) + ((__d) / 2)) / (__d)) :     \
    (((__x) - ((__d) / 2)) / (__d));      \
}                                         \
)

#define MAX_RX_BUF_SIZE   128
#define MAX_TX_BUF_SIZE   128

#define PIN_MAX14830_RST    15

class MAX14830Serial : public Stream
{
public:
  // public methods
  MAX14830Serial(uint8_t port);
  ~MAX14830Serial();
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
  void writeRegister(uint8_t address, uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t address, uint8_t reg);

  int32_t updateBestErr(uint32_t f, int32_t *bestErr);
  void startup(uint8_t address);
  void setRefClock(uint8_t address, uint32_t freq, bool xtal);
  void setTermIos(uint8_t address);
  void setBaudRate(uint8_t address, uint32_t baudRate);
  void setGpio(void);
};

#endif