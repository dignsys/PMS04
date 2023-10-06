/* 
 * PMS04 Main 
 * Author : DIGNSYS Inc.
 */

#include <Arduino.h>
#include <MAX14830Serial.h>
#include <SC16IS752Serial.h>
#include <PZEM004Tv30.h>
#include <SoftWire.h>
#include <AsyncDelay.h>
#include <SPI.h>
#include <Ethernet_Generic.h>

#define DECODE_NEC
//#define DECODE_DENON
#define DISABLE_CODE_FOR_RECEIVER // Disables restarting receiver after each send. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not used.
//#define SEND_PWM_BY_TIMER         // Disable carrier PWM generation in software and use (restricted) hardware PWM.
//#define USE_NO_SEND_PWM           // Use no carrier PWM, just simulate an active low receiver signal. Overrides SEND_PWM_BY_TIMER definition
#include <IRremote.hpp>

#ifdef USE_SC16IS752
SC16IS752Serial serial0 = SC16IS752Serial(0);
SC16IS752Serial serial1 = SC16IS752Serial(1);
SC16IS752Serial serial2 = SC16IS752Serial(2);
SC16IS752Serial serial3 = SC16IS752Serial(3);
#else
MAX14830Serial serial0 = MAX14830Serial(0);
MAX14830Serial serial1 = MAX14830Serial(1);
MAX14830Serial serial2 = MAX14830Serial(2);
MAX14830Serial serial3 = MAX14830Serial(3);
#endif

PZEM004Tv30 pzem0(serial0);
PZEM004Tv30 pzem1(serial1);
PZEM004Tv30 pzem2(serial2);
PZEM004Tv30 pzem3(serial3);

uint8_t rx_buf[MAX_RX_BUF_SIZE];
uint8_t tx_buf[MAX_TX_BUF_SIZE];

#define PZEM004_SUB_TEST

#ifdef PZEM004_SUB_TEST
#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009

#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42

#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

bool checkCRC(const uint8_t *buf, uint16_t len);
void setCRC(uint8_t *buf, uint16_t len);
uint16_t CRC16(const uint8_t *data, uint16_t len);
#endif

#define PIN_I2C_SDA       23
#define PIN_I2C_SCL       22
#define PIN_RTC_I2C_SDA   13  //25
#define PIN_RTC_I2C_SCL   14  //33
#define PIN_RS232_TXD     26  //27
#define PIN_RS232_RXD     25  //26
#define PIN_RS485_TXD     16  //17
#define PIN_RS485_RXD     4   //16
#define PIN_ETH_CS        17  //5
#define PIN_ETH_INT       38  // SENSOR_VP
#define PIN_ETH_MISO      18  //19
#define PIN_ETH_SCLK      5   //18
#define PIN_ETH_MOSI      19  //23

#define PIN_PCM_CONTROL   27
#define PIN_GPIO          32
#define PIN_IR            33
#define IR_SEND_PIN       PIN_IR

#define I2C_ADDR_RTC    0x68

uint8_t rtc_tx_buf[16];
uint8_t rtc_rx_buf[16];

#define I2C_ADDR_IO     0x20

SoftWire sw(PIN_RTC_I2C_SDA, PIN_RTC_I2C_SCL);
void printTwoDigit(int n);
void readTime(void);

#define PIN_W5500_RST       34  // GPIO Expander
#define PIN_SC16IS752_RST   15

uint8_t nc_mac[] = {
  0x00, 0x08, 0xDC, 0x00, 0x00, 0x00
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress nc_ip(192, 168, 1, 35);
IPAddress nc_dns(192, 168, 1, 1);
IPAddress nc_gateway(192, 168, 1, 1);
IPAddress nc_subnet(255, 255, 255, 0);

SPIClass* pspi;
DhcpClass* dhcp = new DhcpClass();

#define PIN_MAX485_DE       21    //4
#define MAX485_DIR_SEND     LOW   //HIGH
#define MAX485_DIR_RECEIVE  HIGH  //LOW

void setRS485Dir(bool dir);
int setAddress(char c, uint8_t* paddr, uint8_t* pch);

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_read_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);
int i2c_write_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen);

#define LED_ON      1
#define LED_OFF     0
#define ETH_RST_ON  1
#define ETH_RST_OFF 0
#define SWITCH_ON   1
#define SWITCH_OFF  0

void gpio_exp_conf(void);
void gpio_exp_read(uint16_t* pdata);
void gpio_exp_write(uint16_t data);

void led_pm(uint8_t num, uint8_t on);
void led_tx(uint8_t on);
void led_rx(uint8_t on);
void led_fail(uint8_t on);

void eth_rst(void);

int get_swtich_val(uint8_t num);

void dual_uart_led_init(void);
void dual_uart_led_set(uint8_t num, uint8_t on);

void sub_test_a(uint8_t ireg);
void sub_test_b(void);
void sub_test_c(void);
int32_t sub_test_c_01(uint32_t f, int32_t *bestErr);
void sub_test_d(void);
void sub_test_e(void);
void sub_test_f(void);
void sub_test_g(void);
void sub_test_h(void);
void sub_test_i(uint8_t iaddr);
void sub_test_l(void);
void sub_test_m(void);
void sub_test_n(void);
void sub_test_o(void);
void sub_test_p(void);
void sub_test_q(void);
void sub_test_y(void);
void sub_test_z(void);

// Run this once at power on or reset.
void setup() {

  // RTC Initialization
  sw.setTxBuffer(rtc_tx_buf, sizeof(rtc_tx_buf));
  sw.setRxBuffer(rtc_rx_buf, sizeof(rtc_rx_buf));
  sw.setDelay_us(5);
  sw.setTimeout(1000);
  sw.begin();

  // GPIO Expander
  gpio_exp_conf();

  eth_rst();

  pspi = new SPIClass(VSPI);
  pspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);

  // Reset SC16IS752
  pinMode(PIN_SC16IS752_RST, OUTPUT);
  digitalWrite(PIN_SC16IS752_RST, LOW);
  delay(100);
  digitalWrite(PIN_SC16IS752_RST, HIGH);
  delay(100);

  // GPIO
  pinMode(PIN_GPIO, OUTPUT);
  digitalWrite(PIN_GPIO, LOW);

  // Arduino USART setup.
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RS232_RXD, PIN_RS232_TXD);  // RS232
  Serial2.begin(115200, SERIAL_8N1, PIN_RS485_RXD, PIN_RS485_TXD);  // RS485
  pinMode(PIN_MAX485_DE, OUTPUT);
  digitalWrite(PIN_MAX485_DE, MAX485_DIR_SEND);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
#ifdef USE_SC16IS752
  Wire.setClock(400000);  // 400kHz
#else
  Wire.setClock(1000000);  // 1MHz, 400kHz, 100kHz
#endif

  // MAX14830 UART setup. (begin is excuted here instead of pzem creation)
  serial0.begin(REF_BAUDRATE);
  serial1.begin(REF_BAUDRATE);
  serial2.begin(REF_BAUDRATE);
  serial3.begin(REF_BAUDRATE);

  // PZEM Initialization
  pzem0.init(&serial0, false, PZEM_DEFAULT_ADDR);
  pzem1.init(&serial1, false, PZEM_DEFAULT_ADDR);
  pzem2.init(&serial2, false, PZEM_DEFAULT_ADDR);
  pzem3.init(&serial3, false, PZEM_DEFAULT_ADDR);

  // GPIO in SC16IS752 (need to init after pzem ?)
  dual_uart_led_init();

  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  Serial.print(F("Send IR signals at pin "));
  Serial.println(IR_SEND_PIN);

  //IrSender.begin(); // Start with IR_SEND_PIN as send pin and if NO_LED_FEEDBACK_CODE is NOT defined, enable feedback LED at default feedback LED pin
  IrSender.begin(DISABLE_LED_FEEDBACK); // Start with IR_SEND_PIN as send pin and disable feedback LED at default feedback LED pin

}

// Process this loop whenever we see a serial event or interrupt from the MAX14830
void loop() {
  // put your main code here, to run repeatedly:
  
  // Display USART header on Arduino.
  Serial.println();
  Serial.println("PMS04 Board testing.");
  Serial.println("(C) 2023 Dignsys");

  char c;
  while(c != 'x') {
    Serial.printf("Input Command: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    Serial.printf("%c", c);
    Serial.println();

    switch(c) {
      case 'a': 
        sub_test_a(0);
        break;
      case 'b':
        sub_test_b();
        break;
      case 'c':
        sub_test_c();
        break;
      case 'd':
        sub_test_d();
        break;
      case 'e':
        sub_test_e();
        break;
      case 'f':
        sub_test_f();
        break;
      case 'g':
        sub_test_g();
        break;
      case 'h':
        sub_test_h();
        break;
      case 'i':
        sub_test_i(0);
        break;
      case 'j':
        for(int i = 1; i < 0xf8; i++){
          sub_test_i((uint8_t)i);
        }
        break;
      case 'k':
        sub_test_a(MAX14830_LSR_IRQSTS_REG);
        break;
      case 'l':
        sub_test_l();
        break;
      case 'm':
        sub_test_m();
        break;
      case 'n':
        sub_test_n();
        break;
      case 'o':
        sub_test_o();
        break;
      case 'p':
        sub_test_p();
        break;
      case 'q':
        sub_test_q();
        break;
      case 'y':
        sub_test_y();
        break;
      case 'z':
        sub_test_z();
        break;
      default:
        break;
    }
  }
  Serial.println("loop exit!");

  sleep(1000);
}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(addr == 0x10){
    Wire.endTransmission(false);
  } else {
    Wire.endTransmission();
  }
  
  if(addr == 0x10) {
      Wire.requestFrom(addr, (size_t)dlen, (bool) false);
  } else {
    Wire.requestFrom(addr, (uint8_t)dlen);
  }
  if (Wire.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = Wire.read();
    }
  }

  return ret;
}

int i2c_write(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  Wire.beginTransmission(addr);
  Wire.write(reg);
  for(int i = 0; i < dlen; i++) {
    Wire.write(pdata[i]);
  }
  Wire.endTransmission();

  return ret;
}

int i2c_read_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw.beginTransmission(addr);
  sw.write(reg);
  sw.endTransmission();
  
  sw.requestFrom(addr, (uint8_t)dlen);

  if (sw.available()) {
    for(int i = 0; i < dlen; i++) {
      pdata[i] = sw.read();
    }
  }

  return ret;
}

int i2c_write_sw(uint8_t addr, uint8_t reg, uint8_t* pdata, uint8_t dlen){

  int ret = 0;

  sw.beginTransmission(addr);
  sw.write(reg);
  for(int i = 0; i < dlen; i++) {
    sw.write(pdata[i]);
  }
  sw.endTransmission();

  return ret;
}

void gpio_exp_conf(void){

  uint8_t data[3] = {0,};

  data[0] = 0xff;
  data[1] = 0x00;

  // IN/OUT set
  i2c_write_sw(I2C_ADDR_IO, 0x06, data, 2);

}

void gpio_exp_read(uint16_t* pdata){

  uint8_t rdata[3] = {0,};

  i2c_read_sw(I2C_ADDR_IO, 0x00, rdata, 2);

  *pdata = rdata[1] * 0x100 + rdata[0];

}

void gpio_exp_write(uint16_t data){

  uint8_t wdata[3] = {0,};

  wdata[0] = (uint8_t) (data & 0x00ff);
  wdata[1] = (uint8_t) ((data>>8) & 0x00ff);

  i2c_write_sw(I2C_ADDR_IO, 0x02, wdata, 2);

}

void led_pm(uint8_t num, uint8_t on){

  uint16_t num_bit;
  uint16_t data = 0;

  if(!num || (num > 4)){
    Serial.print("Invalid PM LED[1~4] Set:"); Serial.println(num);
    return;
  }

  gpio_exp_read(&data);

  if(on == LED_OFF){
    num_bit = 0x1000 << (num-1);
    data |= num_bit;
  } else {
    num_bit = ~(0x1000 << (num-1));
    data &= num_bit;
  }
  
  gpio_exp_write(data);

}

void led_tx(uint8_t on){

  uint16_t data = 0;

  gpio_exp_read(&data);

  if(on == LED_OFF){
    data |= 0x200;
  } else {
    data &= 0xfdff;
  }
  
  gpio_exp_write(data);

}

void led_rx(uint8_t on){

  uint16_t data = 0;

  gpio_exp_read(&data);

  if(on == LED_OFF){
    data |= 0x400;
  } else {
    data &= 0xfbff;
  }
  
  gpio_exp_write(data);

}

void led_fail(uint8_t on){

  uint16_t data = 0;

  gpio_exp_read(&data);

  if(on == LED_OFF){
    data |= 0x100;
  } else {
    data &= 0xfeff;
  }
  
  gpio_exp_write(data);

}

void eth_rst(void){

  uint16_t data = 0;

  gpio_exp_read(&data);

  data &= 0xf7ff;
  gpio_exp_write(data);

  delay(20);

  data |= 0x800;
  gpio_exp_write(data);

}

int get_swtich_val(uint8_t num){

  int ret = 0;
  uint16_t num_bit;
  uint16_t data = 0;

  if(!num || (num > 8)){
    Serial.print("Invalid Switch[1~8] Set:"); Serial.println(num);
    return ret;
  }

  num_bit = 0x01 << (num-1);
  gpio_exp_read(&data);

  if(data & num_bit) {
    ret = SWITCH_OFF;
  } else {
    ret = SWITCH_ON;
  }

  return ret;
}

void dual_uart_led_init(void){

  uint8_t data[2] = {0,};

//reg = (reg<<3)|(channel<<1);

  // device 0
  // IO Control
  i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOCONTROL_REG<<3, data, 1);
  data[0] &= 0xfb;
  i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOCONTROL_REG<<3, data, 1);

  // IO Direction
  i2c_read(SC16IS752_SADDR0, SC16IS7XX_IODIR_REG<<3, data, 1);
  data[0] |= 0x03;
  i2c_write(SC16IS752_SADDR0, SC16IS7XX_IODIR_REG<<3, data, 1);

  // IO State
  i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
  data[0] &= 0xfc;
  i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);

  // device 1
  // IO Control
  i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOCONTROL_REG<<3, data, 1);
  data[0] &= 0xfb;
  i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOCONTROL_REG<<3, data, 1);

  // IO Direction
  i2c_read(SC16IS752_SADDR1, SC16IS7XX_IODIR_REG<<3, data, 1);
  data[0] |= 0x03;
  i2c_write(SC16IS752_SADDR1, SC16IS7XX_IODIR_REG<<3, data, 1);

  // IO State
  i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
  data[0] &= 0xfc;
  i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);

}

void dual_uart_led_set(uint8_t num, uint8_t on){

  uint8_t data[2] = {0,};

  if((num < 2) || (num > 5)){
    Serial.print("Invalid Dual Uart LED[2~5] Set:"); Serial.println(num);
    return;
  }

  if(num == 2){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x01;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfe;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  } else if(num == 3){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x02;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfd;
      i2c_write(SC16IS752_SADDR0, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  } else if(num == 4){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x01;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfe;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  } else if(num == 5){
    if(on == LED_ON){
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] |= 0x02;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    } else {
      i2c_read(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
      data[0] &= 0xfd;
      i2c_write(SC16IS752_SADDR1, SC16IS7XX_IOSTATE_REG<<3, data, 1);
    }
  }
}

// Print with leading zero, as expected for time
void printTwoDigit(int n)
{
  if (n < 10) {
    Serial.print('0');
  }
  Serial.print(n);
}

void readTime(void)
{
  // Ensure register address is valid
  sw.beginTransmission(I2C_ADDR_RTC);
  sw.write(uint8_t(0)); // Access the first register
  sw.endTransmission();

  uint8_t registers[7]; // There are 7 registers we need to read from to get the date and time.
  int numBytes = sw.requestFrom(I2C_ADDR_RTC, (uint8_t)7);
  for (int i = 0; i < numBytes; ++i) {
    registers[i] = sw.read();
  }
  if (numBytes != 7) {
    Serial.print("Read wrong number of bytes: ");
    Serial.println((int)numBytes);
    return;
  }

  int tenYear = (registers[6] & 0xf0) >> 4;
  int unitYear = registers[6] & 0x0f;
  int year = (10 * tenYear) + unitYear;

  int tenMonth = (registers[5] & 0x10) >> 4;
  int unitMonth = registers[5] & 0x0f;
  int month = (10 * tenMonth) + unitMonth;

  int tenDateOfMonth = (registers[4] & 0x30) >> 4;
  int unitDateOfMonth = registers[4] & 0x0f;
  int dateOfMonth = (10 * tenDateOfMonth) + unitDateOfMonth;

  // Reading the hour is messy. See the datasheet for register details!
  bool twelveHour = registers[2] & 0x40;
  bool pm = false;
  int unitHour;
  int tenHour;
  if (twelveHour) {
    pm = registers[2] & 0x20;
    tenHour = (registers[2] & 0x10) >> 4;
  } else {
    tenHour = (registers[2] & 0x30) >> 4;
  }
  unitHour = registers[2] & 0x0f;
  int hour = (10 * tenHour) + unitHour;
  if (twelveHour) {
    // 12h clock? Convert to 24h.
    hour += 12;
  }

  int tenMinute = (registers[1] & 0xf0) >> 4;
  int unitMinute = registers[1] & 0x0f;
  int minute = (10 * tenMinute) + unitMinute;

  int tenSecond = (registers[0] & 0xf0) >> 4;
  int unitSecond = registers[0] & 0x0f;
  int second = (10 * tenSecond) + unitSecond;

  // ISO8601 is the only sensible time format
  Serial.print("Time: ");
  Serial.print(year);
  Serial.print('-');
  printTwoDigit(month);
  Serial.print('-');
  printTwoDigit(dateOfMonth);
  Serial.print('T');
  printTwoDigit(hour);
  Serial.print(':');
  printTwoDigit(minute);
  Serial.print(':');
  printTwoDigit(second);
  Serial.println();
}

void setRS485Dir(bool dir) {

  if(dir == MAX485_DIR_SEND) {
    digitalWrite(PIN_MAX485_DE, MAX485_DIR_SEND);
  } else if(dir == MAX485_DIR_RECEIVE) {
    digitalWrite(PIN_MAX485_DE, MAX485_DIR_RECEIVE);
  } else {
    Serial.println("Invalid set of MAX485");
  }
}

int setAddress(char c, uint8_t* paddr, uint8_t* pch) {

  int ret = 0;

  #ifdef USE_SC16IS752
  if(c == '0') {
    *paddr = SC16IS752_SADDR0; *pch = SC16IS752_CHANNEL_A;
  } else if (c == '1') {
    *paddr = SC16IS752_SADDR0; *pch = SC16IS752_CHANNEL_B;
  } else if (c == '2') {
    *paddr = SC16IS752_SADDR1; *pch = SC16IS752_CHANNEL_A;
  } else if (c == '3') {
    *paddr = SC16IS752_SADDR1; *pch = SC16IS752_CHANNEL_B;
  } else {
    ret = 1;
  }
  #else
  if(c == '0') {
    *paddr = MAX14830_CH0;
  } else if (c == '1') {
    *paddr = MAX14830_CH1;
  } else if (c == '2') {
    *paddr = MAX14830_CH2;
  } else if (c == '3') {
    *paddr = MAX14830_CH3;
  } else {
    ret = 1;
  }
  #endif

  return ret;
}

void sub_test_a(uint8_t ireg) {

  uint8_t address = MAX14830_CH0;
  uint8_t channel;
  uint8_t reg = MAX14830_REVID_REG;
  uint8_t out = 0;
  char c;
  Serial.println("Sub-test A - Read I2C");

  if(ireg) {
    if(ireg > 0x25) {
      Serial.println("Invalid Register Address");
      return;
    }
    reg = ireg;
  }
  else {
    Serial.print("Input Port (0~3): ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    if(setAddress(c, &address, &channel)) {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);
  }

  #ifdef USE_SC16IS752
  reg = (reg<<3)|(channel<<1);
  #endif
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) {
    out = Wire.read();
  }
  Serial.print("I2C Read: address: "); Serial.print(address, HEX);
  Serial.print(", register: "); Serial.print(reg, HEX);
  Serial.print(", value: "); Serial.println(out, HEX);

}

void sub_test_b(void) {

  uint8_t address = MAX14830_CH0;
  uint8_t channel;
  uint8_t reg = MAX14830_REVID_REG;
  uint8_t val = 0;
  char c;
  Serial.println("Sub-test B - Write I2C");

  Serial.print("Input Port (0~3): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }

  if(setAddress(c, &address, &channel)) {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Serial.print("Input Register Address: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'z')) {
    reg = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    reg |= (c - '0');
  } else if ((c >= 'a') && (c <= 'z')) {
    reg |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Serial.print("Input Value to write: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    val = (c - '0')*0x10;
  } else if ((c >= 'a') && (c <= 'z')) {
    val = (c - 'a' + 0xa)*0x10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    val |= (c - '0');
  } else if ((c >= 'a') && (c <= 'z')) {
    val |= (c - 'a' + 0xa);
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  #ifdef USE_SC16IS752
  reg = (reg<<3)|(channel<<1);
  #endif
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();

  Serial.print("I2C Write: address: "); Serial.print(address, HEX);
  Serial.print(", register: "); Serial.print(reg, HEX);
  Serial.print(", value: "); Serial.println(val, HEX);

}

int32_t sub_test_c_01(uint32_t f, int32_t *bestErr) {

  // Use baudRate 115200 for calculate error
  int32_t err = f % (460800 * 16);
  //int32_t err = f % (38400 * 16);

  if ((*bestErr < 0) || (*bestErr > err)) {
    *bestErr = err;
    return 0;
  }

  return 1;
}

void sub_test_c(void) {

  uint8_t address;
  uint32_t freq = REF_FREQ;
  bool xtal;
  uint32_t div, clksrc, pllcfg = 0;
  int32_t bestErr = -1;
  uint32_t fdiv, fmul, bestfreq = freq;

  Serial.println("Sub-test C - Select Clock Setting");
  // Reset Port

  // First, update error without PLL
  sub_test_c_01(freq, &bestErr);

  // Try all possible PLL dividers
  for (div = 1; (div <= 63) && bestErr; div++) {
    fdiv = DIV_ROUND_CLOSEST(freq, div);
    Serial.print("fdiv: "); Serial.print(fdiv, DEC);
    Serial.print(", freq: "); Serial.print(freq, DEC);
    Serial.print(", div: "); Serial.println(div, DEC);

    // Try multiplier 6
    fmul = fdiv * 6;
    if ((fdiv >= 500000) && (fdiv <= 800000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (0 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 48 
    fmul = fdiv * 48;
    if ((fdiv >= 850000) && (fdiv <= 1200000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (1 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 96
    fmul = fdiv * 96;
    if ((fdiv >= 425000) && (fdiv <= 1000000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (2 << 6) | div;
        bestfreq = fmul;
      }
    // Try multiplier 144
    fmul = fdiv * 144;
    if ((fdiv >= 390000) && (fdiv <= 667000))
      if (!sub_test_c_01(fmul, &bestErr)) {
        pllcfg = (3 << 6) | div;
        bestfreq = fmul;
      }
      
    Serial.print("bestfreq: "); Serial.print(bestfreq, DEC);
    Serial.print(", bestErr: "); Serial.println(bestErr, DEC);
  }
  Serial.print("pllcfg: "); Serial.println(pllcfg, HEX);

}

void sub_test_d(void) {

  uint8_t addr;
  Serial.println("Sub-test D - Access PZEM");

  addr = pzem0.getAddress();

  Serial.print("address: "); Serial.println(addr, HEX);
}

#ifdef PZEM004_SUB_TEST
bool checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}

void setCRC(uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}

static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};

uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}

#endif

void sub_test_e(void) {

#ifdef USE_SC16IS752
  uint8_t addr = SC16IS752_SADDR0;
#else
  uint8_t addr = MAX14830_CH0;
#endif
  uint8_t tbuf[MAX_TX_BUF_SIZE] = {0,};
  uint8_t rbuf[MAX_RX_BUF_SIZE] = {0,};
  uint16_t tlength = 8;
  uint16_t rlength = 8;
  uint8_t index = 0;
  Serial.println("Sub-test E - PZEM Command #1");

  //serial0.fifoClear();
#if 0
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ADDR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
#elif 0
  // read power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x02;
#elif 1
  // read data
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RIR;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tbuf[4] = 0x00;
  tbuf[5] = 0x0a;
  rlength = 25;
#else
  // set power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_WSR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x55;
  tbuf[5] = 0xaa;
#endif
  setCRC(tbuf, tlength);
  
  while(tlength--) {
    Wire.beginTransmission(addr);
    #ifdef USE_SC16IS752
    Wire.write((SC16IS7XX_THR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
    #else
    Wire.write(MAX14830_THR_REG);
    #endif
    Wire.write(tbuf[index]);
    Wire.endTransmission();
    index++;
  }

  delay(600);

  index = 0;
  Wire.beginTransmission(addr);
  #ifdef USE_SC16IS752
  Wire.write((SC16IS7XX_RHR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
  #else
  Wire.write(MAX14830_RHR_REG);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) rlength);

  while(rlength) {
    if (Wire.available()) {
      rbuf[index] = Wire.read();
      Serial.print("rbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(rbuf[index], HEX);
      index++;
      rlength--;
    }
  }

}

void sub_test_f(void) {

#ifdef USE_SC16IS752
  uint8_t addr = SC16IS752_SADDR0;
#else
  uint8_t addr = MAX14830_CH0;
#endif
  uint8_t tbuf[MAX_TX_BUF_SIZE] = {0,};
  uint16_t length = 8;
  uint8_t index = 0;
  char c;
  Serial.println("Sub-test F - writeData");

  Serial.print("Input Transfer Length (1 dec): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length = (c - '0');
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  for(int i=0; i <(length-2); i++){

    Serial.print("Input Buffer Data: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      tbuf[i] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      tbuf[i] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      tbuf[i] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      tbuf[i] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

  }

  setCRC(tbuf, length);
  
  while(length--) {
    Wire.beginTransmission(addr);
    #ifdef USE_SC16IS752
    Wire.write((SC16IS7XX_THR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
    #else
    Wire.write(MAX14830_THR_REG);
    #endif
    Wire.write(tbuf[index]);
    Serial.print("tbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(tbuf[index], HEX);
    Wire.endTransmission();
    index++;
  }

}

void sub_test_g(void) {

  uint8_t addr = MAX14830_CH0;
  uint8_t buf[MAX_RX_BUF_SIZE];
  uint16_t length = 8;
  uint8_t index = 0;
  char c;
  Serial.println("Sub-test G - readData");

  Serial.print("Input Total Length (2 dec): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length = (c - '0')*10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length += (c - '0');
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  Wire.beginTransmission(addr);
  #ifdef USE_SC16IS752
  Wire.write((SC16IS7XX_RHR_REG<<3)|(SC16IS752_CHANNEL_A<<1));
  #else
  Wire.write(MAX14830_RHR_REG);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) length);

  while(length) {
    if (Wire.available()) {
      buf[index] = Wire.read();
      Serial.print("buf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(buf[index], HEX);
      index++;
      length--;
    }
  }

}

void sub_test_h(void) {

  uint8_t addr = MAX14830_CH0;
  uint8_t channel;
  uint8_t tbuf[MAX_TX_BUF_SIZE] = {0,};
  uint8_t rbuf[MAX_RX_BUF_SIZE] = {0,};
  uint16_t tlength = 8;
  uint16_t rlength = 8;
  uint8_t index = 0;
  char c;
  Serial.println("Sub-test H - PZEM Command #2");

  Serial.print("Input Port (0~3): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }

  if(setAddress(c, &addr, &channel)) {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  //serial0.fifoClear();
#if 0
  // reset energy
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_REST;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tlength = 4;
  rlength = 5;
#elif 1
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ADDR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
  rlength = 7;
#elif 0
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RIR;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tbuf[4] = 0x00;
  tbuf[5] = 0x0a;
  rlength = 25;
#else
  // set power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_WSR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x55;
  tbuf[5] = 0xaa;
#endif
  setCRC(tbuf, tlength);
  
  while(tlength--) {
    Wire.beginTransmission(addr);
    #ifdef USE_SC16IS752
    Wire.write((SC16IS7XX_THR_REG<<3)|(channel<<1));
    #else
    Wire.write(MAX14830_THR_REG);
    #endif
    Wire.write(tbuf[index]);
    Serial.print("tbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(tbuf[index], HEX);
    Wire.endTransmission();
    index++;
  }

  //Serial.print("Available right after Tx: "); Serial.println(Wire.available(), DEC);
  delay(200);
  //Serial.print("Line Status Register: "); Serial.println(serial0.readRegister(addr, MAX14830_LSR_IRQSTS_REG), HEX);

  index = 0;
  Wire.beginTransmission(addr);
  #ifdef USE_SC16IS752
  Wire.write((SC16IS7XX_RHR_REG<<3)|(channel<<1));
  #else
  Wire.write(MAX14830_RHR_REG);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) rlength);
  Serial.print("Available right after Request: "); Serial.println(Wire.available(), DEC);

  int rlength_tmp;
  while(rlength) {
    rlength_tmp = Wire.available();
    Serial.print("Available: "); Serial.println(rlength_tmp, DEC);
    if(rlength_tmp) {
    //if (Wire.available()) {
      while(rlength_tmp--) {
        rbuf[index] = Wire.read();
        Serial.print("rbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(rbuf[index], HEX);
        index++;
        rlength--;
        //delay(1);
      }
    }
  }

}

void sub_test_i(uint8_t iaddr) {

  uint8_t addr = MAX14830_CH0;
  uint8_t channel;
  uint8_t tbuf[MAX_TX_BUF_SIZE] = {0,};
  uint8_t rbuf[MAX_RX_BUF_SIZE] = {0,};
  uint16_t tlength = 8;
  uint16_t rlength = 8;
  uint8_t index = 0;
  uint8_t reg = 0;
  char c;
  Serial.println("Sub-test I - PZEM Command #3 (Search)");

  if(iaddr) {
    reg = iaddr;
  }
  else {
    Serial.print("Input Port (0~3): ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }

    if(setAddress(c, &addr, &channel)) {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);
  }

  if(!reg || (reg > 0xf8)) {
    Serial.println("Invalid Reg Address (Should be 0x01~0xf8).");
  }

#if 0
  // reset energy
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_REST;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tlength = 4;
  rlength = 5;
#elif 0
  // read address
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_RHR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ADDR;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
#elif 1
  // read address
  tbuf[0] = reg;
  tbuf[1] = CMD_RIR;
  tbuf[2] = 0x00;
  tbuf[3] = 0x00;
  tbuf[4] = 0x00;
  tbuf[5] = 0x01;
  rlength = 7;
#else
  // set power alarm
  tbuf[0] = 0xf8;
  tbuf[1] = CMD_WSR;
  tbuf[2] = 0x00;
  tbuf[3] = WREG_ALARM_THR;
  tbuf[4] = 0x55;
  tbuf[5] = 0xaa;
#endif
  setCRC(tbuf, tlength);
  
  while(tlength--) {
    Wire.beginTransmission(addr);
    #ifdef USE_SC16IS752
    Wire.write((SC16IS7XX_THR_REG<<3)|(channel<<1));
    #else
    Wire.write(MAX14830_THR_REG);
    #endif
    Wire.write(tbuf[index]);
    Serial.print("tbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(tbuf[index], HEX);
    Wire.endTransmission();
    index++;
  }

  delay(200);

  index = 0;
  Wire.beginTransmission(addr);
  #ifdef USE_SC16IS752
  Wire.write((SC16IS7XX_RHR_REG<<3)|(channel<<1));
  #else
  Wire.write(MAX14830_RHR_REG);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t) rlength);

  while(rlength) {
    if (Wire.available()) {
      rbuf[index] = Wire.read();
      Serial.print("rbuf["); Serial.print(index, DEC); Serial.print("]: "); Serial.println(rbuf[index], HEX);
      index++;
      rlength--;
    }
  }
  if(rbuf[0] != rbuf[1]) Serial.println("Something received =============================================");

}

void sub_test_l(void) {

  uint8_t data[8] = {0,};
  int numBytes;
  char c;
  Serial.println("Sub-test L - RTC");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Read RTC 8 Bytes
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0)); // Access the first register
    sw.endTransmission();

    numBytes = sw.requestFrom(I2C_ADDR_RTC, (uint8_t)8);
    for (int i = 0; i < numBytes; ++i) {
      data[i] = sw.read();
      Serial.print("data["); Serial.print(i, DEC); Serial.print("]: "); Serial.println(data[i], HEX);
    }
    if (numBytes != 8) {
      Serial.print("Read wrong number of bytes: ");
      Serial.println((int)numBytes);
      return;
    }
  } else if (c == '1') {  // Enable RTC Clock
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    sw.write(uint8_t(0));
    sw.endTransmission();
  } else if (c == '2') {  // Disable RTC Clock
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    sw.write(uint8_t(0x80));
    sw.endTransmission();
  } else if (c == '3') {  // Set RTC Time
    data[0] = 0*0x10;   // 10 Seconds
    data[0] += 0;       // Seconds

    data[1] = 5*0x10;   // 10 Minutes
    data[1] += 1;       // Minutes

    //data[2] = 0x40;   // 12-Hour Mode
    data[2] = 0x00;     // 24-Hour Mode

    data[2] += 1*0x10;  // 10 Hours
    data[2] += 3;       // Hours

    data[3] = 5;        // Day (1~7), Monday first

    data[4] = 2*0x10;   // 10 Date
    data[4] += 2;       // Date

    data[5] = 0*0x10;   // 10 Month
    data[5] += 9;       // Month

    data[6] = 2*0x10;   // 10 Year
    data[6] += 3;       // Year
    sw.beginTransmission(I2C_ADDR_RTC);
    sw.write(uint8_t(0));
    for (int i = 0; i < 8; ++i) {
      sw.write(data[i]);
    }
    sw.endTransmission();
  } else if (c == '4') {  // Read Time
    readTime();
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_m(void) {

  uint16_t data;
  uint8_t rdata[2];
  uint8_t val[2];
  int numBytes;
  char c;
  uint8_t reg;
  Serial.println("Sub-test M - IO Expander");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Read IO
    gpio_exp_read(&data);
    Serial.print("data: "); Serial.println(data, HEX);

    for(int i=1; i<9; i++){
      if(get_swtich_val(i) == SWITCH_ON){
        Serial.print("Switch"); Serial.print(i); Serial.println(": ON");
      } else {
        Serial.print("Switch"); Serial.print(i); Serial.println(": OFF");
      }
    }

  } else if(c == '1') {  // LED On/Off
    for(int i=1; i<5; i++){
      led_pm(i, LED_OFF);
    }

    for(int i=1; i<5; i++){
      led_pm(i, LED_ON);
      delay(500);
      led_pm(i, LED_OFF);
      delay(500);
    }

    led_fail(LED_ON);
    delay(500);
    led_fail(LED_OFF);
    delay(500);

    led_tx(LED_ON);
    delay(500);
    led_tx(LED_OFF);
    delay(500);

    led_rx(LED_ON);
    delay(500);
    led_rx(LED_OFF);
    delay(500);

  } else if(c == '2') {  // Data Read
    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    i2c_read_sw(I2C_ADDR_IO, reg, rdata, 2);
    Serial.print("data[0]: "); Serial.println(rdata[0], HEX);
    Serial.print("data[1]: "); Serial.println(rdata[1], HEX);

  } else if(c == '3') {  // Data Write
    Serial.print("Input Register Address: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      reg = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      reg |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      reg |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input Value to write: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[0] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      val[0] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[0] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      val[0] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    Serial.print("Input High Byte Value to write: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      val[1] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      val[1] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      val[1] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

    i2c_write_sw(I2C_ADDR_IO, reg, val, 2);

  } else if(c == '4') {  // Dual Uart LED
    for(int i=2; i<6; i++){
      dual_uart_led_set(i, LED_OFF);
    }

    for(int i=2; i<6; i++){
      dual_uart_led_set(i, LED_ON);
      delay(500);
      dual_uart_led_set(i, LED_OFF);
      delay(500);
    }
  } else if(c == '5') {  // GPIO HIGH
    digitalWrite(PIN_GPIO, HIGH);
  } else if(c == '6') {  // GPIO LOW
    digitalWrite(PIN_GPIO, LOW);
  } else if(c == '7') {  // PCM_Control HIGH
    digitalWrite(PIN_PCM_CONTROL, HIGH);
  } else if(c == '8') {  // PCM_Control LOW
    digitalWrite(PIN_PCM_CONTROL, LOW);
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_n(void) {

  uint8_t data;
  int numBytes;
  char c;
  Serial.println("Sub-test N - W5500");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // 
    // W5500
    pinMode(SS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    digitalWrite(SS, HIGH);
    digitalWrite(SCK, LOW);

    SPI.begin(SCK, MISO, MOSI, SS);
    Ethernet.init(SS);

    Serial.print("w5500: "); Serial.println(w5500, DEC);
    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);

  } else if(c == '1') {  // 
    pinMode(SS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    digitalWrite(SS, HIGH);
    digitalWrite(SCK, LOW);

    SPI.begin(SCK, MISO, MOSI, SS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      SPI.transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    SPI.endTransaction();

  } else if(c == '2') {  // 

    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);

    SPIClass* vspi = new SPIClass(VSPI);
    vspi->begin();
    vspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      vspi->transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    vspi->endTransaction();
    delete vspi;

  } else if(c == '3') {  // 
    Ethernet.init(PIN_ETH_CS);
    pCUR_SPI = pspi;
    Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
    pspi->setFrequency(40000000);
    Ethernet._pinRST = PIN_W5500_RST;
    Ethernet._pinCS = PIN_ETH_CS;
    Ethernet.setHostname("PMS04_001");
    Ethernet.setRetransmissionCount(3);
    Ethernet.setRetransmissionTimeout(4000);

    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);
    Serial.print("localIP(): "); Serial.println(Ethernet.localIP());

  } else if(c == '4') {
    Serial.print("linkStatus: "); Serial.println(Ethernet.linkStatus(), DEC); //LINK_ON, LINK_OFF
    Serial.print("PhyState: "); Serial.println(Ethernet.phyState(), HEX);
    Serial.print("HardwareStatus: "); Serial.println(Ethernet.hardwareStatus());
    Serial.print("speed: "); Serial.println(Ethernet.speed(), DEC);
    Serial.print("duplex: "); Serial.println(Ethernet.duplex(), DEC);
    Serial.print("dnsServerIP: "); Serial.println(Ethernet.dnsServerIP());

  } else if(c == '5') {
    unsigned int localPort = 1883;
    char packetBuffer[255];
    int packetSize;
    int len;
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    while(1) {
      packetSize = Udp.parsePacket();

      if (packetSize)
      {
        Serial.print(F("Received packet of size "));
        Serial.println(packetSize);
        Serial.print(F("From "));
        IPAddress remoteIp = Udp.remoteIP();
        Serial.print(remoteIp);
        Serial.print(F(", port "));
        Serial.println(Udp.remotePort());

        // read the packet into packetBufffer
        len = Udp.read(packetBuffer, 255);

        if (len > 0)
        {
          packetBuffer[len] = 0;
        }

        Serial.println(F("Contents:"));
        Serial.println(packetBuffer);
      }

      if(Serial.available()) {
        c = Serial.read();
        if(c == 'q') {
          Serial.println("Exit Udp Receive");
          break;
        }
      }
      delay(100);
      //Serial.print(".");
    }
  } else if(c == '6') {
    unsigned int localPort = 8080;
    unsigned int serverPort = 1883;
    int ret;
    size_t wsize;
    IPAddress server_ip(192, 168, 1, 149);
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    ret = Udp.beginPacket(server_ip, serverPort);
    Serial.print("Return of beginPacket: "); Serial.println(ret, DEC);
    wsize = Udp.write("hello from esp");
    Serial.print("Return of write: "); Serial.println(wsize);
    ret = Udp.endPacket();
    Serial.print("Return of endPacket: "); Serial.println(ret, DEC);

  } else if(c == '7') {
    //DhcpClass* dhcp = new DhcpClass();
    dhcp->beginWithDHCP(nc_mac);
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
    //delete dhcp;
  } else if(c == '8') {
    Serial.print("DhcpServerIp(): "); Serial.println(dhcp->getDhcpServerIp());
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
  } else if(c == '9') {
    //Ethernet.WoL(1);
    //Serial.print("WoL: "); Serial.println(Ethernet.WoL(), DEC);
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_o(void) {

  uint8_t uport = 1;
  uint8_t dtype = 0;
  char c;
  uint8_t data = 0;
  Serial.println("Sub-test O - UART TX");

  Serial.print("Select Port (1:RS232, 2:RS485): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    setRS485Dir(MAX485_DIR_SEND);
  } else {
    Serial.println("Invalid port number");
    return;
  }
/*
  Serial.print("Select Data Type (h:hex, c:char): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);
  c = tolower(c);

  if(c == 'h') {
    dtype = 0;
  } else if(c == 'c') {
    dtype = 1;
  } else {
    Serial.println("Invalid data type");
    return;
  }
*/
  while(1) {
    Serial.println("Input data: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    if(!isalnum(c)){
      Serial.println("Quit data input");
      break;
    }
    Serial.println(c);
    
    if(uport == 1) {
      Serial1.write(c);
    } else if(uport == 2) {
      Serial2.write(c);
    }

  }

}

void sub_test_p(void) {

  uint8_t uport = 1;
  uint8_t dtype = 0;
  char c;
  uint8_t data[128] = {0,};
  uint16_t length, rsize;
  Serial.println("Sub-test P - UART RX");

  Serial.print("Select Port (1:RS232, 2:RS485): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    setRS485Dir(MAX485_DIR_RECEIVE);
  } else {
    Serial.println("Invalid port number");
    return;
  }
/*
  Serial.print("Select Data Type (h:hex, c:char): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);
  c = tolower(c);

  if(c == 'h') {
    dtype = 0;
  } else if(c == 'c') {
    dtype = 1;
  } else {
    Serial.println("Invalid data type");
    return;
  }
*/
  Serial.println("Received Data: ");

  while(1) {
    if(uport == 1) {
      length = Serial1.available();
      if(length) {
        if(length > 128) length = 128;
      }
      rsize = Serial1.read(data, length);
      for(int i=0; i < (int) rsize; i++) {
        Serial.println((char) data[i]);
      }
    } else if(uport == 2) {
      length = Serial2.available();
      if(length) {
        if(length > 128) length = 128;
      }
      rsize = Serial2.read(data, length);
      for(int i=0; i < (int) rsize; i++) {
        Serial.println((char) data[i]);
      }
    }

    if(Serial.available()) {
      c = Serial.read();
      if(c == 'q') {
        Serial.println("Exit Data Receive");
        break;
      }
    }
    delay(100);
  }

}

void sub_test_q(void) {

  char c;
  /*
   * Set up the data to be sent.
   * For most protocols, the data is build up with a constant 8 (or 16 byte) address
   * and a variable 8 bit command.
   * There are exceptions like Sony and Denon, which have 5 bit address.
  */
  uint8_t sCommand = 0x34;
  uint8_t sRepeats = 0;

  Serial.println("Sub-test Q - IR");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Send IR
    Serial.println();
    Serial.print(F("Send now: address=0x00, command=0x"));
    Serial.print(sCommand, HEX);
    Serial.print(F(", repeats="));
    Serial.print(sRepeats);
    Serial.println();

    Serial.println(F("Send standard NEC with 8 bit address"));
    Serial.flush();

    // Receiver output for the first loop must be: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    IrSender.sendNEC(0x00, sCommand, sRepeats);
#if 0
    /*
     * Increment send values
     */
    sCommand += 0x11;
    sRepeats++;
    // clip repeats at 4
    if (sRepeats > 4) {
        sRepeats = 4;
    }
#endif
    delay(100);  // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_y(void) {
  uint8_t buf[MAX_TX_BUF_SIZE] = {0,};
  uint8_t length = 0;
  char c;
  Serial.println("Sub-test Y - CRC check");

  Serial.print("Input Total Length (2 dec): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length = (c - '0')*10;
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.print(c);
  
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  c = tolower(c);
  if((c >= '0') && (c <= '9')) {
    length += (c - '0');
  } else {
    Serial.println("Invalid input");
    return;
  }
  Serial.println(c);

  for(int i=0; i <(length-2); i++){

    Serial.print("Input Buffer Data: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      buf[i] = (c - '0')*0x10;
    } else if ((c >= 'a') && (c <= 'z')) {
      buf[i] = (c - 'a' + 0xa)*0x10;
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.print(c);
  
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        break;
      }
      delay(100);
    }
    c = tolower(c);
    if((c >= '0') && (c <= '9')) {
      buf[i] |= (c - '0');
    } else if ((c >= 'a') && (c <= 'z')) {
      buf[i] |= (c - 'a' + 0xa);
    } else {
      Serial.println("Invalid input");
      return;
    }
    Serial.println(c);

  }

  setCRC(buf, length);

  Serial.print("CRC_H: "); Serial.println(buf[length-2], HEX);
  Serial.print("CRC_L: "); Serial.println(buf[length-1], HEX);

}

void sub_test_z(void) {

  char c;
  int idx=0;
  uint8_t addr;
  PZEM004Tv30* ppzem[4];

  ppzem[0] = &pzem0;
  ppzem[1] = &pzem1;
  ppzem[2] = &pzem2;
  ppzem[3] = &pzem3;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      Serial.println(c);
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }

    addr = ppzem[idx]->readAddress();
    Serial.print("Custom Address["); Serial.print(idx); Serial.print("]: ");
    Serial.println(addr, HEX);

    if(addr) {

      led_pm(idx+1, LED_ON);

      // Read the data from the sensor
      float voltage = ppzem[idx]->voltage();
      float current = ppzem[idx]->current();
      float power = ppzem[idx]->power();
      float energy = ppzem[idx]->energy();
      float frequency = ppzem[idx]->frequency();
      float pf = ppzem[idx]->pf();

      // Check if the data is valid
      if(isnan(voltage)){
          Serial.println("Error reading voltage");
      } else if (isnan(current)) {
          Serial.println("Error reading current");
      } else if (isnan(power)) {
          Serial.println("Error reading power");
      } else if (isnan(energy)) {
          Serial.println("Error reading energy");
      } else if (isnan(frequency)) {
          Serial.println("Error reading frequency");
      } else if (isnan(pf)) {
          Serial.println("Error reading power factor");
      } else {

        // Print the values to the Serial console
        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pf);

      }
      led_pm(idx+1, LED_OFF);
    }
    else{
      led_pm(idx+1, LED_OFF);
    }

    Serial.println();

    if(++idx > 3) idx = 0;
    delay(1000);
  }

}