

#define DEBUG_PORT              Serial      // Serial debug port
#define SERIAL_BAUD             115200      // Serial debug baud rate

#ifdef DEBUG_PORT
#define DEBUG_MSG(...) DEBUG_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------

#define I2C_SDA         21
#define I2C_SCL         22
#define LED_PIN         14
#define RELAIS_PIN         14   // Works with TTGO LoRa32 V2.1 board (confirmed)
#define BUTTON_PIN      39

// -----------------------------------------------------------------------------
// OLED
// -----------------------------------------------------------------------------

#define SSD1306_ADDRESS 0x3C

// -----------------------------------------------------------------------------
// T, Pressure, Humidity
// -----------------------------------------------------------------------------

#define BME280_ADDRESS_0x76 0x76
#define BME280_ADDRESS_0x77 0x77
#define SPL06_ADDRESS_0x76 0x76
#define SPL06_ADDRESS_0x77 0x77
#define SHT21_ADDRESS 0x40

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

#define GPS_BAUDRATE    9600
#define USE_GPS         1
#define ss              Serial2 // define GPSserial as ESP32 Serial2
#define LMIC_WAIT       3000 // The lmic library requires minimum of 2.7 seconds to complete RX2 (maybe more if SF10 to SF12)
#define GPS_WAIT        1900 // Should be longer than 1 second (default on GPS)

// Options on ESP32 are:
//   RX: 3   TX: 1    == Serial0
//   RX: 9   TX: 10   == Serial1
//   RX: 16  TX: 17   == Serial2

#define GPS_RX_PIN      1
#define GPS_TX_PIN      3
#define GPS_POWER_PIN   AXP192_LDO3

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO        5
#define MISO_GPIO       19
#define MOSI_GPIO       27
#define NSS_GPIO        18
#define RESET_GPIO      23
#define DIO0_GPIO       26
#define DIO1_GPIO       33
#define DIO2_GPIO       32

#define CLOCK_ERROR             1

// -----------------------------------------------------------------------------
// AXP192 (Rev1-specific options)
// -----------------------------------------------------------------------------

#define AXP192_SLAVE_ADDRESS  0x34
#define GPS_POWER_CTRL_CH     3
#define LORA_POWER_CTRL_CH    2
#define PMU_IRQ               35
