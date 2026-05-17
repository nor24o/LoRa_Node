#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  node_config.h  —  THE ONLY FILE YOU EDIT PER PHYSICAL NODE
//
//  Steps:
//    1.  Set NODE_ADDR (unique per device, 0x01–0xFE)
//    2.  Set PEER_ADDRS / PEER_COUNT
//    3.  Verify your pin mapping matches your actual wiring
//    4.  Choose RADIO_SX1262, RADIO_SX1278, or RADIO_LLCC68
//    5.  Set LORA_FREQ_MHZ for your region (868.1 EU / 915.0 US)
//    6.  Map IO pins to -1 to disable unused channels
//    7.  Customise MEAS_x_SCALE macros for your actual sensors
// ═══════════════════════════════════════════════════════════════════════════


// ── Node identity ────────────────────────────────────────────────────────
#define NODE_ADDR           0x01    // ← CHANGE for each device (0x01–0xFE)
#define NODE_FW_MAJOR       1       // firmware version reported in beacon
#define NODE_FW_MINOR       0


// ── Peer list ────────────────────────────────────────────────────────────
//  List the addresses of nodes this device talks to directly (peer-to-peer).
//  The gateway (0x00) is always implicitly reachable — do not list it here.
//  Set PEER_COUNT to match the number of entries in PEER_ADDRS.
#define PEER_ADDRS          { 0x02, 0x03 }
#define PEER_COUNT          2


// ═══════════════════════════════════════════════════════════════════════════
//  RADIO MODULE TYPE  —  uncomment exactly ONE
// ═══════════════════════════════════════════════════════════════════════════
#define RADIO_SX1262        // SX1261 / SX1262 / SX1268  (has BUSY pin)
//#define RADIO_SX1278      // SX1276 / SX1277 / SX1278 / SX1279  (no BUSY)
//#define RADIO_LLCC68      // LLCC68  (same wiring as SX1262, max SF=10)


// ═══════════════════════════════════════════════════════════════════════════
//  PIN MAPPING  —  auto-selected by board define from platformio.ini
//  Override any pin by editing the value directly below your board section.
// ═══════════════════════════════════════════════════════════════════════════

// ────────────────────────────────────────────────────────────────────────
//  ESP32 / ESP32-S2 / ESP32-S3  +  SX1262
//  The ESP32 Arduino framework defines ESP32 automatically.
//  Do NOT add -DESP32 to platformio.ini — it causes "redefined" warnings.
// ────────────────────────────────────────────────────────────────────────
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

  #define LORA_CS_PIN       5      // GPIO connected to NSS/CS
  #define LORA_DIO1_PIN     27     // GPIO connected to DIO1 (IRQ line)
  #define LORA_RST_PIN      14     // GPIO connected to RST/NRESET
  #define LORA_BUSY_PIN     26     // GPIO connected to BUSY  (SX126x only)
  #define LORA_DIO2_PIN     -1     // DIO2 used for RF-switch internally — leave -1
  #define LORA_IRQ_ATTR     IRAM_ATTR   // ISR must live in IRAM on ESP32

  // IO pins (digital inputs)
  #define DIN_0_PIN         4
  #define DIN_1_PIN         16
  #define DIN_2_PIN         17
  #define DIN_3_PIN         -1
  #define DIN_4_PIN         -1
  #define DIN_5_PIN         -1
  #define DIN_6_PIN         -1
  #define DIN_7_PIN         -1

  // IO pins (digital outputs)
  #define DOUT_0_PIN        18
  #define DOUT_1_PIN        19
  #define DOUT_2_PIN        21
  #define DOUT_3_PIN        22
  #define DOUT_4_PIN        -1
  #define DOUT_5_PIN        -1
  #define DOUT_6_PIN        -1
  #define DOUT_7_PIN        -1

  // Analog input pins (ESP32: ADC1 only — avoid ADC2 when WiFi is on)
  #define AIN_0_PIN         34     // ADC1_CH6  — input only, no pullup
  #define AIN_1_PIN         35     // ADC1_CH7  — input only, no pullup
  #define AIN_2_PIN         32     // ADC1_CH4
  #define AIN_3_PIN         33     // ADC1_CH5

  #define ADC_VREF_MV       3300
  #define ADC_BITS          12     // ESP32 ADC is 12-bit

  #define LED_STATUS_PIN    2      // built-in LED (active HIGH on most boards)

// ────────────────────────────────────────────────────────────────────────
//  ESP8266  (NodeMCU / Wemos D1 mini)  +  SX1262
//  The ESP8266 framework defines ESP8266 automatically.
// ────────────────────────────────────────────────────────────────────────
#elif defined(ESP8266) || defined(ARDUINO_ARCH_ESP8266)

  #define LORA_CS_PIN       15     // D8
  #define LORA_DIO1_PIN     5      // D1  (any GPIO supports interrupts on ESP8266)
  #define LORA_RST_PIN      0      // D3
  #define LORA_BUSY_PIN     4      // D2
  #define LORA_DIO2_PIN     -1
  #define LORA_IRQ_ATTR     ICACHE_RAM_ATTR

  // IO pins
  #define DIN_0_PIN         12     // D6
  #define DIN_1_PIN         13     // D7
  #define DIN_2_PIN         -1
  #define DIN_3_PIN         -1
  #define DIN_4_PIN         -1
  #define DIN_5_PIN         -1
  #define DIN_6_PIN         -1
  #define DIN_7_PIN         -1

  #define DOUT_0_PIN        14     // D5
  #define DOUT_1_PIN        16     // D0
  #define DOUT_2_PIN        -1
  #define DOUT_3_PIN        -1
  #define DOUT_4_PIN        -1
  #define DOUT_5_PIN        -1
  #define DOUT_6_PIN        -1
  #define DOUT_7_PIN        -1

  // ESP8266 has one ADC pin only
  #define AIN_0_PIN         A0
  #define AIN_1_PIN         -1
  #define AIN_2_PIN         -1
  #define AIN_3_PIN         -1

  #define ADC_VREF_MV       1000   // NodeMCU divider: 0–1 V into A0
  #define ADC_BITS          10

  #define LED_STATUS_PIN    2      // active LOW on NodeMCU

// ────────────────────────────────────────────────────────────────────────
//  Arduino Uno / Nano  +  SX1278  (SX127x — no BUSY pin)
//  Warning: Uno has only 2 kB RAM. Disable DEBUG_ENABLED to save ~400 B.
// ────────────────────────────────────────────────────────────────────────
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)

  #define LORA_CS_PIN       10
  #define LORA_DIO1_PIN     2      // MUST be pin 2 or 3 on Uno/Nano
  #define LORA_RST_PIN      9
  #define LORA_BUSY_PIN     -1     // SX127x has no BUSY pin
  #define LORA_DIO2_PIN     3      // optional DIO2 for RX timeout
  #define LORA_IRQ_ATTR              // no attribute needed on AVR

  #define DIN_0_PIN         4
  #define DIN_1_PIN         5
  #define DIN_2_PIN         -1
  #define DIN_3_PIN         -1
  #define DIN_4_PIN         -1
  #define DIN_5_PIN         -1
  #define DIN_6_PIN         -1
  #define DIN_7_PIN         -1

  #define DOUT_0_PIN        6
  #define DOUT_1_PIN        7
  #define DOUT_2_PIN        8
  #define DOUT_3_PIN        -1
  #define DOUT_4_PIN        -1
  #define DOUT_5_PIN        -1
  #define DOUT_6_PIN        -1
  #define DOUT_7_PIN        -1

  #define AIN_0_PIN         A0
  #define AIN_1_PIN         A1
  #define AIN_2_PIN         A2
  #define AIN_3_PIN         A3

  #define ADC_VREF_MV       5000   // 5V Uno; change to 3300 for 3.3V MCU
  #define ADC_BITS          10

  #define LED_STATUS_PIN    LED_BUILTIN

// ────────────────────────────────────────────────────────────────────────
//  Arduino Mega 2560  +  SX1278
// ────────────────────────────────────────────────────────────────────────
#elif defined(ARDUINO_AVR_MEGA2560)

  #define LORA_CS_PIN       53
  #define LORA_DIO1_PIN     2      // pin 2 or 3 for hw interrupt
  #define LORA_RST_PIN      49
  #define LORA_BUSY_PIN     -1
  #define LORA_DIO2_PIN     3
  #define LORA_IRQ_ATTR

  #define DIN_0_PIN         22
  #define DIN_1_PIN         23
  #define DIN_2_PIN         24
  #define DIN_3_PIN         25
  #define DIN_4_PIN         -1
  #define DIN_5_PIN         -1
  #define DIN_6_PIN         -1
  #define DIN_7_PIN         -1

  #define DOUT_0_PIN        26
  #define DOUT_1_PIN        27
  #define DOUT_2_PIN        28
  #define DOUT_3_PIN        29
  #define DOUT_4_PIN        -1
  #define DOUT_5_PIN        -1
  #define DOUT_6_PIN        -1
  #define DOUT_7_PIN        -1

  #define AIN_0_PIN         A0
  #define AIN_1_PIN         A1
  #define AIN_2_PIN         A2
  #define AIN_3_PIN         A3

  #define ADC_VREF_MV       5000
  #define ADC_BITS          10

  #define LED_STATUS_PIN    LED_BUILTIN

// ────────────────────────────────────────────────────────────────────────
//  STM32  (Arduino core via STM32duino)  +  SX1262
//  Tested on Nucleo-F401RE, Blue Pill F103C8, Black Pill F411CE
// ────────────────────────────────────────────────────────────────────────
#elif defined(STM32) || defined(ARDUINO_ARCH_STM32)

  #define LORA_CS_PIN       PA4    // SPI1 NSS
  #define LORA_DIO1_PIN     PB0    // any GPIO, STM32 EXTI on any pin
  #define LORA_RST_PIN      PB1
  #define LORA_BUSY_PIN     PB2
  #define LORA_DIO2_PIN     -1
  #define LORA_IRQ_ATTR

  #define DIN_0_PIN         PA0
  #define DIN_1_PIN         PA1
  #define DIN_2_PIN         PA2
  #define DIN_3_PIN         PA3
  #define DIN_4_PIN         -1
  #define DIN_5_PIN         -1
  #define DIN_6_PIN         -1
  #define DIN_7_PIN         -1

  #define DOUT_0_PIN        PB_10
  #define DOUT_1_PIN        PB_11
  #define DOUT_2_PIN        PB_12
  #define DOUT_3_PIN        PB_13
  #define DOUT_4_PIN        -1
  #define DOUT_5_PIN        -1
  #define DOUT_6_PIN        -1
  #define DOUT_7_PIN        -1

  #define AIN_0_PIN         PA5
  #define AIN_1_PIN         PA6
  #define AIN_2_PIN         PA7
  #define AIN_3_PIN         -1

  #define ADC_VREF_MV       3300
  #define ADC_BITS          12

  #define LED_STATUS_PIN    PC13   // active LOW on Blue Pill

#else
  #error "Board not recognised — add a pin block for your MCU in node_config.h"
#endif


// ═══════════════════════════════════════════════════════════════════════════
//  LoRa RF parameters  —  all nodes in a network MUST share these values
// ═══════════════════════════════════════════════════════════════════════════
#define LORA_FREQ_MHZ       868.1f  // MHz  — 868.1 EU  /  915.0 US  /  433.0 AS
#define LORA_BW_KHZ         125.0f  // kHz  — 125 (range) / 250 / 500 (speed)
#define LORA_SF             9       // spreading factor 7–12 (LLCC68 max=10)
#define LORA_CR             7       // coding rate denominator 5–8  → 4/x
#define LORA_SYNC_WORD      0xAB    // 1 byte for SX126x; 0x12 for SX127x*
//  *For SX127x, the sync word is hardcoded to 0x12 in main.cpp.
//   This value is only used for SX126x / LLCC68.
#define LORA_TX_POWER_DBM   14      // dBm  2–22 for SX1262; 2–17 for SX1278
#define LORA_PREAMBLE       8       // symbols (minimum 8 recommended)

//  LoRa parameter guide:
//  SF7  BW125 → ~1–2 km  air time ~28 ms   fast, short range
//  SF9  BW125 → ~3–5 km  air time ~165 ms  ← recommended default
//  SF10 BW125 → ~5–8 km  air time ~330 ms  good range, moderate speed
//  SF12 BW125 → 10–15 km air time ~1.3 s   maximum range, slowest


// ═══════════════════════════════════════════════════════════════════════════
//  Measurement sensor scaling
//  raw_mv is the ADC reading converted to millivolts.
//  Return int16 = real_value × 10  (e.g. 25.4 °C → 254)
//  Replace these with your actual sensor transfer functions.
// ═══════════════════════════════════════════════════════════════════════════

//  Channel 0: temperature example — LM35 output 10 mV/°C, 0 offset
//  LM35 at 25°C → 250 mV → returns 250 → decoded as 25.0 °C
#define MEAS_0_SCALE(mv)    ((int16_t)((mv) * 1))

//  Channel 1: 0–5V level sensor scaled to 0–100.0%
//  Full scale 3300 mV → 100.0% → returns 1000 → decoded as 100.0
#define MEAS_1_SCALE(mv)    ((int16_t)((mv) * 1000L / ADC_VREF_MV))

//  Channel 2: generic 0–3.3V → 0–330.0 arbitrary units (×10)
#define MEAS_2_SCALE(mv)    ((int16_t)((mv) * 10L / 100))

//  Channel 3: disabled — always returns 0
#define MEAS_3_SCALE(mv)    ((int16_t)(0))


// ═══════════════════════════════════════════════════════════════════════════
//  IO behaviour
// ═══════════════════════════════════════════════════════════════════════════
//  INPUT_MODE: INPUT_PULLUP (default), INPUT, or INPUT_PULLDOWN
#define DIN_INPUT_MODE      INPUT_PULLUP

//  When INPUT_PULLUP is used, the pin reads LOW when asserted (active-low).
//  Set to true to invert the logic so bit=1 means the input is active.
#define DIN_INVERT_LOGIC    true

//  Output safe state on startup (applied before radio init)
#define DOUT_STARTUP_STATE  0x00    // all outputs OFF at boot


// ═══════════════════════════════════════════════════════════════════════════
//  Timing
// ═══════════════════════════════════════════════════════════════════════════
//  Full report (PKT_FULL) to gateway — every N milliseconds
#define REPORT_INTERVAL_MS      30000UL

//  Beacon broadcast — every N milliseconds
#define BEACON_INTERVAL_MS      60000UL

//  How long to wait for an ACK before counting as lost
#define ACK_TIMEOUT_MS          2000UL

//  How many times to retransmit before giving up (0 = no retry)
#define ACK_RETRY_COUNT         3

//  Extra delay added per retry attempt (exponential: retry × this)
#define ACK_RETRY_BACKOFF_MS    500UL

//  How long CAD is allowed to keep trying before forcing a TX
#define CAD_TIMEOUT_MS          800UL

//  Uptime counter: how often to update the uptime_min counter (ms)
#define UPTIME_TICK_MS          60000UL

//  IO change debounce: ignore further changes within this window (ms)
#define IO_DEBOUNCE_MS          50UL

//  Minimum interval between IO-change peer notifications (ms)
//  Prevents flooding if an input bounces repeatedly
#define IO_NOTIFY_COOLDOWN_MS   500UL


// ═══════════════════════════════════════════════════════════════════════════
//  Status LED
//  Blink patterns communicate state without a serial monitor.
// ═══════════════════════════════════════════════════════════════════════════
#define LED_ENABLED             true
//  Blink count on successful radio init
#define LED_BLINK_INIT          3
//  Blink on TX (ms on, ms off)
#define LED_TX_ON_MS            30
//  Blink on RX
#define LED_RX_ON_MS            15
//  Fast blink rate when radio init fails (stays in error loop)
#define LED_ERROR_BLINK_MS      200


// ═══════════════════════════════════════════════════════════════════════════
//  Debug serial output
//  Set DEBUG_ENABLED to false for production builds (saves ~2 kB flash, ~400 B RAM)
// ═══════════════════════════════════════════════════════════════════════════
#ifndef DEBUG_ENABLED
  #define DEBUG_ENABLED         true
#endif
#define DEBUG_SERIAL            Serial
#define DEBUG_BAUD              115200