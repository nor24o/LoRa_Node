#include "radio_driver.h"
#include "debug.h"

// ── Radio object — type selected at compile time ───────────────────────────
// File-scope statics: no heap, constructed before setup(), safe for ISR.
#if defined(RADIO_SX1262)
  static Module g_mod(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);
  static SX1262 g_radio(&g_mod);
#elif defined(RADIO_SX1278)
  static Module g_mod(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_DIO2_PIN);
  static SX1278 g_radio(&g_mod);
#elif defined(RADIO_LLCC68)
  static Module g_mod(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);
  static LLCC68 g_radio(&g_mod);
#else
  #error "No radio type defined. Set RADIO_SX1262, RADIO_SX1278, or RADIO_LLCC68 in node_config.h"
#endif

// ── Static member definitions ─────────────────────────────────────────────
volatile RadioMode RadioDriver::_mode  = MODE_RX;
volatile bool      RadioDriver::rxDone = false;
volatile bool      RadioDriver::txDone = false;

// ── LED helpers ───────────────────────────────────────────────────────────
#if LED_ENABLED
static void led_tx() {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(LED_TX_ON_MS);
    digitalWrite(LED_STATUS_PIN, LOW);
}
#else
static void led_tx() {}
#endif

// ── ISR — keep minimal, no SPI/RadioLib calls ─────────────────────────────
void LORA_IRQ_ATTR RadioDriver::onIsr() {
    if (_mode == MODE_TX) txDone = true;
    else                  rxDone = true;
}

// ── begin ─────────────────────────────────────────────────────────────────
bool RadioDriver::begin() {
    int err;

#if defined(RADIO_SX1262) || defined(RADIO_LLCC68)
    err = g_radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                        LORA_SYNC_WORD, LORA_TX_POWER_DBM, LORA_PREAMBLE);
    if (err == RADIOLIB_ERR_NONE) {
        g_radio.setCurrentLimit(140.0f);
        // g_radio.setDio2AsRfSwitch(true);  // uncomment if module uses DIO2 for RF switch
        // g_radio.setTCXO(1.8f);            // uncomment if module has a TCXO
    }
#elif defined(RADIO_SX1278)
    // SX127x private-network sync word is always 0x12
    err = g_radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                        0x12, LORA_TX_POWER_DBM, LORA_PREAMBLE);
#endif

    if (err != RADIOLIB_ERR_NONE) {
        DBG("[RADIO] INIT FAILED: error %d\n", err);
        return false;
    }

    g_radio.setDio1Action(RadioDriver::onIsr);
    _mode = MODE_RX;
    g_radio.startReceive();
    DBGLN("[RADIO] OK — RX armed");
    return true;
}

// ── _cadWaitClear — CSMA backoff before TX ────────────────────────────────
bool RadioDriver::_cadWaitClear() {
#if !defined(RADIO_SX1278)
    uint32_t deadline = millis() + CAD_TIMEOUT_MS;
    uint8_t  attempts = 0;
    while (millis() < deadline) {
        int result = g_radio.scanChannel();
        if (result == RADIOLIB_ERR_NONE)      return true;
        if (result == RADIOLIB_LORA_DETECTED) {
            attempts++;
            uint32_t backoff = 20UL << (attempts < 4 ? attempts : 4);
            backoff += (uint32_t)(random(backoff / 2));
            DBG("[CAD] busy (attempt %d) — backoff %lu ms\n", attempts, backoff);
            delay(backoff);
            continue;
        }
        DBG("[CAD] scanChannel err %d — skipping CAD\n", result);
        return true;
    }
    DBG("[CAD] timeout — forcing TX\n");
#endif
    return true;
}

// ── tx — CAD + blocking startTransmit + re-arm RX ────────────────────────
void RadioDriver::tx(const uint8_t *buf, uint8_t len) {
    if (_txBusy) { DBGLN("[TX] busy — skipping"); return; }

    _cadWaitClear();

    txDone      = false;
    rxDone      = false;
    _mode       = MODE_TX;
    _txBusy     = true;
    _txStartMs  = millis();

    int err = g_radio.startTransmit((uint8_t *)buf, len);
    if (err != RADIOLIB_ERR_NONE) {
        DBG("[TX] startTransmit error %d\n", err);
        _mode   = MODE_RX;
        _txBusy = false;
        g_radio.startReceive();
        return;
    }

    uint32_t t0 = millis();
    while (!txDone && (millis() - t0 < TX_WATCHDOG_MS)) { /* spin */ }

    if (!txDone) {
        DBGLN("[TX] watchdog timeout — standby");
        g_radio.standby();
    }

    txDone  = false;
    _txBusy = false;
    _mode   = MODE_RX;

    led_tx();
    DBG("[TX] %d bytes  seq=%d\n", len, (buf[3] >> 2) & 0x3F);
    g_radio.startReceive();
}

// ── readRx — drain FIFO, re-arm receiver, return raw bytes ───────────────
bool RadioDriver::readRx(uint8_t *outBuf, uint8_t *outLen,
                          int16_t *rssi, float *snr) {
    rxDone = false;

    int state = g_radio.readData(outBuf, 0);   // 0 = read all available
    *rssi = (int16_t)g_radio.getRSSI();
    *snr  = g_radio.getSNR();

    _mode = MODE_RX;
    g_radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        DBG("[RX] readData error %d\n", state);
        return false;
    }
    *outLen = (uint8_t)g_radio.getPacketLength(false);
    return true;
}

// ── tickWatchdog — defensive: recover from stuck TX ──────────────────────
void RadioDriver::tickWatchdog() {
    if (_txBusy && (millis() - _txStartMs) > TX_WATCHDOG_MS) {
        DBGLN("[TX] watchdog — forcing standby");
        g_radio.standby();
        _txBusy = false;
        txDone  = false;
        _mode   = MODE_RX;
        g_radio.startReceive();
    }
}
