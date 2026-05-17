#pragma once
#include <Arduino.h>
#include <RadioLib.h>
#include "node_config.h"
#include "lora_protocol.h"

#define TX_WATCHDOG_MS 5000UL

typedef enum { MODE_RX, MODE_TX } RadioMode;

// Wraps RadioLib: init, CAD, blocking TX, RX read, watchdog.
// The radio object itself (SX1262/SX1278/LLCC68) lives as a file-scope
// static in radio_driver.cpp — only one instance should ever exist.
class RadioDriver {
public:
    bool begin();
    void tx(const uint8_t *buf, uint8_t len);
    bool readRx(uint8_t *outBuf, uint8_t *outLen, int16_t *rssi, float *snr);
    void tickWatchdog();
    bool isTxBusy() const { return _txBusy; }

    static void LORA_IRQ_ATTR onIsr();  // attached to DIO1 interrupt
    static volatile bool rxDone;
    static volatile bool txDone;

private:
    bool _cadWaitClear();

    static volatile RadioMode _mode;
    bool     _txBusy    = false;
    uint32_t _txStartMs = 0;
};
