#include "io_manager.h"
#include "debug.h"

const int16_t IoManager::_dinPins [8] = { DIN_0_PIN,  DIN_1_PIN,  DIN_2_PIN,  DIN_3_PIN,
                                           DIN_4_PIN,  DIN_5_PIN,  DIN_6_PIN,  DIN_7_PIN };
const int16_t IoManager::_doutPins[8] = { DOUT_0_PIN, DOUT_1_PIN, DOUT_2_PIN, DOUT_3_PIN,
                                           DOUT_4_PIN, DOUT_5_PIN, DOUT_6_PIN, DOUT_7_PIN };
const int16_t IoManager::_ainPins [4] = { AIN_0_PIN,  AIN_1_PIN,  AIN_2_PIN,  AIN_3_PIN };

void IoManager::begin() {
    for (int i = 0; i < 8; i++) {
        if (_dinPins[i] >= 0)
            pinMode((uint8_t)_dinPins[i], DIN_INPUT_MODE);
        if (_doutPins[i] >= 0) {
            pinMode((uint8_t)_doutPins[i], OUTPUT);
            bool on = (DOUT_STARTUP_STATE >> i) & 0x01;
            digitalWrite((uint8_t)_doutPins[i], on ? HIGH : LOW);
            if (on) _out |= (1u << i);
        }
    }
}

uint8_t IoManager::_readRaw() const {
    uint8_t state = 0;
    for (int i = 0; i < 8; i++) {
        if (_dinPins[i] < 0) continue;
        bool level = digitalRead((uint8_t)_dinPins[i]);
#if DIN_INVERT_LOGIC
        if (!level) state |= (1u << i);
#else
        if ( level) state |= (1u << i);
#endif
    }
    return state;
}

bool IoManager::pollDebounce() {
    uint8_t raw = _readRaw();
    if (raw != _raw) {
        _raw        = raw;
        _debounceMs = millis();
        return false;
    }
    if (raw != _in && (millis() - _debounceMs) >= IO_DEBOUNCE_MS) {
        _in = raw;
        return true;
    }
    return false;
}

void IoManager::applyCmd(uint8_t mask, uint8_t value) {
    for (int i = 0; i < 8; i++) {
        if (!(mask & (1u << i))) continue;
        if (_doutPins[i] < 0)   continue;
        bool on = (value >> i) & 0x01;
        digitalWrite((uint8_t)_doutPins[i], on ? HIGH : LOW);
        if (on) _out |=  (uint8_t)(1u << i);
        else    _out &= ~(uint8_t)(1u << i);
    }
    DBG("[IO] mask=0x%02X val=0x%02X → out=0x%02X\n", mask, value, _out);
}

uint8_t IoManager::combinedState() const {
    return (uint8_t)((_out & 0xF0) | (_in & 0x0F));
}

int16_t IoManager::measRead(uint8_t ch) const {
    if (ch >= 4 || _ainPins[ch] < 0) return 0;
    int raw = analogRead((uint8_t)_ainPins[ch]);
    int mv  = (int)(((long)raw * ADC_VREF_MV) / ((1 << ADC_BITS) - 1));
    switch (ch) {
        case 0: return MEAS_0_SCALE(mv);
        case 1: return MEAS_1_SCALE(mv);
        case 2: return MEAS_2_SCALE(mv);
        case 3: return MEAS_3_SCALE(mv);
    }
    return 0;
}
