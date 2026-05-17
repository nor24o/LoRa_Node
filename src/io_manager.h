#pragma once
#include <Arduino.h>
#include "node_config.h"

// Owns all digital I/O and analog measurement pins.
// Call begin() once in setup, pollDebounce() every loop iteration.
class IoManager {
public:
    void    begin();
    bool    pollDebounce();                     // true on stable input change
    void    applyCmd(uint8_t mask, uint8_t value);
    int16_t measRead(uint8_t ch) const;
    uint8_t combinedState()  const;             // outputs[7:4] | inputs[3:0]
    uint8_t inputState()     const { return _in;  }
    uint8_t outputState()    const { return _out; }

private:
    uint8_t _readRaw() const;

    uint8_t  _in         = 0x00;
    uint8_t  _out        = 0x00;
    uint8_t  _raw        = 0xFF;
    uint32_t _debounceMs = 0;

    static const int16_t _dinPins [8];
    static const int16_t _doutPins[8];
    static const int16_t _ainPins [4];
};
