#pragma once
#include <stdint.h>
#include <string.h>
#include "lora_protocol.h"

class RadioDriver;  // forward declaration — full type in ack_manager.cpp

// Single-slot ACK tracker: stores one unacknowledged packet and retransmits
// it with exponential backoff until an ACK arrives or retries are exhausted.
// Pass RadioDriver by reference at each call site — no stored pointer needed.
class AckManager {
public:
    void send(const uint8_t *buf, uint8_t len, uint8_t dst, uint8_t seq,
              RadioDriver &radio);
    void received(uint8_t src, uint8_t seq);
    void tick(RadioDriver &radio);
    bool isActive() const { return _p.active; }

private:
    struct Pending {
        bool     active      = false;
        uint8_t  dst         = 0;
        uint8_t  seq         = 0;
        uint8_t  retriesLeft = 0;
        uint32_t deadlineMs  = 0;
        uint8_t  pkt[PKT_MAX_LEN + 2];
        uint8_t  pktLen      = 0;
    } _p;
};
