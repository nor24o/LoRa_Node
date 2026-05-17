#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "node_config.h"

struct PeerEntry {
    uint8_t  addr;
    bool     alive;
    int16_t  rssi;
    float    snr;
    uint32_t lastSeenMs;
    uint32_t pktCount;
};

// Fixed-size table of known peers; size is PEER_COUNT from node_config.h.
class PeerTable {
public:
    static const uint8_t COUNT = (PEER_COUNT > 0 ? PEER_COUNT : 1);

    void       begin();
    PeerEntry* find(uint8_t addr);
    void       update(uint8_t addr, int16_t rssi, float snr);
    bool       isKnown(uint8_t addr);
    uint8_t    addrAt(uint8_t i) const;

private:
    PeerEntry _e[PEER_COUNT > 0 ? PEER_COUNT : 1];
    static const uint8_t _addrs[PEER_COUNT > 0 ? PEER_COUNT : 1];
};
