#include "peer_table.h"
#include <Arduino.h>

#if PEER_COUNT > 0
const uint8_t PeerTable::_addrs[PEER_COUNT] = PEER_ADDRS;
#else
const uint8_t PeerTable::_addrs[1] = {0};
#endif

void PeerTable::begin() {
    for (uint8_t i = 0; i < COUNT; i++) {
        _e[i] = { _addrs[i], false, 0, 0.0f, 0, 0 };
    }
}

PeerEntry* PeerTable::find(uint8_t addr) {
    for (uint8_t i = 0; i < COUNT; i++)
        if (_e[i].addr == addr) return &_e[i];
    return nullptr;
}

void PeerTable::update(uint8_t addr, int16_t rssi, float snr) {
    PeerEntry *p = find(addr);
    if (!p) return;
    p->alive      = true;
    p->rssi       = rssi;
    p->snr        = snr;
    p->lastSeenMs = millis();
    p->pktCount++;
}

bool PeerTable::isKnown(uint8_t addr) {
    return find(addr) != nullptr;
}

uint8_t PeerTable::addrAt(uint8_t i) const {
    return (i < COUNT) ? _addrs[i] : 0;
}
