#pragma once
#include <Arduino.h>
#include "lora_protocol.h"
#include "io_manager.h"
#include "radio_driver.h"
#include "ack_manager.h"
#include "peer_table.h"

// Top-level coordinator: owns all subsystems, drives the main loop.
// In main.cpp:  LoRaNode node;  setup(){ node.begin(); }  loop(){ node.tick(); }
class LoRaNode {
public:
    void begin();
    void tick();

    // ── Public API — call from the application hook in loop() ─────────────
    void sendIoCmd(uint8_t peer, uint8_t mask, uint8_t value);
    void sendMeas (uint8_t dst,  uint8_t flags);

private:
    void    _processRx();
    void    _handleFrame(const LoRaFrame *f, int16_t rssi, float snr);
    bool    _buildRelay (const LoRaFrame *f, LoRaPacket *out, uint8_t fseq);
    void    _taskReportGateway();
    void    _taskBeacon();
    void    _taskNotifyPeer(uint8_t peerAddr);
    uint8_t _nextSeq();

    IoManager   _io;
    RadioDriver _radio;
    AckManager  _ack;
    PeerTable   _peers;
    DedupTable  _dedup;

    uint8_t  _seq            = 0;
    uint8_t  _uptimeMin      = 0;
    uint32_t _lastReportMs   = 0;
    uint32_t _lastBeaconMs   = 0;
    uint32_t _lastIoNotifyMs = 0;
    uint32_t _lastUptimeMs   = 0;
};
