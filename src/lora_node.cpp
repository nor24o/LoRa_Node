#include "lora_node.h"
#include "debug.h"
#include "node_config.h"

// ── LED helpers ───────────────────────────────────────────────────────────
#if LED_ENABLED
static void led_blink(uint8_t count, uint16_t on_ms, uint16_t off_ms) {
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(LED_STATUS_PIN, HIGH);
        delay(on_ms);
        digitalWrite(LED_STATUS_PIN, LOW);
        if (i < count - 1) delay(off_ms);
    }
}
static void led_rx() { digitalWrite(LED_STATUS_PIN, HIGH); delay(LED_RX_ON_MS); digitalWrite(LED_STATUS_PIN, LOW); }
#else
static void led_blink(uint8_t, uint16_t, uint16_t) {}
static void led_rx()  {}
#endif


// ════════════════════════════════════════════════════════════════════════
//  begin — hardware init, radio start, initial beacon
// ════════════════════════════════════════════════════════════════════════
void LoRaNode::begin() {
#if DEBUG_ENABLED
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    delay(500);
    DBGLN("========================================");
    DBG(" LoRa Custom Node  addr=0x%02X  fw=%d.%d\n",
        NODE_ADDR, NODE_FW_MAJOR, NODE_FW_MINOR);
    DBGLN("========================================");
#endif

#if LED_ENABLED
    pinMode(LED_STATUS_PIN, OUTPUT);
    digitalWrite(LED_STATUS_PIN, LOW);
#endif

    _io.begin();
    _peers.begin();
    dedup_init(&_dedup);

    DBG("[RADIO] %.1f MHz SF%d BW%.0f CR4/%d\n",
        LORA_FREQ_MHZ, LORA_SF, LORA_BW_KHZ, LORA_CR);

    if (!_radio.begin()) {
        while (true) led_blink(1, LED_ERROR_BLINK_MS, LED_ERROR_BLINK_MS);
    }

#if LED_ENABLED
    led_blink(LED_BLINK_INIT, 100, 100);
#endif

    // Stagger first GW report by NODE_ADDR × 3 s to avoid collisions
    uint32_t now = millis();
    _lastReportMs   = now - REPORT_INTERVAL_MS + ((uint32_t)NODE_ADDR * 3000UL);
    _lastBeaconMs   = now;
    _lastIoNotifyMs = now;
    _lastUptimeMs   = now;

    _taskBeacon();
    DBGLN("[BOOT] ready");
}


// ════════════════════════════════════════════════════════════════════════
//  tick — called every loop(); drives all periodic tasks
// ════════════════════════════════════════════════════════════════════════
void LoRaNode::tick() {
    uint32_t now = millis();

    // 1. Process any received packet
    if (RadioDriver::rxDone)
        _processRx();

    // 2. TX watchdog (defensive: recover if ISR never fires)
    _radio.tickWatchdog();

    // 3. ACK retry
    _ack.tick(_radio);

    // 4. Uptime counter
    if (now - _lastUptimeMs >= UPTIME_TICK_MS) {
        _lastUptimeMs = now;
        if (_uptimeMin < 255) _uptimeMin++;
    }

    // 5. Periodic full report to gateway
    if (now - _lastReportMs >= REPORT_INTERVAL_MS) {
        _lastReportMs = now;
        _taskReportGateway();
    }

    // 6. Periodic beacon broadcast
    if (now - _lastBeaconMs >= BEACON_INTERVAL_MS) {
        _lastBeaconMs = now;
        _taskBeacon();
    }

    // 7. IO change → notify peers
    if (_io.pollDebounce()) {
        DBG("[IO] input change 0x%02X\n", _io.inputState());
        if (!_ack.isActive()
            && (now - _lastIoNotifyMs) >= IO_NOTIFY_COOLDOWN_MS) {
            _lastIoNotifyMs = now;
            for (uint8_t i = 0; i < PeerTable::COUNT; i++)
                _taskNotifyPeer(_peers.addrAt(i));
        }
    }
}


// ════════════════════════════════════════════════════════════════════════
//  _processRx — read + validate + route one received packet
// ════════════════════════════════════════════════════════════════════════
void LoRaNode::_processRx() {
    uint8_t raw[PKT_MAX_LEN + 4];
    uint8_t rxLen;
    int16_t rssi;
    float   snr;

    if (!_radio.readRx(raw, &rxLen, &rssi, &snr)) return;

    if (rxLen < PKT_HEADER_LEN || rxLen > PKT_MAX_LEN) {
        DBG("[RX] bad length %d — dropped\n", rxLen);
        return;
    }
    if (raw[0] != LORA_NETVER) {
        DBG("[RX] foreign NETVER 0x%02X — dropped\n", raw[0]);
        return;
    }

    LoRaFrame f;
    if (!pkt_decode(raw, rxLen, &f)) {
        DBG("[RX] CRC fail — dropped\n");
        return;
    }
    if (f.dst != NODE_ADDR && f.dst != ADDR_BROADCAST && f.dst != ADDR_GATEWAY) {
        DBG("[RX] dst=0x%02X not for us — dropped\n", f.dst);
        return;
    }
    if (f.type != PKT_ACK && f.dst != ADDR_BROADCAST) {
        if (dedup_check(&_dedup, f.src, f.seq)) {
            DBG("[RX] duplicate src=0x%02X seq=%d — dropped\n", f.src, f.seq);
            return;
        }
    }

    _handleFrame(&f, rssi, snr);
}


// ════════════════════════════════════════════════════════════════════════
//  _buildRelay — try to forward a received packet to the gateway
//  Returns false if the packet type cannot be relayed.
// ════════════════════════════════════════════════════════════════════════
bool LoRaNode::_buildRelay(const LoRaFrame *f, LoRaPacket *out, uint8_t fseq) {
    if (f->type == PKT_FULL && f->pay_ok) {
        int16_t m[4];
        frame_full_meas(f, m);
        pkt_build_full(out, NODE_ADDR, ADDR_GATEWAY, fseq,
                       frame_full_io(f), m[0], m[1], m[2], m[3], FLAG_ACK_REQ);
        return true;
    }
    if (f->type == PKT_IO_STATUS && f->pay_ok) {
        pkt_build_io_status(out, NODE_ADDR, ADDR_GATEWAY, fseq,
                            frame_io_state(f), FLAG_ACK_REQ);
        return true;
    }
    return false;
}


// ════════════════════════════════════════════════════════════════════════
//  _handleFrame — dispatch a validated, addressed frame
// ════════════════════════════════════════════════════════════════════════
void LoRaNode::_handleFrame(const LoRaFrame *f, int16_t rssi, float snr) {
    DBG("[RX] %s  src=0x%02X dst=0x%02X seq=%d flags=0x%X len=%d  RSSI=%d SNR=%.1f\n",
        pkt_type_name(f->type), f->src, f->dst,
        f->seq, f->flags, f->paylen, rssi, snr);
    DBGHEX(f->payload, f->paylen);
    led_rx();

    if (_peers.isKnown(f->src))
        _peers.update(f->src, rssi, snr);

    // ── ACK reply ────────────────────────────────────────────────────────
    if ((f->flags & FLAG_ACK_REQ) && f->dst == NODE_ADDR) {
        LoRaPacket ack;
        pkt_build_ack(&ack, NODE_ADDR, f->src, f->seq);
        _radio.tx(ack.buf, ack.len);
        DBG("[ACK] sent to 0x%02X seq=%d\n", f->src, f->seq);
    }

    // ── GW relay ─────────────────────────────────────────────────────────
    if ((f->flags & FLAG_GW_COPY)
        && !addr_is_gateway(f->src)
        && f->dst != ADDR_GATEWAY) {
        LoRaPacket fwd;
        uint8_t fseq = _nextSeq();
        if (!_ack.isActive() && _buildRelay(f, &fwd, fseq)) {
            DBG("[RELAY] %s from 0x%02X → GW\n", pkt_type_name(f->type), f->src);
            _ack.send(fwd.buf, fwd.len, ADDR_GATEWAY, fseq, _radio);
        }
    }

    // ── Dispatch by type ─────────────────────────────────────────────────
    switch (f->type) {

        case PKT_ACK:
            _ack.received(f->src, f->seq);
            break;

        case PKT_IO_STATUS: {
            uint8_t st = frame_io_state(f);
            DBG("[IO_STATUS] from 0x%02X  state=0x%02X\n", f->src, st);
            // Application hook: react to remote IO state here, e.g.:
            // _io.applyCmd(0x10, (st & 0x01) ? 0x10 : 0x00);
            break;
        }

        case PKT_IO_CMD:
            if (f->dst != NODE_ADDR) break;
            _io.applyCmd(frame_io_mask(f), frame_io_value(f));
            {
                LoRaPacket rep;
                uint8_t seq = _nextSeq();
                pkt_build_io_status(&rep, NODE_ADDR, f->src, seq,
                                    _io.combinedState(), 0);
                _radio.tx(rep.buf, rep.len);
            }
            break;

        case PKT_MEAS: {
            int16_t m[4];
            frame_meas(f, m);
            DBG("[MEAS] from 0x%02X  %.1f %.1f %.1f %.1f\n", f->src,
                i16_to_float(m[0]), i16_to_float(m[1]),
                i16_to_float(m[2]), i16_to_float(m[3]));
            break;
        }

        case PKT_FULL: {
            int16_t m[4];
            frame_full_meas(f, m);
            DBG("[FULL] from 0x%02X  IO=0x%02X  %.1f %.1f %.1f %.1f\n",
                f->src, frame_full_io(f),
                i16_to_float(m[0]), i16_to_float(m[1]),
                i16_to_float(m[2]), i16_to_float(m[3]));
            break;
        }

        case PKT_BEACON:
            DBG("[BEACON] from 0x%02X  fw=%d.%d peers=%d\n",
                f->src, frame_beacon_fw_major(f),
                frame_beacon_fw_minor(f), frame_beacon_peers(f));
            break;

        case PKT_PING:
            if (f->dst != NODE_ADDR && f->dst != ADDR_BROADCAST) break;
            {
                LoRaPacket pong;
                uint8_t rssi_half = (uint8_t)((-rssi) * 2);
                int8_t  snr_qtr   = (int8_t)(snr * 4.0f);
                pkt_build_pong(&pong, NODE_ADDR, f->src, f->seq,
                               rssi_half, snr_qtr, _uptimeMin);
                _radio.tx(pong.buf, pong.len);
                DBG("[PING] replied to 0x%02X  up=%d min\n", f->src, _uptimeMin);
            }
            break;

        case PKT_PONG:
            DBG("[PONG] from 0x%02X  rssi=%.1f snr=%.1f up=%d min\n",
                f->src, frame_pong_rssi(f), frame_pong_snr(f),
                frame_pong_uptime(f));
            break;

        default:
            DBG("[RX] unknown type 0x%X — ignored\n", f->type);
            break;
    }
}


// ════════════════════════════════════════════════════════════════════════
//  Periodic tasks
// ════════════════════════════════════════════════════════════════════════

void LoRaNode::_taskReportGateway() {
    LoRaPacket p;
    uint8_t seq = _nextSeq();
    pkt_build_full(&p, NODE_ADDR, ADDR_GATEWAY, seq,
                   _io.combinedState(),
                   _io.measRead(0), _io.measRead(1),
                   _io.measRead(2), _io.measRead(3),
                   FLAG_ACK_REQ);
    if (!_ack.isActive()) {
        DBGLN("[TASK] GW report");
        _ack.send(p.buf, p.len, ADDR_GATEWAY, seq, _radio);
    } else {
        DBGLN("[TASK] GW report skipped — ACK pending");
    }
}

void LoRaNode::_taskBeacon() {
    LoRaPacket p;
    pkt_build_beacon(&p, NODE_ADDR, _nextSeq(),
                     NODE_FW_MAJOR, NODE_FW_MINOR, PeerTable::COUNT);
    _radio.tx(p.buf, p.len);
    DBGLN("[TASK] beacon sent");
}

void LoRaNode::_taskNotifyPeer(uint8_t peerAddr) {
    LoRaPacket p;
    uint8_t seq = _nextSeq();
    pkt_build_io_status(&p, NODE_ADDR, peerAddr, seq,
                        _io.combinedState(),
                        FLAG_ACK_REQ | FLAG_GW_COPY);
    if (!_ack.isActive()) {
        DBG("[TASK] IO notify → 0x%02X\n", peerAddr);
        _ack.send(p.buf, p.len, peerAddr, seq, _radio);
    }
}


// ════════════════════════════════════════════════════════════════════════
//  Public API — call from the application hook in main.cpp loop()
// ════════════════════════════════════════════════════════════════════════

void LoRaNode::sendIoCmd(uint8_t peer, uint8_t mask, uint8_t value) {
    LoRaPacket p;
    uint8_t seq = _nextSeq();
    pkt_build_io_cmd(&p, NODE_ADDR, peer, seq, mask, value);
    if (!_ack.isActive()) {
        DBG("[CMD] IO cmd → 0x%02X mask=0x%02X val=0x%02X\n", peer, mask, value);
        _ack.send(p.buf, p.len, peer, seq, _radio);
    }
}

void LoRaNode::sendMeas(uint8_t dst, uint8_t flags) {
    LoRaPacket p;
    uint8_t seq = _nextSeq();
    pkt_build_meas(&p, NODE_ADDR, dst, seq,
                   _io.measRead(0), _io.measRead(1),
                   _io.measRead(2), _io.measRead(3), flags);
    if (!_ack.isActive())
        _ack.send(p.buf, p.len, dst, seq, _radio);
}

uint8_t LoRaNode::_nextSeq() {
    _seq = seq_inc(_seq);
    return _seq;
}
