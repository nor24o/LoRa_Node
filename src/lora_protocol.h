#pragma once
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════════════════════════
//  lora_protocol.h  —  Custom LoRa Packet Protocol
//  Copy unchanged to every node, gateway, and backend project.
//  No external dependencies. Pure C99. Works on all MCUs and Python.
//
//  PACKET FRAME LAYOUT
//  ─────────────────────────────────────────────────────────────────────────
//  Byte  Field       Bits      Description
//  ─────────────────────────────────────────────────────────────────────────
//   0    NET·VER    [7:4]  Network ID  (0–15, set LORA_NET_ID below)
//                  [3:0]  Protocol version (currently 1)
//   1    SRC        [7:0]  Source address
//   2    DST        [7:0]  Destination address
//   3    SEQ·FL     [7:2]  Rolling sequence number 0–63
//                  [1]    FLAG_ACK_REQ — sender wants ACK
//                  [0]    FLAG_GW_COPY — receiver must relay to GW
//   4    TYPE·LEN   [7:4]  Packet type (PKT_xxx)
//                  [3:0]  Payload byte count (0–9)
//   5    HDR_CRC8   CRC-8 of bytes 0–4
//   6…n  PAYLOAD    Type-specific (see each PKT_xxx below)
//   n+1  PAY_CRC8   CRC-8 of payload only (omitted when paylen=0)
//
//  ADDRESS SCHEME
//   0x00 = Gateway (LPS8)         0x01–0xFE = Nodes        0xFF = Broadcast
//
//  PACKET TYPES
//   0x0  PKT_ACK         0-B payload  →  6 B total  (seq echoes original)
//   0x1  PKT_IO_STATUS   1-B payload  →  8 B total  (IO bitmap)
//   0x2  PKT_IO_CMD      2-B payload  →  9 B total  (mask + value)
//   0x3  PKT_MEAS        8-B payload  → 15 B total  (4× int16 ×10)
//   0x4  PKT_FULL        9-B payload  → 16 B total  (IO + 4× int16)
//   0x5  PKT_BEACON      3-B payload  → 10 B total  (fw_major/minor/peers)
//   0x6  PKT_PING        0-B payload  →  6 B total  (GW→node keepalive)
//   0x7  PKT_PONG        3-B payload  → 10 B total  (node→GW ping reply)
//
//  IO BITMAP  (PKT_IO_STATUS and PKT_FULL)
//   Bits 3:0 = digital inputs  DIN_3..DIN_0  (1=asserted)
//   Bits 7:4 = digital outputs DOUT_3..DOUT_0 (1=ON)
//   Both are OR-combined into one byte for PKT_FULL.
//
//  IO CMD  (PKT_IO_CMD)
//   mask  = which output bits to touch  (1 = change this bit)
//   value = target state for those bits (1 = ON, 0 = OFF)
//   Unmasked bits are never disturbed.
//
//  MEASUREMENTS  (PKT_MEAS, PKT_FULL)
//   4× int16 little-endian, real_value = raw / 10.0
//   Range −3276.8 … +3276.7 in any unit. 0x0000 = channel disabled.
// ═══════════════════════════════════════════════════════════════════════════

// ── Network identity ─────────────────────────────────────────────────────
#define LORA_NET_ID      0xC
#define LORA_VERSION     0x1
#define LORA_NETVER      ((uint8_t)((LORA_NET_ID << 4) | LORA_VERSION))

// ── Addresses ────────────────────────────────────────────────────────────
#define ADDR_GATEWAY     ((uint8_t)0x00)
#define ADDR_BROADCAST   ((uint8_t)0xFF)

// ── Packet types ─────────────────────────────────────────────────────────
#define PKT_ACK          0x0
#define PKT_IO_STATUS    0x1
#define PKT_IO_CMD       0x2
#define PKT_MEAS         0x3
#define PKT_FULL         0x4
#define PKT_BEACON       0x5
#define PKT_PING         0x6
#define PKT_PONG         0x7

// ── Flags (bits 1:0 of byte 3) ───────────────────────────────────────────
#define FLAG_ACK_REQ     ((uint8_t)0x02)
#define FLAG_GW_COPY     ((uint8_t)0x01)

// ── Size constants ───────────────────────────────────────────────────────
#define PKT_HEADER_LEN   6
#define PKT_MAX_PAYLOAD  9
#define PKT_MAX_LEN      16   // header(6) + max_payload(9) + pay_crc(1)

// ── Duplicate-detection ring-buffer depth ────────────────────────────────
#define DEDUP_SLOTS      8
#define SEQ_MOD          64

// ════════════════════════════════════════════════════════════════════════
//  Scaling helpers
// ════════════════════════════════════════════════════════════════════════
static inline int16_t float_to_i16(float v) {
    return (int16_t)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
}
static inline float i16_to_float(int16_t v) {
    return (float)v / 10.0f;
}

// ════════════════════════════════════════════════════════════════════════
//  CRC-8  poly=0x07  init=0x00  (ITU / CCITT)
// ════════════════════════════════════════════════════════════════════════
static inline uint8_t prot_crc8(const uint8_t *d, uint8_t n) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}

// ════════════════════════════════════════════════════════════════════════
//  Raw packet buffer (outgoing)
// ════════════════════════════════════════════════════════════════════════
typedef struct {
    uint8_t buf[PKT_MAX_LEN + 2];
    uint8_t len;
} LoRaPacket;

// ════════════════════════════════════════════════════════════════════════
//  Decoded frame (incoming)
// ════════════════════════════════════════════════════════════════════════
typedef struct {
    uint8_t  netver;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  seq;
    uint8_t  flags;
    uint8_t  type;
    uint8_t  paylen;
    uint8_t  payload[PKT_MAX_PAYLOAD + 1];
    bool     hdr_ok;
    bool     pay_ok;
} LoRaFrame;

// ════════════════════════════════════════════════════════════════════════
//  Duplicate-detection table
//  Prevents re-processing a packet that was relayed by an intermediate node.
// ════════════════════════════════════════════════════════════════════════
typedef struct {
    uint8_t  src  [DEDUP_SLOTS];
    uint8_t  seq  [DEDUP_SLOTS];
    bool     valid[DEDUP_SLOTS];
    uint8_t  head;
} DedupTable;

static inline void dedup_init(DedupTable *t) {
    memset(t, 0, sizeof(*t));
}

// Returns true if src+seq seen recently (is a duplicate).
// If not duplicate, inserts into table and returns false.
static inline bool dedup_check(DedupTable *t, uint8_t src, uint8_t seq) {
    for (uint8_t i = 0; i < DEDUP_SLOTS; i++)
        if (t->valid[i] && t->src[i] == src && t->seq[i] == seq)
            return true;
    t->src[t->head]   = src;
    t->seq[t->head]   = seq;
    t->valid[t->head] = true;
    t->head = (t->head + 1) % DEDUP_SLOTS;
    return false;
}

// ════════════════════════════════════════════════════════════════════════
//  Internal: build the 6-byte header and its CRC into p->buf[0..5]
// ════════════════════════════════════════════════════════════════════════
static inline void _hdr(LoRaPacket *p,
                         uint8_t src, uint8_t dst, uint8_t seq,
                         uint8_t type, uint8_t paylen, uint8_t flags) {
    p->buf[0] = LORA_NETVER;
    p->buf[1] = src;
    p->buf[2] = dst;
    p->buf[3] = (uint8_t)(((seq & 0x3F) << 2) | (flags & 0x03));
    p->buf[4] = (uint8_t)(((type & 0x0F) << 4) | (paylen & 0x0F));
    p->buf[5] = prot_crc8(p->buf, 5);
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_ACK  —  6 bytes
//  ack_seq must mirror the SEQ of the packet being acknowledged.
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_ack(LoRaPacket *p,
                                   uint8_t src, uint8_t dst,
                                   uint8_t ack_seq) {
    _hdr(p, src, dst, ack_seq, PKT_ACK, 0, 0);
    p->len = PKT_HEADER_LEN;
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_IO_STATUS  —  8 bytes
//  io_state: combined input+output bitmap (inputs in bits 3:0, outputs 7:4)
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_io_status(LoRaPacket *p,
                                        uint8_t src, uint8_t dst,
                                        uint8_t seq, uint8_t io_state,
                                        uint8_t flags) {
    _hdr(p, src, dst, seq, PKT_IO_STATUS, 1, flags);
    p->buf[6] = io_state;
    p->buf[7] = prot_crc8(&p->buf[6], 1);
    p->len    = 8;
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_IO_CMD  —  9 bytes
//  mask : bitmask of outputs to modify (1 = this output is affected)
//  value: target state for masked outputs (1 = ON, 0 = OFF)
//  FLAG_ACK_REQ is always set.
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_io_cmd(LoRaPacket *p,
                                     uint8_t src, uint8_t dst,
                                     uint8_t seq,
                                     uint8_t mask, uint8_t value) {
    _hdr(p, src, dst, seq, PKT_IO_CMD, 2, FLAG_ACK_REQ);
    p->buf[6] = mask;
    p->buf[7] = value;
    p->buf[8] = prot_crc8(&p->buf[6], 2);
    p->len    = 9;
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_MEAS  —  15 bytes
//  m0..m3: measured values as int16 ×10  (use float_to_i16() to convert)
//          Pass 0 for disabled/unavailable channels.
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_meas(LoRaPacket *p,
                                   uint8_t src, uint8_t dst,
                                   uint8_t seq,
                                   int16_t m0, int16_t m1,
                                   int16_t m2, int16_t m3,
                                   uint8_t flags) {
    _hdr(p, src, dst, seq, PKT_MEAS, 8, flags);
    // little-endian int16 — explicit byte order for cross-platform safety
    p->buf[6]  = (uint8_t)(m0);       p->buf[7]  = (uint8_t)((uint16_t)m0 >> 8);
    p->buf[8]  = (uint8_t)(m1);       p->buf[9]  = (uint8_t)((uint16_t)m1 >> 8);
    p->buf[10] = (uint8_t)(m2);       p->buf[11] = (uint8_t)((uint16_t)m2 >> 8);
    p->buf[12] = (uint8_t)(m3);       p->buf[13] = (uint8_t)((uint16_t)m3 >> 8);
    p->buf[14] = prot_crc8(&p->buf[6], 8);
    p->len     = 15;
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_FULL  —  16 bytes  (most common GW report packet)
//  io_state: combined input+output bitmap (same as PKT_IO_STATUS)
//  m0..m3  : measurements ×10
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_full(LoRaPacket *p,
                                   uint8_t src, uint8_t dst,
                                   uint8_t seq, uint8_t io_state,
                                   int16_t m0, int16_t m1,
                                   int16_t m2, int16_t m3,
                                   uint8_t flags) {
    _hdr(p, src, dst, seq, PKT_FULL, 9, flags);
    p->buf[6]  = io_state;
    p->buf[7]  = (uint8_t)(m0);       p->buf[8]  = (uint8_t)((uint16_t)m0 >> 8);
    p->buf[9]  = (uint8_t)(m1);       p->buf[10] = (uint8_t)((uint16_t)m1 >> 8);
    p->buf[11] = (uint8_t)(m2);       p->buf[12] = (uint8_t)((uint16_t)m2 >> 8);
    p->buf[13] = (uint8_t)(m3);       p->buf[14] = (uint8_t)((uint16_t)m3 >> 8);
    p->buf[15] = prot_crc8(&p->buf[6], 9);
    p->len     = 16;
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_BEACON  —  10 bytes
//  Always broadcast (dst = ADDR_BROADCAST).  No ACK expected.
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_beacon(LoRaPacket *p,
                                     uint8_t src, uint8_t seq,
                                     uint8_t fw_major, uint8_t fw_minor,
                                     uint8_t peer_count) {
    _hdr(p, src, ADDR_BROADCAST, seq, PKT_BEACON, 3, 0);
    p->buf[6] = fw_major;
    p->buf[7] = fw_minor;
    p->buf[8] = peer_count;
    p->buf[9] = prot_crc8(&p->buf[6], 3);
    p->len    = 10;
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_PING  —  6 bytes  (GW → node keepalive, node must reply with PONG)
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_ping(LoRaPacket *p,
                                   uint8_t src, uint8_t dst,
                                   uint8_t seq) {
    _hdr(p, src, dst, seq, PKT_PING, 0, FLAG_ACK_REQ);
    p->len = PKT_HEADER_LEN;
}

// ════════════════════════════════════════════════════════════════════════
//  PKT_PONG  —  10 bytes  (node → GW reply to ping)
//  rssi_half : |received RSSI| × 2   (e.g. –87 dBm → 174)
//  snr_qtr   : received SNR  × 4     (e.g. +6.25 dB → 25; can be negative)
//  uptime_min: node uptime in minutes (wraps at 255 → ~4.25 h)
// ════════════════════════════════════════════════════════════════════════
static inline void pkt_build_pong(LoRaPacket *p,
                                   uint8_t src, uint8_t dst,
                                   uint8_t echo_seq,
                                   uint8_t rssi_half,
                                   int8_t  snr_qtr,
                                   uint8_t uptime_min) {
    _hdr(p, src, dst, echo_seq, PKT_PONG, 3, 0);
    p->buf[6] = rssi_half;
    p->buf[7] = (uint8_t)snr_qtr;
    p->buf[8] = uptime_min;
    p->buf[9] = prot_crc8(&p->buf[6], 3);
    p->len    = 10;
}

// ════════════════════════════════════════════════════════════════════════
//  Packet decoder
//  Returns true when both HDR and payload CRCs pass.
//  Call pkt_decode() first, then use frame_xxx() accessors below.
// ════════════════════════════════════════════════════════════════════════
static inline bool pkt_decode(const uint8_t *raw, uint8_t raw_len,
                               LoRaFrame *f) {
    memset(f, 0, sizeof(*f));
    if (raw_len < PKT_HEADER_LEN) return false;

    f->netver = raw[0];
    f->src    = raw[1];
    f->dst    = raw[2];
    f->seq    = (raw[3] >> 2) & 0x3F;
    f->flags  = raw[3] & 0x03;
    f->type   = (raw[4] >> 4) & 0x0F;
    f->paylen = raw[4] & 0x0F;

    f->hdr_ok = (prot_crc8(raw, 5) == raw[5]);
    if (!f->hdr_ok) return false;
    if (f->paylen > PKT_MAX_PAYLOAD) return false;

    uint8_t need = (uint8_t)(PKT_HEADER_LEN + f->paylen
                             + (f->paylen > 0 ? 1 : 0));
    if (raw_len < need) return false;

    if (f->paylen > 0) {
        memcpy(f->payload, raw + PKT_HEADER_LEN, f->paylen);
        f->pay_ok = (prot_crc8(raw + PKT_HEADER_LEN, f->paylen)
                     == raw[PKT_HEADER_LEN + f->paylen]);
    } else {
        f->pay_ok = true;
    }
    return f->pay_ok;
}

// ════════════════════════════════════════════════════════════════════════
//  Payload accessors  —  only call after pkt_decode() returns true
// ════════════════════════════════════════════════════════════════════════

// PKT_IO_STATUS
static inline uint8_t frame_io_state(const LoRaFrame *f) {
    return f->payload[0];
}

// PKT_IO_CMD
static inline uint8_t frame_io_mask (const LoRaFrame *f) { return f->payload[0]; }
static inline uint8_t frame_io_value(const LoRaFrame *f) { return f->payload[1]; }

// PKT_MEAS  — out[0..3] are the four channels (real = out[i] / 10.0)
static inline void frame_meas(const LoRaFrame *f, int16_t out[4]) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t lo = f->payload[i * 2];
        uint8_t hi = f->payload[i * 2 + 1];
        out[i] = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    }
}

// PKT_FULL
static inline uint8_t frame_full_io(const LoRaFrame *f) { return f->payload[0]; }
static inline void frame_full_meas(const LoRaFrame *f, int16_t out[4]) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t lo = f->payload[1 + i * 2];
        uint8_t hi = f->payload[2 + i * 2];
        out[i] = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    }
}

// PKT_BEACON
static inline uint8_t frame_beacon_fw_major(const LoRaFrame *f) { return f->payload[0]; }
static inline uint8_t frame_beacon_fw_minor(const LoRaFrame *f) { return f->payload[1]; }
static inline uint8_t frame_beacon_peers   (const LoRaFrame *f) { return f->payload[2]; }

// PKT_PONG
static inline float   frame_pong_rssi    (const LoRaFrame *f) { return -(float)f->payload[0] / 2.0f; }
static inline float   frame_pong_snr     (const LoRaFrame *f) { return  (float)(int8_t)f->payload[1] / 4.0f; }
static inline uint8_t frame_pong_uptime  (const LoRaFrame *f) { return f->payload[2]; }

// ════════════════════════════════════════════════════════════════════════
//  Utility helpers
// ════════════════════════════════════════════════════════════════════════
static inline const char *pkt_type_name(uint8_t t) {
    switch (t) {
        case PKT_ACK:       return "ACK";
        case PKT_IO_STATUS: return "IO_STATUS";
        case PKT_IO_CMD:    return "IO_CMD";
        case PKT_MEAS:      return "MEAS";
        case PKT_FULL:      return "FULL";
        case PKT_BEACON:    return "BEACON";
        case PKT_PING:      return "PING";
        case PKT_PONG:      return "PONG";
        default:            return "?";
    }
}
static inline bool addr_is_node     (uint8_t a) { return a != ADDR_GATEWAY && a != ADDR_BROADCAST; }
static inline bool addr_is_broadcast(uint8_t a) { return a == ADDR_BROADCAST; }
static inline bool addr_is_gateway  (uint8_t a) { return a == ADDR_GATEWAY; }
static inline uint8_t seq_inc  (uint8_t s)             { return (uint8_t)((s + 1) % SEQ_MOD); }
static inline bool    seq_equal(uint8_t a, uint8_t b)  { return (a & 0x3F) == (b & 0x3F); }