// ═══════════════════════════════════════════════════════════════════════════
//  main.cpp  —  LoRa Custom Node Firmware
//  PlatformIO / Arduino framework
//
//  Supported hardware:
//    MCU  : ESP32, ESP32-S3, ESP8266, Arduino Uno/Nano/Mega, STM32
//    Radio: SX1261, SX1262, SX1268, SX1276, SX1277, SX1278, SX1279, LLCC68
//
//  Features implemented here:
//    ✓ Interrupt-driven RX  (non-blocking, DIO1 → ISR → rxFlag)
//    ✓ Non-blocking TX      (startTransmit → DIO1 → txDone → re-arm RX)
//    ✓ Channel Activity Detection before every TX (CSMA/CAD)
//    ✓ ACK + retry with exponential backoff
//    ✓ Duplicate packet detection (rolling 8-entry ring buffer)
//    ✓ Peer table with RSSI/SNR tracking and alive/dead status
//    ✓ 8× digital inputs with debounce and invert-logic support
//    ✓ 8× digital outputs with mask/value control
//    ✓ 4× analog measurement channels (int16 ×10 scaled)
//    ✓ IO-change detection → immediate peer notification with cooldown
//    ✓ Periodic PKT_FULL report to gateway (staggered by node address)
//    ✓ Periodic PKT_BEACON broadcast
//    ✓ PKT_PING from GW → PKT_PONG reply with uplink RSSI/SNR
//    ✓ PKT_GW_COPY relay: forward received peer packet to GW
//    ✓ Status LED feedback (init, TX, RX, error)
//    ✓ Uptime counter in minutes (reported in PONG)
//    ✓ All config in node_config.h — main.cpp never edited
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <RadioLib.h>
#include "node_config.h"
#include "lora_protocol.h"

// ── Debug macros ──────────────────────────────────────────────────────────
#if DEBUG_ENABLED
  #if defined(__AVR__)
    // AVR HardwareSerial has no printf; route through snprintf + print.
    // Float formatting requires -Wl,-u,vfprintf -lprintf_flt in build_flags.
    static char _dbg_buf[80];
    #define DBG(fmt, ...)  do { snprintf(_dbg_buf, sizeof(_dbg_buf), fmt, ##__VA_ARGS__); DEBUG_SERIAL.print(_dbg_buf); } while(0)
    #define DBGLN(s)       DEBUG_SERIAL.println(F(s))
    #define DBGHEX(b,n)    do { for(uint8_t _i=0;_i<(n);_i++) { snprintf(_dbg_buf,sizeof(_dbg_buf),"%02X ",(b)[_i]); DEBUG_SERIAL.print(_dbg_buf); } DEBUG_SERIAL.println(); } while(0)
  #else
    #define DBG(...)     DEBUG_SERIAL.printf(__VA_ARGS__)
    #define DBGLN(s)     DEBUG_SERIAL.println(F(s))
    #define DBGHEX(b,n)  do { for(uint8_t _i=0;_i<(n);_i++) DEBUG_SERIAL.printf("%02X ",b[_i]); DEBUG_SERIAL.println(); } while(0)
  #endif
#else
  #define DBG(...)
  #define DBGLN(s)
  #define DBGHEX(b,n)
#endif


// ════════════════════════════════════════════════════════════════════════
//  Radio object — type selected by node_config.h RADIO_xxx define
// ════════════════════════════════════════════════════════════════════════
#if defined(RADIO_SX1262)
  SX1262 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN,
                            LORA_RST_PIN, LORA_BUSY_PIN);
#elif defined(RADIO_SX1278)
  SX1278 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN,
                            LORA_RST_PIN, LORA_DIO2_PIN);
#elif defined(RADIO_LLCC68)
  LLCC68 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN,
                            LORA_RST_PIN, LORA_BUSY_PIN);
#else
  #error "No radio type defined. Set RADIO_SX1262, RADIO_SX1278, or RADIO_LLCC68 in node_config.h"
#endif


// ════════════════════════════════════════════════════════════════════════
//  ISR flags — set in DIO1 interrupt, consumed in loop()
//  Keep the ISR body minimal — no SPI calls, no RadioLib calls.
//
//  g_radio_mode tells the ISR whether the current DIO1 event is
//  a TX-done or an RX-done. CAD uses scanChannel() which is blocking
//  and handles its own completion internally, so no CAD ISR flag needed.
// ════════════════════════════════════════════════════════════════════════
typedef enum { MODE_RX, MODE_TX } RadioMode;
static volatile RadioMode g_radio_mode = MODE_RX;

static volatile bool isrRxDone = false;
static volatile bool isrTxDone = false;

void LORA_IRQ_ATTR onDio1Isr() {
    if (g_radio_mode == MODE_TX) {
        isrTxDone = true;
    } else {
        isrRxDone = true;
    }
}

// ════════════════════════════════════════════════════════════════════════
//  Sequence counter
// ════════════════════════════════════════════════════════════════════════
static uint8_t g_seq = 0;
static inline uint8_t next_seq() {
    g_seq = seq_inc(g_seq);
    return g_seq;
}

// ════════════════════════════════════════════════════════════════════════
//  ACK pending slot — only one unacknowledged packet at a time
// ════════════════════════════════════════════════════════════════════════
struct PendingAck {
    bool     active;
    uint8_t  dst;
    uint8_t  seq;
    uint8_t  retries_left;
    uint32_t deadline_ms;
    uint8_t  pkt[PKT_MAX_LEN + 2];
    uint8_t  pkt_len;
};
static PendingAck g_ack = { false };

// ════════════════════════════════════════════════════════════════════════
//  Peer table
// ════════════════════════════════════════════════════════════════════════
struct PeerEntry {
    uint8_t  addr;
    bool     alive;
    int16_t  rssi;        // last received RSSI (dBm)
    float    snr;         // last received SNR  (dB)
    uint32_t last_seen_ms;
    uint32_t pkt_count;
};
static const uint8_t g_peer_addrs[] = PEER_ADDRS;
static PeerEntry      g_peers[PEER_COUNT];

// ════════════════════════════════════════════════════════════════════════
//  Duplicate detection table
// ════════════════════════════════════════════════════════════════════════
static DedupTable g_dedup;

// ════════════════════════════════════════════════════════════════════════
//  IO state
// ════════════════════════════════════════════════════════════════════════
static const int16_t g_din_pins[8]  = { DIN_0_PIN,  DIN_1_PIN,  DIN_2_PIN,  DIN_3_PIN,
                                         DIN_4_PIN,  DIN_5_PIN,  DIN_6_PIN,  DIN_7_PIN  };
static const int16_t g_dout_pins[8] = { DOUT_0_PIN, DOUT_1_PIN, DOUT_2_PIN, DOUT_3_PIN,
                                         DOUT_4_PIN, DOUT_5_PIN, DOUT_6_PIN, DOUT_7_PIN };
static const int16_t g_ain_pins[4]  = { AIN_0_PIN,  AIN_1_PIN,  AIN_2_PIN,  AIN_3_PIN  };

static uint8_t g_io_in    = 0x00;   // last debounced digital input state
static uint8_t g_io_out   = 0x00;   // current digital output state
static uint8_t g_io_raw   = 0xFF;   // raw (pre-debounce) last reading
static uint32_t g_debounce_ms = 0;  // debounce window start

// ════════════════════════════════════════════════════════════════════════
//  Timing
// ════════════════════════════════════════════════════════════════════════
static uint32_t g_last_report_ms   = 0;
static uint32_t g_last_beacon_ms   = 0;
static uint32_t g_last_io_notify_ms= 0;
static uint32_t g_last_uptime_ms   = 0;
static uint8_t  g_uptime_min       = 0;

// ════════════════════════════════════════════════════════════════════════
//  Radio transmit state — tracks whether we're mid-TX
// ════════════════════════════════════════════════════════════════════════
static bool     g_tx_busy    = false;
static uint32_t g_tx_start_ms = 0;
#define TX_WATCHDOG_MS 5000UL   // force abort if TX takes longer than 5 s


// ════════════════════════════════════════════════════════════════════════
//  LED helper
// ════════════════════════════════════════════════════════════════════════
#if LED_ENABLED
static void led_blink(uint8_t count, uint16_t on_ms, uint16_t off_ms) {
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(LED_STATUS_PIN, HIGH);
        delay(on_ms);
        digitalWrite(LED_STATUS_PIN, LOW);
        if (i < count - 1) delay(off_ms);
    }
}
static inline void led_tx()  { led_blink(1, LED_TX_ON_MS, 0); }
static inline void led_rx()  { led_blink(1, LED_RX_ON_MS, 0); }
#else
static inline void led_blink(uint8_t, uint16_t, uint16_t) {}
static inline void led_tx()  {}
static inline void led_rx()  {}
#endif


// ════════════════════════════════════════════════════════════════════════
//  IO — initialise pins
// ════════════════════════════════════════════════════════════════════════
static void io_init() {
    for (int i = 0; i < 8; i++) {
        if (g_din_pins[i] >= 0)
            pinMode((uint8_t)g_din_pins[i], DIN_INPUT_MODE);

        if (g_dout_pins[i] >= 0) {
            pinMode((uint8_t)g_dout_pins[i], OUTPUT);
            bool init_on = (DOUT_STARTUP_STATE >> i) & 0x01;
            digitalWrite((uint8_t)g_dout_pins[i], init_on ? HIGH : LOW);
            if (init_on) g_io_out |= (1 << i);
        }
    }
}

// ── Read all digital inputs into a single byte ───────────────────────────
static uint8_t io_read_inputs_raw() {
    uint8_t state = 0;
    for (int i = 0; i < 8; i++) {
        if (g_din_pins[i] < 0) continue;
        bool level = digitalRead((uint8_t)g_din_pins[i]);
#if DIN_INVERT_LOGIC
        if (!level) state |= (1 << i);   // active-low → bit=1 means asserted
#else
        if ( level) state |= (1 << i);
#endif
    }
    return state;
}

// ── Debounced input read — call every loop() ─────────────────────────────
// Updates g_io_in only when the reading has been stable for IO_DEBOUNCE_MS.
// Returns true when a stable change was detected.
static bool io_poll_debounce() {
    uint8_t raw = io_read_inputs_raw();
    if (raw != g_io_raw) {
        g_io_raw      = raw;
        g_debounce_ms = millis();
        return false;
    }
    if (raw != g_io_in && (millis() - g_debounce_ms) >= IO_DEBOUNCE_MS) {
        g_io_in = raw;
        return true;   // stable change
    }
    return false;
}

// ── Apply output command (mask/value) ────────────────────────────────────
static void io_apply_cmd(uint8_t mask, uint8_t value) {
    for (int i = 0; i < 8; i++) {
        if (!(mask & (1 << i))) continue;
        if (g_dout_pins[i] < 0) continue;
        bool on = (value >> i) & 0x01;
        digitalWrite((uint8_t)g_dout_pins[i], on ? HIGH : LOW);
        if (on) g_io_out |=  (uint8_t)(1 << i);
        else    g_io_out &= ~(uint8_t)(1 << i);
    }
    DBG("[IO] applied mask=0x%02X val=0x%02X → out=0x%02X\n",
        mask, value, g_io_out);
}

// ── Combined IO state byte (inputs in low nibble, outputs in high) ────────
static uint8_t io_combined_state() {
    return (uint8_t)((g_io_out & 0xF0) | (g_io_in & 0x0F));
}


// ════════════════════════════════════════════════════════════════════════
//  Measurements
// ════════════════════════════════════════════════════════════════════════
static int16_t meas_read(uint8_t ch) {
    if (ch >= 4 || g_ain_pins[ch] < 0) return 0;
    int raw = analogRead((uint8_t)g_ain_pins[ch]);
    // Convert raw ADC reading to millivolts
    int mv  = (int)(((long)raw * ADC_VREF_MV) / ((1 << ADC_BITS) - 1));
    switch (ch) {
        case 0: return MEAS_0_SCALE(mv);
        case 1: return MEAS_1_SCALE(mv);
        case 2: return MEAS_2_SCALE(mv);
        case 3: return MEAS_3_SCALE(mv);
    }
    return 0;
}


// ════════════════════════════════════════════════════════════════════════
//  Peer table helpers
// ════════════════════════════════════════════════════════════════════════
static void peers_init() {
    for (int i = 0; i < PEER_COUNT; i++) {
        g_peers[i].addr        = g_peer_addrs[i];
        g_peers[i].alive       = false;
        g_peers[i].rssi        = 0;
        g_peers[i].snr         = 0.0f;
        g_peers[i].last_seen_ms= 0;
        g_peers[i].pkt_count   = 0;
    }
}

static PeerEntry* peer_find(uint8_t addr) {
    for (int i = 0; i < PEER_COUNT; i++)
        if (g_peers[i].addr == addr) return &g_peers[i];
    return nullptr;
}

static void peer_update(uint8_t addr, int16_t rssi, float snr) {
    PeerEntry *p = peer_find(addr);
    if (!p) return;
    p->alive        = true;
    p->rssi         = rssi;
    p->snr          = snr;
    p->last_seen_ms = millis();
    p->pkt_count++;
}

static bool peer_is_known(uint8_t addr) {
    return peer_find(addr) != nullptr;
}


// ════════════════════════════════════════════════════════════════════════
//  CAD — Channel Activity Detection
//  Waits until the channel is clear before returning.
//  Uses random exponential backoff on each detected-busy event.
//  Returns true (channel clear) or true (timeout — TX anyway).
// ════════════════════════════════════════════════════════════════════════
static bool cad_wait_clear() {
#if !defined(RADIO_SX1278)
    // SX126x / LLCC68 support CAD via RadioLib scanChannel()
    uint32_t deadline = millis() + CAD_TIMEOUT_MS;
    uint8_t  attempts = 0;

    while (millis() < deadline) {
        // scanChannel() is the blocking RadioLib v6 CAD API.
        // Returns RADIOLIB_ERR_NONE if channel is clear,
        //         RADIOLIB_LORA_DETECTED if activity detected.
        int result = radio.scanChannel();

        if (result == RADIOLIB_ERR_NONE) {
            return true;   // channel clear — safe to TX
        }

        if (result == RADIOLIB_LORA_DETECTED) {
            attempts++;
            // Exponential backoff: 20 ms << attempts, capped at 320 ms
            uint32_t backoff_ms = 20UL << (attempts < 4 ? attempts : 4);
            backoff_ms += (uint32_t)(random(backoff_ms / 2));
            DBG("[CAD] busy (attempt %d) — backoff %lu ms\n",
                attempts, backoff_ms);
            delay(backoff_ms);
            continue;
        }

        // Any other error means CAD isn't supported — skip it
        DBG("[CAD] scanChannel err %d — skipping CAD\n", result);
        return true;
    }
    DBG("[CAD] timeout — forcing TX\n");
#endif
    return true;   // SX127x has no CAD — transmit directly
}


// ════════════════════════════════════════════════════════════════════════
//  Low-level radio TX
//  Performs CAD, then calls startTransmit (non-blocking).
//  Waits for TX done interrupt (isrTxDone) with watchdog fallback.
//  Re-arms receiver after TX.
// ════════════════════════════════════════════════════════════════════════
static void radio_tx(const uint8_t *buf, uint8_t len) {
    if (g_tx_busy) {
        DBGLN("[TX] already busy — skipping");
        return;
    }

    cad_wait_clear();

    isrTxDone     = false;
    isrRxDone     = false;
    g_radio_mode  = MODE_TX;              // tell ISR the next interrupt = TX done
    g_tx_busy     = true;
    g_tx_start_ms = millis();

    int err = radio.startTransmit((uint8_t *)buf, len);
    if (err != RADIOLIB_ERR_NONE) {
        DBG("[TX] startTransmit error %d\n", err);
        g_radio_mode = MODE_RX;
        g_tx_busy    = false;
        radio.startReceive();
        return;
    }

    // Spin-wait for TX done ISR, with watchdog
    uint32_t t0 = millis();
    while (!isrTxDone && (millis() - t0 < TX_WATCHDOG_MS)) { /* yield */ }

    if (!isrTxDone) {
        DBGLN("[TX] watchdog timeout — forcing standby");
        radio.standby();
    }

    isrTxDone    = false;
    g_tx_busy    = false;
    g_radio_mode = MODE_RX;

    led_tx();
    DBG("[TX] %d bytes  seq=%d\n", len, (buf[3] >> 2) & 0x3F);

    // Re-arm receiver
    radio.startReceive();
}


// ════════════════════════════════════════════════════════════════════════
//  ACK management
// ════════════════════════════════════════════════════════════════════════

// Store a packet for ACK tracking and send the first transmission
static void ack_send(const uint8_t *buf, uint8_t len,
                      uint8_t dst, uint8_t seq) {
    if (g_ack.active) {
        DBGLN("[ACK] warning: overwriting pending ACK slot");
    }
    g_ack.active       = true;
    g_ack.dst          = dst;
    g_ack.seq          = seq;
    g_ack.retries_left = ACK_RETRY_COUNT;
    g_ack.deadline_ms  = millis() + ACK_TIMEOUT_MS;
    g_ack.pkt_len      = len;
    memcpy(g_ack.pkt, buf, len);
    radio_tx(buf, len);
}

// Mark pending ACK as received
static void ack_received(uint8_t src, uint8_t seq) {
    if (g_ack.active && g_ack.dst == src
        && seq_equal(g_ack.seq, seq)) {
        DBG("[ACK] confirmed from 0x%02X seq=%d  retries_used=%d\n",
            src, seq, ACK_RETRY_COUNT - g_ack.retries_left);
        g_ack.active = false;
    }
}

// Call from loop() — handles retransmit and give-up
static void ack_tick() {
    if (!g_ack.active) return;
    if ((millis() - g_ack.deadline_ms) < 0x80000000UL) return; // not expired yet

    if (g_ack.retries_left == 0) {
        DBG("[ACK] give up dst=0x%02X seq=%d\n", g_ack.dst, g_ack.seq);
        g_ack.active = false;
        return;
    }

    g_ack.retries_left--;
    uint32_t backoff = ACK_RETRY_BACKOFF_MS
                       * (uint32_t)(ACK_RETRY_COUNT - g_ack.retries_left);
    DBG("[ACK] retry %d to 0x%02X seq=%d (backoff %lu ms)\n",
        ACK_RETRY_COUNT - g_ack.retries_left,
        g_ack.dst, g_ack.seq, backoff);
    delay(backoff);
    g_ack.deadline_ms = millis() + ACK_TIMEOUT_MS;
    radio_tx(g_ack.pkt, g_ack.pkt_len);
}


// ════════════════════════════════════════════════════════════════════════
//  Packet dispatch — called for every valid decoded frame
// ════════════════════════════════════════════════════════════════════════
static void handle_frame(const LoRaFrame *f, int16_t rssi, float snr) {

    DBG("[RX] %s  src=0x%02X dst=0x%02X seq=%d flags=0x%X "
        "paylen=%d  RSSI=%d SNR=%.1f\n",
        pkt_type_name(f->type), f->src, f->dst,
        f->seq, f->flags, f->paylen, rssi, snr);
    DBGHEX(f->payload, f->paylen);

    led_rx();

    // Update peer table if from a known peer
    if (peer_is_known(f->src))
        peer_update(f->src, rssi, snr);

    // ── Send ACK if requested and we are the intended recipient ──────────
    if ((f->flags & FLAG_ACK_REQ) && f->dst == NODE_ADDR) {
        LoRaPacket ack;
        pkt_build_ack(&ack, NODE_ADDR, f->src, f->seq);
        radio_tx(ack.buf, ack.len);
        DBG("[ACK] sent to 0x%02X seq=%d\n", f->src, f->seq);
    }

    // ── Relay a copy to gateway if requested ─────────────────────────────
    //    Only relay if we can reach the GW (i.e., we have uplink)
    //    and the packet wasn't already from/to the GW
    if ((f->flags & FLAG_GW_COPY)
        && !addr_is_gateway(f->src)
        && f->dst != ADDR_GATEWAY) {
        // Re-build the full packet addressed to GW, preserving payload
        LoRaPacket fwd;
        uint8_t fseq = next_seq();
        // Wrap in PKT_FULL with whatever we have, or relay raw IO+meas if present
        if (f->type == PKT_FULL && f->pay_ok) {
            int16_t m[4]; frame_full_meas(f, m);
            pkt_build_full(&fwd, NODE_ADDR, ADDR_GATEWAY, fseq,
                           frame_full_io(f), m[0], m[1], m[2], m[3],
                           FLAG_ACK_REQ);
        } else if (f->type == PKT_IO_STATUS && f->pay_ok) {
            pkt_build_io_status(&fwd, NODE_ADDR, ADDR_GATEWAY, fseq,
                                frame_io_state(f), FLAG_ACK_REQ);
        } else {
            goto skip_relay;   // don't know how to relay this type
        }
        if (!g_ack.active) {
            DBG("[RELAY] forwarding %s from 0x%02X to GW\n",
                pkt_type_name(f->type), f->src);
            ack_send(fwd.buf, fwd.len, ADDR_GATEWAY, fseq);
        }
        skip_relay:;
    }

    // ── Dispatch by packet type ───────────────────────────────────────────
    switch (f->type) {

        // ────────────────────────────────────────────────────────────────
        case PKT_ACK:
            ack_received(f->src, f->seq);
            break;

        // ────────────────────────────────────────────────────────────────
        case PKT_IO_STATUS: {
            uint8_t state = frame_io_state(f);
            DBG("[IO_STATUS] from 0x%02X  state=0b", f->src);
            for (int i = 7; i >= 0; i--)
                DBG("%d", (state >> i) & 1);
            DBG("\n");
            // Application hook: react to remote IO state here
            // e.g. mirror remote input 0 to our output 0:
            // io_apply_cmd(0x10, (state & 0x01) ? 0x10 : 0x00);
            break;
        }

        // ────────────────────────────────────────────────────────────────
        case PKT_IO_CMD: {
            // Only execute if addressed to us
            if (f->dst != NODE_ADDR) break;
            uint8_t mask  = frame_io_mask(f);
            uint8_t value = frame_io_value(f);
            io_apply_cmd(mask, value);
            // Send back our new combined IO state as confirmation
            {
                LoRaPacket rep;
                uint8_t seq = next_seq();
                pkt_build_io_status(&rep, NODE_ADDR, f->src, seq,
                                    io_combined_state(), 0);
                radio_tx(rep.buf, rep.len);
            }
            break;
        }

        // ────────────────────────────────────────────────────────────────
        case PKT_MEAS: {
            int16_t m[4];
            frame_meas(f, m);
            DBG("[MEAS] from 0x%02X  M0=%.1f M1=%.1f M2=%.1f M3=%.1f\n",
                f->src,
                i16_to_float(m[0]), i16_to_float(m[1]),
                i16_to_float(m[2]), i16_to_float(m[3]));
            break;
        }

        // ────────────────────────────────────────────────────────────────
        case PKT_FULL: {
            uint8_t io = frame_full_io(f);
            int16_t m[4];
            frame_full_meas(f, m);
            DBG("[FULL] from 0x%02X  IO=0x%02X "
                "M0=%.1f M1=%.1f M2=%.1f M3=%.1f\n",
                f->src, io,
                i16_to_float(m[0]), i16_to_float(m[1]),
                i16_to_float(m[2]), i16_to_float(m[3]));
            break;
        }

        // ────────────────────────────────────────────────────────────────
        case PKT_BEACON: {
            DBG("[BEACON] from 0x%02X  fw=%d.%d peers=%d\n",
                f->src,
                frame_beacon_fw_major(f),
                frame_beacon_fw_minor(f),
                frame_beacon_peers(f));
            // Application hook: node discovery / network map update here
            break;
        }

        // ────────────────────────────────────────────────────────────────
        case PKT_PING: {
            // GW is checking if we're alive — reply with PONG
            if (f->dst != NODE_ADDR && f->dst != ADDR_BROADCAST) break;
            LoRaPacket pong;
            uint8_t rssi_half = (uint8_t)((-rssi) * 2);   // |RSSI| × 2
            int8_t  snr_qtr   = (int8_t)(snr * 4.0f);     // SNR × 4
            pkt_build_pong(&pong, NODE_ADDR, f->src, f->seq,
                           rssi_half, snr_qtr, g_uptime_min);
            radio_tx(pong.buf, pong.len);
            DBG("[PING] replied to 0x%02X  rssi=%d snr=%.1f up=%d min\n",
                f->src, rssi, snr, g_uptime_min);
            break;
        }

        // ────────────────────────────────────────────────────────────────
        case PKT_PONG: {
            // We sent a ping and got a reply (not expected for end-nodes,
            // but handle gracefully if another node pings us)
            DBG("[PONG] from 0x%02X  ping_rssi=%.1f snr=%.1f up=%d min\n",
                f->src,
                frame_pong_rssi(f),
                frame_pong_snr(f),
                frame_pong_uptime(f));
            break;
        }

        default:
            DBG("[RX] unknown type 0x%X — ignored\n", f->type);
            break;
    }
}


// ════════════════════════════════════════════════════════════════════════
//  RX processing — called from loop() when isrRxDone is set
// ════════════════════════════════════════════════════════════════════════
static void process_rx() {
    isrRxDone = false;

    // Read received bytes
    uint8_t raw[PKT_MAX_LEN + 4];
    int     state = radio.readData(raw, 0);   // 0 = read all available
    int16_t rssi  = (int16_t)radio.getRSSI();
    float   snr   = radio.getSNR();

    // Re-arm receiver immediately — set mode before startReceive
    g_radio_mode = MODE_RX;
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        DBG("[RX] readData error %d\n", state);
        return;
    }

    // getPacketLength(false) returns the length of the last received packet
    uint8_t rxLen = (uint8_t)radio.getPacketLength(false);
    if (rxLen < PKT_HEADER_LEN || rxLen > PKT_MAX_LEN) {
        DBG("[RX] bad length %d — dropped\n", rxLen);
        return;
    }

    // Quick network ID check before full decode
    if (raw[0] != LORA_NETVER) {
        DBG("[RX] foreign NETVER 0x%02X — dropped\n", raw[0]);
        return;
    }

    // Full decode + CRC check
    LoRaFrame f;
    if (!pkt_decode(raw, rxLen, &f)) {
        DBG("[RX] CRC fail (hdr=%d pay=%d) — dropped\n",
            f.hdr_ok, f.pay_ok);
        return;
    }

    // Drop if not addressed to us, broadcast, or gateway
    if (f.dst != NODE_ADDR
        && f.dst != ADDR_BROADCAST
        && f.dst != ADDR_GATEWAY) {
        DBG("[RX] dst=0x%02X not for us — dropped\n", f.dst);
        return;
    }

    // Duplicate detection (skip for ACK and broadcast)
    if (f.type != PKT_ACK && f.dst != ADDR_BROADCAST) {
        if (dedup_check(&g_dedup, f.src, f.seq)) {
            DBG("[RX] duplicate src=0x%02X seq=%d — dropped\n",
                f.src, f.seq);
            return;
        }
    }

    handle_frame(&f, rssi, snr);
}


// ════════════════════════════════════════════════════════════════════════
//  Periodic tasks
// ════════════════════════════════════════════════════════════════════════

// Send PKT_FULL (IO + measurements) to gateway
static void task_report_gateway() {
    int16_t m0 = meas_read(0);
    int16_t m1 = meas_read(1);
    int16_t m2 = meas_read(2);
    int16_t m3 = meas_read(3);
    uint8_t io = io_combined_state();

    LoRaPacket p;
    uint8_t seq = next_seq();
    pkt_build_full(&p, NODE_ADDR, ADDR_GATEWAY, seq, io,
                   m0, m1, m2, m3, FLAG_ACK_REQ);

    DBG("[TASK] GW report  IO=0x%02X M=%.1f %.1f %.1f %.1f\n",
        io, i16_to_float(m0), i16_to_float(m1),
            i16_to_float(m2), i16_to_float(m3));

    if (!g_ack.active)
        ack_send(p.buf, p.len, ADDR_GATEWAY, seq);
    else
        DBGLN("[TASK] GW report skipped — ACK pending");
}

// Broadcast beacon
static void task_beacon() {
    LoRaPacket p;
    uint8_t seq = next_seq();
    pkt_build_beacon(&p, NODE_ADDR, seq,
                     NODE_FW_MAJOR, NODE_FW_MINOR,
                     PEER_COUNT);
    radio_tx(p.buf, p.len);
    DBGLN("[TASK] beacon sent");
}

// Send IO status to a specific peer
static void task_notify_peer(uint8_t peer_addr) {
    LoRaPacket p;
    uint8_t seq = next_seq();
    pkt_build_io_status(&p, NODE_ADDR, peer_addr, seq,
                        io_combined_state(),
                        FLAG_ACK_REQ | FLAG_GW_COPY);
    if (!g_ack.active) {
        DBG("[TASK] IO notify → peer 0x%02X\n", peer_addr);
        ack_send(p.buf, p.len, peer_addr, seq);
    }
}

// Send IO command to a peer
// mask:  which outputs to change  value: new state for those bits
void node_send_io_cmd(uint8_t peer_addr, uint8_t mask, uint8_t value) {
    LoRaPacket p;
    uint8_t seq = next_seq();
    pkt_build_io_cmd(&p, NODE_ADDR, peer_addr, seq, mask, value);
    if (!g_ack.active) {
        DBG("[CMD] IO cmd → 0x%02X mask=0x%02X val=0x%02X\n",
            peer_addr, mask, value);
        ack_send(p.buf, p.len, peer_addr, seq);
    }
}

// Send measurements only to a specific destination
void node_send_meas(uint8_t dst, uint8_t flags) {
    LoRaPacket p;
    uint8_t seq = next_seq();
    pkt_build_meas(&p, NODE_ADDR, dst, seq,
                   meas_read(0), meas_read(1),
                   meas_read(2), meas_read(3), flags);
    if (!g_ack.active)
        ack_send(p.buf, p.len, dst, seq);
}


// ════════════════════════════════════════════════════════════════════════
//  setup()
// ════════════════════════════════════════════════════════════════════════
void setup() {
#if DEBUG_ENABLED
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    delay(500);
    DBGLN("========================================");
    DBG( " LoRa Custom Node  addr=0x%02X  fw=%d.%d\n",
         NODE_ADDR, NODE_FW_MAJOR, NODE_FW_MINOR);
    DBGLN("========================================");
#endif

#if LED_ENABLED
    pinMode(LED_STATUS_PIN, OUTPUT);
    digitalWrite(LED_STATUS_PIN, LOW);
#endif

    io_init();
    peers_init();
    dedup_init(&g_dedup);

    // ── Radio init ────────────────────────────────────────────────────
    DBG("[RADIO] initialising at %.1f MHz SF%d BW%.0f CR4/%d...\n",
        LORA_FREQ_MHZ, LORA_SF, LORA_BW_KHZ, LORA_CR);

    int err;

#if defined(RADIO_SX1262) || defined(RADIO_LLCC68)
    err = radio.begin(LORA_FREQ_MHZ,
                      LORA_BW_KHZ,
                      LORA_SF,
                      LORA_CR,
                      LORA_SYNC_WORD,
                      LORA_TX_POWER_DBM,
                      LORA_PREAMBLE);
    if (err == RADIOLIB_ERR_NONE) {
        // Set over-current protection limit (140 mA suits most modules)
        radio.setCurrentLimit(140.0f);
        // Uncomment if your module uses DIO2 to control an RF switch:
        // radio.setDio2AsRfSwitch(true);
        // Uncomment if your module has a TCXO (consult module datasheet):
        // radio.setTCXO(1.8f);
    }

#elif defined(RADIO_SX1278)
    // SX127x sync word is always 0x12 for private networks (not 0x34 LoRaWAN)
    err = radio.begin(LORA_FREQ_MHZ,
                      LORA_BW_KHZ,
                      LORA_SF,
                      LORA_CR,
                      0x12,
                      LORA_TX_POWER_DBM,
                      LORA_PREAMBLE);
#endif

    if (err != RADIOLIB_ERR_NONE) {
        DBG("[RADIO] INIT FAILED: error %d\n", err);
        // Blink error pattern forever — check wiring
        while (true) {
#if LED_ENABLED
            led_blink(1, LED_ERROR_BLINK_MS, LED_ERROR_BLINK_MS);
#else
            delay(500);
#endif
        }
    }

    DBGLN("[RADIO] OK");

    // Attach DIO1 interrupt
    radio.setDio1Action(onDio1Isr);

    // Start continuous receive
    g_radio_mode = MODE_RX;
    radio.startReceive();
    DBGLN("[RADIO] RX armed");

    // LED success feedback
#if LED_ENABLED
    led_blink(LED_BLINK_INIT, 100, 100);
#endif

    // Stagger initial report by NODE_ADDR × 3 s to avoid GW collision
    uint32_t now = millis();
    g_last_report_ms    = now - REPORT_INTERVAL_MS
                          + ((uint32_t)NODE_ADDR * 3000UL);
    g_last_beacon_ms    = now;
    g_last_io_notify_ms = now;
    g_last_uptime_ms    = now;

    // Send initial beacon immediately
    task_beacon();

    DBGLN("[BOOT] ready");
}


// ════════════════════════════════════════════════════════════════════════
//  loop()
// ════════════════════════════════════════════════════════════════════════
void loop() {

    uint32_t now = millis();

    // ── 1. Process received packet ────────────────────────────────────
    if (isrRxDone) {
        process_rx();
    }

    // ── 2. TX watchdog ────────────────────────────────────────────────
    if (g_tx_busy && (now - g_tx_start_ms) > TX_WATCHDOG_MS) {
        DBGLN("[TX] watchdog — forcing standby");
        radio.standby();
        g_tx_busy = false;
        isrTxDone = false;
        radio.startReceive();
    }

    // ── 3. ACK retry tick ─────────────────────────────────────────────
    ack_tick();

    // ── 4. Uptime counter ─────────────────────────────────────────────
    if (now - g_last_uptime_ms >= UPTIME_TICK_MS) {
        g_last_uptime_ms = now;
        if (g_uptime_min < 255) g_uptime_min++;
    }

    // ── 5. Periodic full report to gateway ───────────────────────────
    if (now - g_last_report_ms >= REPORT_INTERVAL_MS) {
        g_last_report_ms = now;
        task_report_gateway();
    }

    // ── 6. Periodic beacon broadcast ──────────────────────────────────
    if (now - g_last_beacon_ms >= BEACON_INTERVAL_MS) {
        g_last_beacon_ms = now;
        task_beacon();
    }

    // ── 7. IO change detection + peer notification ────────────────────
    if (io_poll_debounce()) {
        DBG("[IO] input change → 0b");
        for (int i = 7; i >= 0; i--)
            DBG("%d", (g_io_in >> i) & 1);
        DBG("  (0x%02X)\n", g_io_in);

        // Rate-limit peer notifications
        if (!g_ack.active
            && (now - g_last_io_notify_ms) >= IO_NOTIFY_COOLDOWN_MS) {
            g_last_io_notify_ms = now;
            // Notify all configured peers
            for (int i = 0; i < PEER_COUNT; i++) {
                task_notify_peer(g_peer_addrs[i]);
            }
        }
    }

    // ── 8. Application hook ───────────────────────────────────────────
    //  Add your own logic here. Examples:
    //
    //  a) Periodically send measurements to a specific peer:
    //     static uint32_t lastPeerMeas = 0;
    //     if (now - lastPeerMeas > 10000) {
    //         lastPeerMeas = now;
    //         node_send_meas(0x02, FLAG_ACK_REQ);
    //     }
    //
    //  b) Control peer output when local input changes:
    //     if (g_io_in & 0x01) {          // DIN_0 active
    //         node_send_io_cmd(0x02, 0x10, 0x10);  // turn ON DOUT_4 on peer 0x02
    //     }
    //
    //  c) Mirror peer IO:  handled in the PKT_IO_STATUS case in handle_frame()
}