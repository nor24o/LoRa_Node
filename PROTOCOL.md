# LoRa Custom Node — Protocol Reference

## Frame Layout

Every packet shares a **6-byte header** followed by an optional typed payload and a payload CRC.

```
Byte:  0        1        2        3        4        5      6 … n    n+1
      ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
      │NET·VER │  SRC   │  DST   │SEQ·FLG │TYPE·LEN│HDR_CRC │PAYLOAD │PAY_CRC │
      └────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
        1 byte   1 byte   1 byte   1 byte   1 byte   1 byte   0–9 B    0–1 B
                                                              (absent when PAYLEN=0)
```

---

## Bit-level header detail

```
Byte 0 — NET·VER
  Bit: 7   6   5   4   3   2   1   0
      ┌───┬───┬───┬───┬───┬───┬───┬───┐
      │      NET_ID (4 bits)  │VER(4b)│
      └───┴───┴───┴───┴───┴───┴───┴───┘
  NET_ID = 0xC   network discriminator — foreign-network packets are dropped
  VER    = 0x1   protocol version
  Combined: NETVER = 0xC1

Byte 1 — SRC       Source address
Byte 2 — DST       Destination address

Byte 3 — SEQ·FLG
  Bit: 7   6   5   4   3   2   1   0
      ┌───┬───┬───┬───┬───┬───┬───┬───┐
      │        SEQ  (6 bits)  │ A │ G │
      └───┴───┴───┴───┴───┴───┴───┴───┘
  SEQ  rolling counter 0–63 (wraps, used for dedup and ACK matching)
  A    FLAG_ACK_REQ  (1 = sender expects an ACK back)
  G    FLAG_GW_COPY  (1 = receiver must relay this packet to the gateway)

Byte 4 — TYPE·LEN
  Bit: 7   6   5   4   3   2   1   0
      ┌───┬───┬───┬───┬───┬───┬───┬───┐
      │  PACKET TYPE (4 bits) │PAYLEN │
      └───┴───┴───┴───┴───┴───┴───┴───┘
  TYPE   packet kind 0x0–0x7 (see table below)
  PAYLEN payload byte count 0–9

Byte 5 — HDR_CRC   CRC-8 (poly=0x07, init=0x00) over bytes 0–4
```

---

## Address Scheme

| Value    | Meaning                       |
|----------|-------------------------------|
| `0x00`   | Gateway                       |
| `0x01`–`0xFE` | Node (unique per device) |
| `0xFF`   | Broadcast (no ACK expected)   |

---

## Packet Type Reference

| TYPE | Name           | PAYLEN | Total bytes | Byte 4 |
|------|----------------|--------|-------------|--------|
| 0x0  | PKT_ACK        | 0      | 6           | `0x00` |
| 0x1  | PKT_IO_STATUS  | 1      | 8           | `0x11` |
| 0x2  | PKT_IO_CMD     | 2      | 9           | `0x22` |
| 0x3  | PKT_MEAS       | 8      | 15          | `0x38` |
| 0x4  | PKT_FULL       | 9      | 16          | `0x49` |
| 0x5  | PKT_BEACON     | 3      | 10          | `0x53` |
| 0x6  | PKT_PING       | 0      | 6           | `0x60` |
| 0x7  | PKT_PONG       | 3      | 10          | `0x73` |

---

## Individual Packet Diagrams

### PKT_ACK — 6 bytes

```
 0        1        2        3        4        5
┌────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xC1   │  SRC   │  DST   │SEQ·FLG │  0x00  │HDR_CRC │
└────────┴────────┴────────┴────────┴────────┴────────┘
                              └── SEQ echoes the original packet's SEQ
                                  TYPE=0x0, PAYLEN=0, flags=0
```
No payload. SEQ must mirror the sequence number of the packet being acknowledged.

---

### PKT_IO_STATUS — 8 bytes

```
 0        1        2        3        4        5        6        7
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xC1   │  SRC   │  DST   │SEQ·FLG │  0x11  │HDR_CRC │IO_STATE│PAY_CRC │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
                                                         Bit 7..0:
                                                         [DOUT3..0][DIN3..0]
```

IO_STATE bitmap:
```
  Bit:  7    6    5    4    3    2    1    0
       DOUT3 DOUT2 DOUT1 DOUT0 DIN3 DIN2 DIN1 DIN0
       └─────── outputs ──────┘└──────── inputs ──────┘
  1 = ON / asserted,  0 = OFF / idle
```

---

### PKT_IO_CMD — 9 bytes

```
 0        1        2        3        4        5        6        7        8
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xC1   │  SRC   │  DST   │SEQ·FLG │  0x22  │HDR_CRC │  MASK  │ VALUE  │PAY_CRC │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
                              FLAG_ACK_REQ always set
```

- **MASK**  — which output bits to touch (1 = change this output)
- **VALUE** — target state for masked bits (1 = ON, 0 = OFF)
- Unmasked bits are never changed.

---

### PKT_MEAS — 15 bytes

```
 0      1      2      3      4      5     6    7    8    9   10   11   12   13   14
┌──────┬──────┬──────┬──────┬──────┬─────┬────┬────┬────┬────┬────┬────┬────┬────┬─────┐
│0xC1  │ SRC  │ DST  │SEQ·FL│ 0x38 │HDR_C│  M0 (LE) │  M1 (LE) │  M2 (LE) │  M3 (LE) │PAY_C│
└──────┴──────┴──────┴──────┴──────┴─────┴────┴────┴────┴────┴────┴────┴────┴────┴─────┘
```

**M0–M3**: `int16` little-endian, real_value = raw / 10.0
- Example: 25.4 → stored as 254 (0x00FE), bytes: `FE 00`
- Example: –12.5 → stored as –125 (0xFF83), bytes: `83 FF`
- 0x0000 = channel disabled / unavailable

---

### PKT_FULL — 16 bytes  *(most common gateway report)*

```
 0      1      2      3      4      5      6     7    8    9   10   11   12   13   14   15
┌──────┬──────┬──────┬──────┬──────┬──────┬─────┬────┬────┬────┬────┬────┬────┬────┬────┬─────┐
│0xC1  │ SRC  │ DST  │SEQ·FL│ 0x49 │HDR_C │IO_ST│  M0 (LE) │  M1 (LE) │  M2 (LE) │  M3 (LE) │PAY_C│
└──────┴──────┴──────┴──────┴──────┴──────┴─────┴────┴────┴────┴────┴────┴────┴────┴────┴─────┘
                               FLAG_ACK_REQ set for GW reports
```

IO_STATE same bitmap as PKT_IO_STATUS.
M0–M3 same int16×10 encoding as PKT_MEAS.

---

### PKT_BEACON — 10 bytes

```
 0        1        2        3        4        5        6        7        8        9
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xC1   │  SRC   │  0xFF  │SEQ·FLG │  0x53  │HDR_CRC │FW_MAJOR│FW_MINOR│ PEERS  │PAY_CRC │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
            DST is always BROADCAST (0xFF). No ACK expected.
```

- **FW_MAJOR / FW_MINOR** — firmware version from node_config.h
- **PEERS** — number of configured peers (PEER_COUNT)

---

### PKT_PING — 6 bytes  *(gateway → node keepalive)*

```
 0        1        2        3        4        5
┌────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xC1   │  SRC   │  DST   │SEQ·FLG │  0x60  │HDR_CRC │
└────────┴────────┴────────┴────────┴────────┴────────┘
                              FLAG_ACK_REQ set — node must reply with PKT_PONG
```
No payload.

---

### PKT_PONG — 10 bytes  *(node → gateway reply)*

```
 0        1        2        3        4        5        6        7        8        9
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xC1   │  SRC   │  DST   │SEQ=echo│  0x73  │HDR_CRC │RSSI_H  │SNR_Q   │UP_MIN  │PAY_CRC │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
                   SEQ echoes the PING's SEQ
```

| Field    | Encoding                                       | Example                     |
|----------|------------------------------------------------|-----------------------------|
| RSSI_H   | `uint8` = \|received RSSI\| × 2               | –87 dBm → 174               |
| SNR_Q    | `int8`  = received SNR × 4                     | +6.25 dB → 25 ; –3 dB → –12|
| UP_MIN   | `uint8` uptime in minutes (wraps at 255 ≈ 4 h) | 120 min = 0x78              |

---

## CRC-8 Algorithm

Polynomial 0x07, init 0x00 (ITU/CCITT):

```c
uint8_t crc8(const uint8_t *d, uint8_t n) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}
```

- **HDR_CRC** covers bytes 0–4 (5 bytes)
- **PAY_CRC** covers payload bytes only (PAYLEN bytes starting at byte 6)
- PAY_CRC is absent when PAYLEN = 0

---

## Typical Exchange Flows

### Node → Gateway: periodic full report
```
Node ──[PKT_FULL, FLAG_ACK_REQ]──────────────────────► Gateway
Node ◄─[PKT_ACK]─────────────────────────────────────  Gateway

On timeout (no ACK within ACK_TIMEOUT_MS): retransmit up to ACK_RETRY_COUNT times
with ACK_RETRY_BACKOFF_MS × retry_number delay between attempts.
```

### Gateway → Node: keepalive check
```
Gateway ──[PKT_PING, FLAG_ACK_REQ]───────────────────► Node
Gateway ◄─[PKT_PONG]─────────────────────────────────  Node
```

### Node A input change → peer notification + gateway relay
```
Node A ──[PKT_IO_STATUS, FLAG_ACK_REQ | FLAG_GW_COPY]─► Node B
Node A ◄─[PKT_ACK]───────────────────────────────────── Node B

Node B ──[PKT_IO_STATUS, FLAG_ACK_REQ]────────────────► Gateway  (relay copy)
Node B ◄─[PKT_ACK]───────────────────────────────────── Gateway
```

### Gateway → Node: output command
```
Gateway ──[PKT_IO_CMD, FLAG_ACK_REQ]─────────────────► Node
Gateway ◄─[PKT_IO_STATUS (new state confirmation)]────  Node
```

### Node beacon (no ACK, no reply expected)
```
Node ──[PKT_BEACON, DST=0xFF]────────────────────────► All nodes & Gateway
```

---

## CAD (Channel Activity Detection) flow before every TX

```
        ┌─────────────┐
        │  Want to TX  │
        └──────┬───────┘
               │
               ▼
    ┌──── scanChannel() ────┐
    │                       │
    ▼ CLEAR                 ▼ BUSY
  Transmit           Exponential backoff
                     20ms << attempt (max 320ms)
                     + random(backoff/2)
                           │
                           └──► retry (up to CAD_TIMEOUT_MS total)
                                    │ timeout
                                    ▼
                               Force TX anyway
```

CAD is only performed on SX126x / LLCC68.  
SX127x (SX1278) does not support CAD via RadioLib — always transmits directly.
