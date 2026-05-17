#include <Arduino.h>
#include "lora_node.h"

static LoRaNode node;

void setup() { node.begin(); }

void loop() {
    node.tick();

    // ── Application hook ──────────────────────────────────────────────────
    //  Add custom logic here after node.tick(). Examples:
    //
    //  a) Periodically send measurements to peer 0x02:
    //     static uint32_t t = 0;
    //     if (millis() - t > 10000) { t = millis(); node.sendMeas(0x02, FLAG_ACK_REQ); }
    //
    //  b) Turn on peer 0x02 DOUT_4 when our DIN_0 is active:
    //     node.sendIoCmd(0x02, 0x10, (node is missing inputState — call sendIoCmd directly);
    //
    //  c) Mirror remote IO: handle in PKT_IO_STATUS case inside lora_node.cpp
}
