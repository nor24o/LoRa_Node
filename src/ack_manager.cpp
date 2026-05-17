#include "ack_manager.h"
#include "radio_driver.h"
#include "debug.h"
#include "node_config.h"

void AckManager::send(const uint8_t *buf, uint8_t len,
                       uint8_t dst, uint8_t seq, RadioDriver &radio) {
    if (_p.active) DBGLN("[ACK] warning: overwriting pending slot");
    _p.active      = true;
    _p.dst         = dst;
    _p.seq         = seq;
    _p.retriesLeft = ACK_RETRY_COUNT;
    _p.deadlineMs  = millis() + ACK_TIMEOUT_MS;
    _p.pktLen      = len;
    memcpy(_p.pkt, buf, len);
    radio.tx(buf, len);
}

void AckManager::received(uint8_t src, uint8_t seq) {
    if (_p.active && _p.dst == src && seq_equal(_p.seq, seq)) {
        DBG("[ACK] confirmed from 0x%02X seq=%d  retries_used=%d\n",
            src, seq, ACK_RETRY_COUNT - _p.retriesLeft);
        _p.active = false;
    }
}

void AckManager::tick(RadioDriver &radio) {
    if (!_p.active) return;
    // Unsigned subtraction wraps — if deadline hasn't passed, difference < 0x80000000
    if ((millis() - _p.deadlineMs) < 0x80000000UL) return;

    if (_p.retriesLeft == 0) {
        DBG("[ACK] give up dst=0x%02X seq=%d\n", _p.dst, _p.seq);
        _p.active = false;
        return;
    }

    _p.retriesLeft--;
    uint32_t backoff = ACK_RETRY_BACKOFF_MS
                       * (uint32_t)(ACK_RETRY_COUNT - _p.retriesLeft);
    DBG("[ACK] retry %d to 0x%02X seq=%d (backoff %lu ms)\n",
        ACK_RETRY_COUNT - _p.retriesLeft, _p.dst, _p.seq, backoff);
    delay(backoff);
    _p.deadlineMs = millis() + ACK_TIMEOUT_MS;
    radio.tx(_p.pkt, _p.pktLen);
}
