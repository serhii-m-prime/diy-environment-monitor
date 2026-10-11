#pragma once
#include <cstdint>

namespace monitor {
// GPIO edge notifications wake the worker. Poll only while the existing 50 ms
// button / 20 ms PIR debouncers can still change state; held levels do not poll.
class InputWaitPolicy {
public:
    uint32_t observe(uint32_t now, uint8_t rawLevels) {
        if (!seen_ || rawLevels != raw_) {
            seen_ = true; raw_ = rawLevels; changed_ = now;
        }
        return uint32_t(now - changed_) < 70 ? 10 : 1000;
    }
    // Opposite-level wakeup avoids continuously waking on a held button/PIR.
    static bool wakeOnHigh(bool rawHigh) { return !rawHigh; }
private:
    uint32_t changed_ = 0;
    uint8_t raw_ = 0;
    bool seen_ = false;
};
} // namespace monitor
