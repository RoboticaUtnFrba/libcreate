#include "create/encoder.h"

/**
 * Monitors a single wheel encoder and
 * accumulates delta ticks since the 
 * last time they were requested.
 */

namespace create {

Encoder::Encoder() {
    Encoder(0);
}

Encoder::Encoder(uint16_t _initCount) {
    setRange(0, 65535); // Create 2 Open Interface p.29
    initCount(_initCount);
    setReversed(false);
}

void Encoder::setRange(uint16_t _low,  uint16_t _high) {
    this->range = _high - _low + 1;
    this->lowThresh = _low + floor(this->range * 0.3);
    this->highThresh = _low + floor(this->range * 0.7);
}

void Encoder::initCount(uint16_t _startCount) {
    this->delta = 0;
    this->last = _startCount;
}

void Encoder::update(uint16_t _newCount) {
    int32_t _increment;

    if (this->last > this->highThresh &&
        _newCount < this->lowThresh) {
        // Wrapped around the upper limit
        _increment = _newCount + this->range - this->last;
    }
    else if (this->last < this->lowThresh &&
             _newCount > this->highThresh) {
        // Wrapped around the lower limit
        _increment = _newCount - this->range - this->last;
    }
    else
    {
        _increment = _newCount - this->last;
    }
    this->delta += _increment;
    this->last = _newCount;
}

void Encoder::setReversed(bool _isReversed) {
    this->isReversed = _isReversed;
}

int32_t Encoder::getDelta() {
    const int32_t _delta = this->delta;
    this->delta = 0;
    return this->isReversed ? -_delta : _delta;
}

} // namespace create