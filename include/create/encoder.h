#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <stdint.h>
#include <cmath>

namespace create {

class Encoder {
private:
    void initCount(uint16_t _startCount);
    void setReversed(bool _isReversed);
    // Getters
    uint32_t getRange() { return this->range; }
    uint16_t getHighThreshold() { return this->highThresh; }
    uint16_t getLowThreshold() { return this->lowThresh; }

    uint32_t range;
    uint16_t lowThresh;
    uint16_t highThresh;
    int32_t delta;
    uint16_t last;
    bool isReversed;
public:
    Encoder();
    Encoder(uint16_t _initCount);
    void setRange(uint16_t _low,  uint16_t _high);
    void update(uint16_t _newCount);
    int32_t getDelta();
};
} // namespace create

#endif //ENCODER_H