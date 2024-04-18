#ifndef DallasTemperature_STM32_h
#define DallasTemperature_STM32_h

#define DALLASTEMPLIBVERSION "3.8.1" // To be deprecated -> TODO remove in 4.0.0

#include <inttypes.h>
#include "OneWire_STM32.h" // Include the modified OneWire library for STM32

class DallasTemperature_STM32_h{
public:
    DallasTemperature();
    DallasTemperature(OneWire*);

    void setOneWire(OneWire*);

    void begin(void);

    uint8_t getDeviceCount(void);

    bool requestTemperaturesByIndex(uint8_t);

    float getTempCByIndex(uint8_t);

private:
    OneWire* _wire;
};

#endif