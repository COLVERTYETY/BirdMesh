#ifndef PMU_H
#define PMU_H

#include "XPowersLib.h"
#define I2C1_SDA                    42
#define I2C1_SCL                    41
#define PMU_IRQ                     40
#define PMU_WIRE_PORT   Wire1

extern XPowersLibInterface *PMU;

void setPmuFlag();

bool initPMU();

#endif // PMU_H