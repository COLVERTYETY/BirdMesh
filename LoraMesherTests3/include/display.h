#ifndef DISPLAY_H
#define DISPLAY_H

#include <U8g2lib.h>
#include "messages.h"
#include "pmu.h"
#include "gps.h"

#define DISPLAY_MODEL U8G2_SSD1306_128X64_NONAME_F_HW_I2C
#define I2C_SDA                    17
#define I2C_SCL                    18

void splashScreen();
void display_info();
void display_record(int file_number);

#endif // DISPLAY_H