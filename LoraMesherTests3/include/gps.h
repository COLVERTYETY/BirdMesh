#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>
#include "message.pb.h"
#define GPS_RX_PIN                  9
#define GPS_TX_PIN                  8
#define GPS_WAKEUP_PIN              7
#define GPS_1PPS_PIN                6
#define GPS_BAUD_RATE               9600

extern TinyGPSPlus gps;

void setupGPS();
void readGPS();
void printGPS();
void gps_task(void*);

#endif // GPS_H