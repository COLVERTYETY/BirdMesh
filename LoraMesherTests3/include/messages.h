#ifndef MESSAGES_H
#define MESSAGES_H

#include "pb_encode.h"
#include "pb_decode.h"
#include "message.pb.h"
#include "LoraMesher.h"
#include "gps.h"

#define LoRa_frequency            915.0
#define RADIO_SCLK_PIN              (12)
#define RADIO_MISO_PIN              (13)
#define RADIO_MOSI_PIN              (11)
#define RADIO_CS_PIN                (10)
#define RADIO_DIO0_PIN               (-1)
#define RADIO_RST_PIN               (5)
#define RADIO_DIO1_PIN              (1)
#define RADIO_BUSY_PIN              (4)

extern LoraMesher& radio;

enum msgs_types {
    MSG_TYPE_MEASUREMENT = 0,
    MSG_TYPE_GPS = 1,
    MSG_TYPE_FILE = 2,
};
extern uint32_t sentCounter;
extern uint32_t receiveCounter;
extern uint8_t encode_buffer[64];
extern uint8_t decode_buffer[64];
extern pb_ostream_t ostream;
extern pb_istream_t istream;
extern size_t written;
extern measurement data;
extern measurement dataReceived;
extern GPS gps_data;

void sendGPS(TinyGPSPlus gps);

void print_measurement(measurement data);
void decode_measurement(uint8_t* buffer, size_t len);
void print_gps(GPS gps_data);
void decode_gps(uint8_t* buffer, size_t len);

void processReceivedPackets();
void setupLoraMesher();


void sendDummyMeasurement();

#endif // MESSAGES_H