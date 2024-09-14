#include <Arduino.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "message.pb.h"
#include "LoraMesher.h"
#include <SPI.h>

#include "Wire.h"

#include "pmu.h"
#include "gps.h"
#include "messages.h"
#include "display.h"
#include "audio.h"

void setup() {
    Serial.begin(115200);
    Wire1.begin(I2C1_SDA, I2C1_SCL);
    initPMU();
    Serial.println("Starting OLED ...");
    splashScreen();
    Serial.println("Starting GPS ...");
    setupGPS();
    Serial.println("Starting Lora ...");
    setupLoraMesher();
    Serial.println("Starting Audio ...");
    setupAudio();
    Serial.println("Board Initialized");
}

void loop() {
    for (;;) {
        Serial.printf("Send packet %d\n", sentCounter);
        Serial.printf("receivedCounter: %d\n", receiveCounter);
        char addrStr[15];
        snprintf(addrStr, 15, "Id: %X\r\n", radio.getLocalAddress());
        Serial.println(addrStr);
        // print info
        display_info();
        // send dummy measurement
        sendDummyMeasurement();
        //Wait 5 seconds to send the next packet
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}