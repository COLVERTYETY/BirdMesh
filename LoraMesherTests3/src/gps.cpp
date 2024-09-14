#include "gps.h"
#include "Arduino.h"
#include "messages.h"

TinyGPSPlus gps;

void printGPS() {
    Serial.print("Location: ");
    if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(", ");
        Serial.print(gps.location.lng(), 6);
    } else {
        Serial.print("INVALID");
    }
    Serial.print("  Date/Time: ");
    if (gps.date.isValid()) {
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.year());
    } else {
        Serial.print("INVALID");
    }
    Serial.print(" ");
    if (gps.time.isValid()) {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(":");
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(":");
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(".");
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    } else {
        Serial.print("INVALID");
    }
    Serial.println();
}



void readGPS() {
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }
    if (gps.location.isValid()) {
        gps_data.latitude = gps.location.lat();
        gps_data.longitude = gps.location.lng();
        gps_data.altitude = gps.altitude.meters();
        gps_data.speed = gps.speed.kmph();
        gps_data.course = gps.course.deg();
        gps_data.satellites = gps.satellites.value();
        gps_data.hdop = gps.hdop.hdop();
        gps_data.timestamp = gps.time.value();
        printGPS();
        sendGPS(gps);
    }else if (gps.time.isValid() && gps.time.minute()%2 == 0) {
        Serial.print("GPS time valid, waiting for location...");
        Serial.print(gps.satellites.value());
        Serial.print(" ");
        Serial.print(gps.hdop.hdop());
        Serial.println();
    }
    if (millis() > 15000 && gps.charsProcessed() < 10) {
        Serial.println("No GPS detected: check wiring.");
    }
}

void gps_task(void*) {
    for (;;) {
        readGPS();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setupGPS() {
    // pinMode(GPS_WAKEUP_PIN, OUTPUT);
    // digitalWrite(GPS_WAKEUP_PIN, HIGH);
    // pinMode(GPS_1PPS_PIN, INPUT);
    delay(1500);
    Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    // create a task to read the GPS
    int res = xTaskCreate(
        gps_task,
        "GPS Task",
        4096,
        (void*)1, // Pass the address of the gps object as a parameter
        2,
        NULL);
    if (res != pdPASS) {
        Serial.printf("Error: GPS Task creation gave error: %d\n", res);
    }else{
        Serial.println("GPS Task created");
    }
    // delay(1000);
    Serial.println("GPS initialized");
}