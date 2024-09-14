#include "messages.h"
#include "Arduino.h"
#include "gps.h"

LoraMesher& radio = LoraMesher::getInstance();

uint32_t sentCounter;
uint32_t receiveCounter;
uint8_t encode_buffer[64];
uint8_t decode_buffer[64];
pb_ostream_t ostream;
pb_istream_t istream;
size_t written;
measurement data;
measurement dataReceived;
GPS gps_data;

TaskHandle_t receiveLoRaMessage_Handle = NULL;

void sendGPS(TinyGPSPlus gps) {
    if (gps.location.isValid()) {
        gps_data.latitude = gps.location.lat();
        gps_data.longitude = gps.location.lng();
        gps_data.altitude = gps.altitude.meters();
        gps_data.speed = gps.speed.kmph();
        gps_data.course = gps.course.deg();
        gps_data.satellites = gps.satellites.value();
        gps_data.hdop = gps.hdop.hdop();
        gps_data.timestamp = millis();
        encode_buffer[0] = (uint8_t) MSG_TYPE_GPS;
        ostream = pb_ostream_from_buffer(encode_buffer+1, sizeof(encode_buffer));
        int status = pb_encode(&ostream, &GPS_msg, &gps_data);
        if (!status) {
            Serial.println("Encoding failed");
        } else {
            written = ostream.bytes_written;
            radio.createPacketAndSend(BROADCAST_ADDR, encode_buffer, written+1);
        }
    }else{
        Serial.println("GPS not valid");
    }
}

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void print_measurement(measurement data) {
    Serial.printf("received message {timestamp: %d, value: %f, sensor: %d}\n", data.timestamp, data.value, data.sensor);
    receiveCounter++;
}

void decode_measurement(uint8_t* buffer, size_t len) {
    istream = pb_istream_from_buffer(buffer, len);
    int status = pb_decode(&istream, &measurement_msg, &dataReceived);
    if (!status) {
      Serial.println("Decoding failed");
    } else {
      print_measurement(dataReceived);
    }
}

void print_gps(GPS gps_data) {
    Serial.printf("received gps {latitude: %f, longitude: %f, altitude: %f, speed: %f, course: %f, satellites: %d, hdop: %f, timestamp: %d}\n", gps_data.latitude, gps_data.longitude, gps_data.altitude, gps_data.speed, gps_data.course, gps_data.satellites, gps_data.hdop, gps_data.timestamp);
}

void decode_gps(uint8_t* buffer, size_t len) {
    istream = pb_istream_from_buffer(buffer, len);
    int status = pb_decode(&istream, &GPS_msg, &gps_data);
    if (!status) {
      Serial.println("Decoding failed");
    } else {
        print_gps(gps_data);
    }
}

/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);

        //Iterate through all the packets inside the Received User Packets Queue
        while (radio.getReceivedQueueSize() > 0) {
            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());
            //Get the first element inside the Received User Packets Queue
            AppPacket<uint8_t>* packet = radio.getNextAppPacket<uint8_t>();
            Serial.printf("Packet arrived from %X with size %d\n", packet->src, packet->payloadSize);
            // get the message type
            uint8_t msgType = packet->payload[0];
            switch (msgType) {
                case MSG_TYPE_MEASUREMENT:
                    Serial.println("Received a measurement message");
                    decode_measurement(packet->payload+1, packet->payloadSize-1);
                    break;
                case MSG_TYPE_GPS:
                    Serial.println("Received a GPS message");
                    decode_gps(packet->payload+1, packet->payloadSize-1);
                    break;
                case MSG_TYPE_FILE:
                    Serial.println("Received a file message");
                    break;
                default:
                    Serial.println("Received an unknown message");
                    break;
            }

            //Delete the packet when used. It is very important to call this function to release the memory of the packet.
            radio.deletePacket(packet);
        }
    }
}

/**
 * @brief Create a Receive Messages Task and add it to the LoRaMesher
 *
 */
void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void*)1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS) {
        Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
    }

    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
}

/**
 * @brief Initialize LoRaMesher
 *
 */
void setupLoraMesher() {

    data = measurement_init_zero;
    dataReceived = measurement_init_zero;
    gps_data = GPS_init_zero;


    // Example on how to change the module. See LoraMesherConfig to see all the configurable parameters.
    LoraMesher::LoraMesherConfig config;
    config.module = LoraMesher::LoraModules::SX1262_MOD;
    config.freq = LoRa_frequency;
    config.loraCs = RADIO_CS_PIN;
    config.loraIrq = RADIO_BUSY_PIN;
    config.loraRst = RADIO_RST_PIN;
    config.loraIo1 = RADIO_DIO1_PIN;
    //Init the loramesher with a processReceivedPackets function
    radio.begin(config);


    //Create the receive task and add it to the LoRaMesher
    createReceiveMessages();

    //Start LoRaMesher
    radio.start();

    Serial.println("Lora initialized");
}

void sendDummyMeasurement(){
    sentCounter++;
    data.timestamp = millis();
    data.value = random(0, 100);
    data.sensor = Sensor(random(0, 3));
    encode_buffer[0] = (uint8_t) MSG_TYPE_MEASUREMENT;
    ostream = pb_ostream_from_buffer(encode_buffer+1, sizeof(encode_buffer));
    int status = pb_encode(&ostream, &measurement_msg, &data);
    if (!status) {
        Serial.println("Encoding failed");
    } else {
        //Create packet and send it.
        written = ostream.bytes_written;
        radio.createPacketAndSend(BROADCAST_ADDR, encode_buffer, written+1);
    }
}