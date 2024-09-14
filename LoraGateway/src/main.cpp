#include <Arduino.h>
#include "LoraMesher.h"
#include <SPI.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "message.pb.h"
#include "Wire.h"
#include <WiFi.h>
#include <simplePGSQL.h>

// wifi
const char ssid[] = "tiger123";             //  your network SSID (name)
const char pass[] = "12345679";             // your network password
const char user[] = "Birds";                // your database user
const char password[] = "BirdMesh";         // your database password
const char dbname[] = "BirdMeshDB";         // your database name

int WiFiStatus;
WiFiClient client;
IPAddress PGIP(10,0,0,72);  // IP of the MySQL *server* here
char buffer[1024];
PGconnection conn(&client, 0, 1024, buffer);
int pg_status = 0;

char query[256];
char INSERT_SQL_GPS[] = "INSERT INTO GPS (mac_address, timestamp, longitude, latitude, n_satellites) VALUES ('%X', NOW(), %f, %f, %d)";

char INSERT_SQL_MEASUREMENT[] = "INSERT INTO Measurement (mac_address, timestamp, measurement_type, measurement_value) VALUES ('%X', NOW(), %d, %f)";
char *pgmsg;


void checkConnection()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connection lost to the network");
        // reconnect
        WiFi.begin((char *)ssid, pass);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.println("Reconnecting to WiFi..");
        }
    }
}

void connectToDB()
{
  int rc_status = conn.status();
  char *c;
  while (rc_status != CONNECTION_OK) {
    delay(100);
    switch (rc_status) {
      case CONNECTION_NEEDED:
        Serial.println("Needs to init connection");
        rc_status = conn.setDbLogin(PGIP, user, "trust", dbname, "UTF8", 5432);
        delay(200);
        break;
      case CONNECTION_OK:
        Serial.println("Connection OK");
        break;
      case CONNECTION_BAD:
        Serial.println("Connection BAD");
        c=conn.getMessage();
        if (c) Serial.println(c);
        conn.close();
        rc_status = conn.status();
        break;
      case CONNECTION_AUTH_OK:
        Serial.println("Connection auth ok, waiting for backend");
        delay(20);
        rc_status = conn.status();
        break;
      case CONNECTION_AWAITING_RESPONSE:
        Serial.println("Connection awaiting response");
        delay(20);
        rc_status = conn.status();
        break;
      default:
        Serial.println("Unknown connection status");
        break;
    }
  }
}

// lora
#define LoRa_frequency            915.0
#define RADIO_SCLK_PIN              (12)
#define RADIO_MISO_PIN              (13)
#define RADIO_MOSI_PIN              (11)
#define RADIO_CS_PIN                (10)
#define RADIO_DIO0_PIN               (-1)
#define RADIO_RST_PIN               (5)
#define RADIO_DIO1_PIN              (1)
#define RADIO_BUSY_PIN              (4)

#include "XPowersLib.h"
#define I2C1_SDA                    42
#define I2C1_SCL                    41
#define PMU_IRQ                     40
#define PMU_WIRE_PORT   Wire1
XPowersLibInterface *PMU = NULL;
uint16_t last_received_mac;

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
uint32_t receiveCounter = 0;


// protobuf
uint8_t encode_buffer[64];
uint8_t decode_buffer[64];
pb_ostream_t ostream;
pb_istream_t istream;
size_t written;

measurement data = measurement_init_zero;
measurement dataReceived = measurement_init_zero;
GPS gps_data = GPS_init_zero;

enum msgs_types {
    MSG_TYPE_MEASUREMENT = 0,
    MSG_TYPE_GPS = 1,
    MSG_TYPE_FILE = 2,
};

bool pmuInterrupt;
void setPmuFlag()
{
    pmuInterrupt = true;
}

bool initPMU()
{
    if (!PMU) {
        PMU = new XPowersAXP2101(PMU_WIRE_PORT);
        if (!PMU->init()) {
            Serial.println("Warning: Failed to find AXP2101 power management");
            delete PMU;
            PMU = NULL;
        } else {
            Serial.println("AXP2101 PMU init succeeded, using AXP2101 PMU");
        }
    }

    if (!PMU) {
        PMU = new XPowersAXP192(PMU_WIRE_PORT);
        if (!PMU->init()) {
            Serial.println("Warning: Failed to find AXP192 power management");
            delete PMU;
            PMU = NULL;
        } else {
            Serial.println("AXP192 PMU init succeeded, using AXP192 PMU");
        }
    }

    if (!PMU) {
        return false;
    }

    PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, setPmuFlag, FALLING);

    if (PMU->getChipModel() == XPOWERS_AXP192) {

        PMU->setProtectedChannel(XPOWERS_DCDC3);

        // lora
        PMU->setPowerChannelVoltage(XPOWERS_LDO2, 3300);
        // gps
        PMU->setPowerChannelVoltage(XPOWERS_LDO3, 3300);
        // oled
        PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);

        PMU->enablePowerOutput(XPOWERS_LDO2);
        PMU->enablePowerOutput(XPOWERS_LDO3);

        //protected oled power source
        PMU->setProtectedChannel(XPOWERS_DCDC1);
        //protected esp32 power source
        PMU->setProtectedChannel(XPOWERS_DCDC3);
        // enable oled power
        PMU->enablePowerOutput(XPOWERS_DCDC1);

        //disable not use channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);

        PMU->disableIRQ(XPOWERS_AXP192_ALL_IRQ);

        PMU->enableIRQ(XPOWERS_AXP192_VBUS_REMOVE_IRQ |
                       XPOWERS_AXP192_VBUS_INSERT_IRQ |
                       XPOWERS_AXP192_BAT_CHG_DONE_IRQ |
                       XPOWERS_AXP192_BAT_CHG_START_IRQ |
                       XPOWERS_AXP192_BAT_REMOVE_IRQ |
                       XPOWERS_AXP192_BAT_INSERT_IRQ |
                       XPOWERS_AXP192_PKEY_SHORT_IRQ
                      );

    } else if (PMU->getChipModel() == XPOWERS_AXP2101) {

        //t-beam m.2 inface
        //gps
        PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO4);

        // lora
        PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO3);

        // In order to avoid bus occupation, during initialization, the SD card and QMC sensor are powered off and restarted
        if (ESP_SLEEP_WAKEUP_UNDEFINED == esp_sleep_get_wakeup_cause()) {
            Serial.println("Power off and restart ALDO BLDO..");
            PMU->disablePowerOutput(XPOWERS_ALDO1);
            PMU->disablePowerOutput(XPOWERS_ALDO2);
            PMU->disablePowerOutput(XPOWERS_BLDO1);
            delay(250);
        }

        // Sensor
        PMU->setPowerChannelVoltage(XPOWERS_ALDO1, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO1);

        PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO2);

        //Sdcard

        PMU->setPowerChannelVoltage(XPOWERS_BLDO1, 3300);
        PMU->enablePowerOutput(XPOWERS_BLDO1);

        PMU->setPowerChannelVoltage(XPOWERS_BLDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_BLDO2);

        //face m.2
        PMU->setPowerChannelVoltage(XPOWERS_DCDC3, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC3);

        PMU->setPowerChannelVoltage(XPOWERS_DCDC4, XPOWERS_AXP2101_DCDC4_VOL2_MAX);
        PMU->enablePowerOutput(XPOWERS_DCDC4);

        PMU->setPowerChannelVoltage(XPOWERS_DCDC5, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC5);


        //not use channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);
        // PMU->disablePowerOutput(XPOWERS_DCDC4);
        // PMU->disablePowerOutput(XPOWERS_DCDC5);
        PMU->disablePowerOutput(XPOWERS_DLDO1);
        PMU->disablePowerOutput(XPOWERS_DLDO2);
        PMU->disablePowerOutput(XPOWERS_VBACKUP);

        // Set constant current charge current limit
        PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

        // Set charge cut-off voltage
        PMU->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

        // Disable all interrupts
        PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
        // Clear all interrupt flags
        PMU->clearIrqStatus();
        // Enable the required interrupt function
        PMU->enableIRQ(
            XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
            XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
            XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
            XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
            // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
        );

    }

    PMU->enableSystemVoltageMeasure();
    PMU->enableVbusVoltageMeasure();
    PMU->enableBattVoltageMeasure();
    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU->disableTSPinMeasure();

    Serial.printf("=========================================\n");
    if (PMU->isChannelAvailable(XPOWERS_DCDC1)) {
        Serial.printf("DC1  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC1)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC1));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC2)) {
        Serial.printf("DC2  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC2)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC2));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC3)) {
        Serial.printf("DC3  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC3)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC3));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC4)) {
        Serial.printf("DC4  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC4)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC4));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC5)) {
        Serial.printf("DC5  : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_DCDC5)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_DCDC5));
    }
    if (PMU->isChannelAvailable(XPOWERS_LDO2)) {
        Serial.printf("LDO2 : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_LDO2)   ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_LDO2));
    }
    if (PMU->isChannelAvailable(XPOWERS_LDO3)) {
        Serial.printf("LDO3 : %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_LDO3)   ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_LDO3));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO1)) {
        Serial.printf("ALDO1: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO1)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO1));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO2)) {
        Serial.printf("ALDO2: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO2)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO2));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO3)) {
        Serial.printf("ALDO3: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO3)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO3));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO4)) {
        Serial.printf("ALDO4: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_ALDO4)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_ALDO4));
    }
    if (PMU->isChannelAvailable(XPOWERS_BLDO1)) {
        Serial.printf("BLDO1: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_BLDO1)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_BLDO1));
    }
    if (PMU->isChannelAvailable(XPOWERS_BLDO2)) {
        Serial.printf("BLDO2: %s   Voltage: %04u mV \n",  PMU->isPowerChannelEnable(XPOWERS_BLDO2)  ? "+" : "-",  PMU->getPowerChannelVoltage(XPOWERS_BLDO2));
    }
    Serial.printf("=========================================\n");


    // Set the time of pressing the button to turn off
    PMU->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = PMU->getPowerKeyPressOffTime();
    Serial.print("PowerKeyPressOffTime:");
    switch (opt) {
    case XPOWERS_POWEROFF_4S: Serial.println("4 Second");
        break;
    case XPOWERS_POWEROFF_6S: Serial.println("6 Second");
        break;
    case XPOWERS_POWEROFF_8S: Serial.println("8 Second");
        break;
    case XPOWERS_POWEROFF_10S: Serial.println("10 Second");
        break;
    default:
        break;
    }

    return true;
}

void log_measurement(measurement data) {
    snprintf(query, 256, INSERT_SQL_MEASUREMENT, last_received_mac, (int)data.sensor, data.value);
    conn.close();
    connectToDB();
    Serial.println(query);
    pg_status = conn.execute(query);
    if (pg_status < 0) {
        Serial.println("Error executing MEASUREMENT query");
    }
    int data_status = conn.getData();
    if (data_status < 0) {
        Serial.println("Error getting data");
    }
    if (data_status == 0) {
        Serial.println("no interesting data arrived");
    }
    if (data_status == PG_RSTAT_READY) {
        Serial.println("RTSATS READY");
    }
    if (data_status > 0) {
        pgmsg = conn.getMessage();
        Serial.println("current data status if some data arrived");
        Serial.println(data_status);
        if (pgmsg) Serial.println(pgmsg);
    }
    Serial.println(" ");
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
    //   print_measurement(dataReceived);
      log_measurement(dataReceived);
    }
}

    
void log_gps(GPS gps_data) {
    snprintf(query, 256, INSERT_SQL_GPS, last_received_mac, gps_data.longitude, gps_data.latitude, gps_data.satellites);
    conn.close();
    connectToDB();
    Serial.println(query);
    pg_status = conn.execute(query);
    if (pg_status < 0) {
        Serial.println("Error executing GPS query");
    }
    int data_status = conn.getData();
    if (data_status < 0) {
        Serial.println("Error getting data");
    }
    if (data_status == 0) {
        Serial.println("no interesting data arrived");
    }
    if (data_status > 0) {
        Serial.println("current data status if some data arrived");
        Serial.println(data_status);
        pgmsg = conn.getMessage();
        if (pgmsg) Serial.println(pgmsg);
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
        log_gps(gps_data);
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
            last_received_mac= packet->src;
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

TaskHandle_t receiveLoRaMessage_Handle = NULL;

/**
 * @brief Create a Receive Messages Task and add it to the LoRaMesher
 *
 */
void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void*) 1,
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


void setup() {
    Serial.begin(115200);
    Wire1.begin(I2C1_SDA, I2C1_SCL);
    initPMU();
    setupLoraMesher();

    // wifi
    WiFi.begin((char *)ssid, pass);
    WiFi.setAutoReconnect(true);
    client.setTimeout(100);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
    Serial.println(WiFi.localIP());
    connectToDB();
    Serial.println("Connected to the database");
    Serial.println("Board Initialized");
}

void printRoutingTable() {

    //Set the routing table list that is being used and cannot be accessed (Remember to release use after usage)
    LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();

    routingTableList->setInUse();
    char text[20];
    snprintf(text, 20, "%.2fdB %d->%d", radio.getSNR(), dataCounter,receiveCounter);
    Serial.println("==================================");
    Serial.println(text);
    Serial.println("");
    for (int i = 0; i < radio.routingTableSize(); i++) {
        RouteNode* rNode = (*routingTableList)[i];
        NetworkNode node = rNode->networkNode;
        snprintf(text, 20, ("|%X(%d)->%X"), node.address, node.metric, rNode->via);
        Serial.println(text);
    }
    Serial.println("==================================");

    //Release routing table list usage.
    routingTableList->releaseInUse();

    // Delete routing table list
    delete routingTableList;
}

void loop() {
    for (;;) {
        Serial.printf("Send packet %d\n", dataCounter);
        Serial.printf("receivedCounter: %d\n", receiveCounter);
        char addrStr[15];
        snprintf(addrStr, 15, "Id: %X\r\n", radio.getLocalAddress());
        Serial.println(addrStr);
        // print info
        printRoutingTable();
        dataCounter++;
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
        //Wait 30 seconds to send the next packet
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}