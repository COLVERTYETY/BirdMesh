#include "display.h"

DISPLAY_MODEL *u8g2 = nullptr;

void display_info() {

    //Set the routing table list that is being used and cannot be accessed (Remember to release use after usage)
    LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();

    char text[20];
    Serial.println("==================================");
    snprintf(text, 20, "%.2fdB %d->%d", radio.getSNR(), sentCounter%100,receiveCounter%100);
    Serial.println(text);
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_amstrad_cpc_extended_8f);
    u8g2->drawStr(0, 20, text);
    snprintf(text, 20, "Bat: %dmV", PMU->getBattVoltage());
    u8g2->drawStr(0, 30, text);
    snprintf(text, 20, "gps: %d", gps.satellites.value());
    u8g2->drawStr(0, 40, text);

    // format text with routing table
    Serial.println("");
    routingTableList->setInUse();
    for (int i = 0; i < radio.routingTableSize(); i++) {
        RouteNode* rNode = (*routingTableList)[i];
        NetworkNode node = rNode->networkNode;
        snprintf(text, 20, ("|%X(%d)->%X"), node.address, node.metric, rNode->via);
        Serial.println(text);
        u8g2->drawStr(0, 50 + i * 10, text);
    }
    Serial.println("==================================");

    //Release routing table list usage.
    routingTableList->releaseInUse();

    // Delete routing table list
    delete routingTableList;
    u8g2->sendBuffer();
}

void splashScreen(){
    // init the screen to use the display wire
    u8g2 = new DISPLAY_MODEL(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ I2C_SCL, /* data=*/ I2C_SDA);
    u8g2->begin();
    u8g2->clearBuffer();
    u8g2->setFlipMode(0);
    u8g2->setFontMode(1); // Transparent
    u8g2->setDrawColor(1);
    u8g2->setFontDirection(0);
    u8g2->setFont(u8g2_font_fur11_tf);
    u8g2->drawStr(0, 30, " cool LoRA Mesh");
    u8g2->sendBuffer();
    u8g2->setFont(u8g2_font_fur11_tf);
    delay(2000);
}

void display_record(int number) {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_fur11_tf);
    u8g2->drawStr(0, 30, "Recording...");
    char text[20];
    snprintf(text, 20, "File: %d.wav", number);
    u8g2->drawStr(0, 40, text);
    u8g2->sendBuffer();
}