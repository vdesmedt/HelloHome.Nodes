/*
 * Copyright (c) 2014 by Valery De Smedt <valery@indri.be>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

//This is made based on owsome work from Felix Risu
//It allows to reveive from RFM69 and communicate the payload added with the issuer nodeId to R-Pi
//It also allow for remote programming the Rfm69 node in the network
#define SSD1306 0
#define SSD1309 1
#define LCD_TYPE SSD1306

#include <version.h>
#include <Wire.h>
#if LCD_TYPE == SSD1306
#include <ssd1306.h>
#else
#include <ssd1309.h>
#endif
#include <Arduino.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

#define PIN_PRO 6
#define PIN_HW 5
#define NODEID 1
#define FREQUENCY RF69_868MHZ
#define ENCRYPTKEY "passiondesfruits" //(16 bytes of your choice - keep the same on all encrypted nodes)
#define ACK_TIME 50                   // # of ms to wait for an ack
#define TIMEOUT 3000
#define LOGSIZE 6

#ifdef __AVR_ATmega1284P__
#define LED 15 // Moteino MEGAs have LEDs on D15
#else
#define LED 9 // Moteinos have LEDs on D9
#endif
#define SERIAL_BAUD 115200

struct HHCInput
{
    uint16_t messageId;
    uint16_t destNode;
    byte msgLength;
    char message[64];
};
struct HHCOutput
{
    uint16_t srcNode;
    int16_t rssi;
    char message[64];
};

const char *rptType[] = {
    "PG", //Ping
    "NS", //Node started
    "NI", //Node info
    "EN", //Environemnt
    "PU", //Pulse
    "PS", //Push button
    "SW", //Switch
    "VR", //Vario
};
char *cmdType[] = {"NC", "RS", "RL", "PG"};

RFM69 radio;
uint8_t networkId = 0;
bool rfm69_hw = false;

char serOutBuffer[100];
uint8_t serOutBufferLength = 0;
struct HHCOutput *serOutData = (struct HHCOutput *)serOutBuffer;
char serInBuffer[100];
uint8_t serInBufferLength = 0;
struct HHCInput *serInData = (struct HHCInput *)serInBuffer;

//Logging
char netId[20]; //Node identification to be displayed
char screenLogLine[20];
char screenLog[LOGSIZE][20];
unsigned long screenLogTimes[LOGSIZE];
unsigned long lastLogMillis = 0;
int screenLogIndex = 0;

void addToScreenLog(const char *log);
bool draw(void);
bool readDipSwitches();

void setup()
{
    pinMode(PIN_HW, INPUT_PULLUP);
    pinMode(PIN_PRO, INPUT_PULLUP);
    pinMode(LED, OUTPUT);
    readDipSwitches();

    radio.initialize(RF69_868MHZ, NODEID, networkId);
    radio.encrypt(ENCRYPTKEY);
    radio.setHighPower(rfm69_hw);
    snprintf(netId, 20, "%s-%s [%s]", rfm69_hw ? "HW" : "W", networkId == 50 ? "50(DEV)" : "51(PRO)", VERSION);

    Serial.begin(SERIAL_BAUD);
    while (!Serial)
        ;

    ssd1306_128x64_i2c_init();
    ssd1306_fillScreen(0x00);
    ssd1306_setFixedFont(ssd1306xled_font6x8);
}

void loop()
{
    if (radio.receiveDone())
    {
        digitalWrite(LED, HIGH);
        if (radio.DATALEN > 64)
        {
            serOutBufferLength = sprintf(serOutBuffer, "//ERR:Message.Size %u > 64 bytes", radio.DATALEN);
        }
        else
        {
            serOutData->srcNode = radio.SENDERID;
            serOutData->rssi = radio.RSSI;
            memcpy(serOutData->message, radio.DATA, radio.DATALEN);
            serOutBufferLength = radio.DATALEN + 4;
        }
        if (radio.ACKRequested())
            radio.sendACK();

        Serial.write(serOutBuffer, serOutBufferLength);
        Serial.println("");
        snprintf(screenLogLine, 20, "< %s %03hu %d ", rptType[serOutData->message[0] >> 2], serOutData->srcNode, serOutData->rssi);
        addToScreenLog(screenLogLine);
        digitalWrite(LED, LOW);
        serOutBufferLength = 0;
    }

    while (Serial.available() > 0)
    {
        serInBuffer[serInBufferLength++] = Serial.read();
        if (serInBufferLength >= 2 && serInBuffer[serInBufferLength - 2] == 13 && serInBuffer[serInBufferLength - 1] == 10)
        {
            digitalWrite(LED, HIGH);
            bool success = radio.sendWithRetry(serInData->destNode, serInData->message, serInData->msgLength, 3, 40);
            digitalWrite(LED, LOW);

            //Log
            snprintf(screenLogLine, 20, "%s %s %03hu %03d", success ? ">" : "x", cmdType[serInData->message[0] >> 2], serInData->destNode, success ? radio.RSSI : 0);
            addToScreenLog(screenLogLine);

            //Report to Gateway
            serOutData->srcNode = NODEID;
            serOutData->rssi = success ? radio.RSSI : 0;
            serOutData->message[0] = 0xFF;
            memcpy((serOutData->message) + 1, &(serInData->messageId), 2);
            serOutData->message[3] = success ? 1 : 0;
            Serial.write(serOutBuffer, 8);
            Serial.println("");
            Serial.flush();
            serInBufferLength = 0;
        }
    }

    //Detect dip switch changes
    if (readDipSwitches())
    {
        radio.setNetwork(networkId);
        radio.setHighPower(rfm69_hw);
        snprintf(netId, 20, "%s-%s [%s]", rfm69_hw ? "HW" : "W", networkId == 50 ? "50(DEV)" : "51(PRO)", VERSION);
    }
    draw();
}

bool readDipSwitches()
{
    uint8_t nnid = digitalRead(PIN_PRO) == LOW ? 51 : 50;
    bool nhw = digitalRead(PIN_HW) == LOW;
    if (nnid != networkId || nhw != rfm69_hw)
    {
        delay(50); //Debounce
        networkId = digitalRead(PIN_PRO) == LOW ? 51 : 50;
        rfm69_hw = digitalRead(PIN_HW) == LOW;
        return true;
    }
    return false;
}

void addToScreenLog(const char *log)
{
    strncpy(screenLog[screenLogIndex], log, 20);
    screenLogTimes[screenLogIndex] = millis();
    screenLogIndex = (screenLogIndex + 1) % LOGSIZE;
}

unsigned long lastPrint = millis();
bool draw(void)
{
    //Draw only every 500ms
    if (millis() - lastPrint < 1000)
        return false;

    ssd1306_printFixed(0, 0, netId, STYLE_NORMAL);
    ssd1306_printFixed(0, 8, "      d mt rfi rss", STYLE_NORMAL);

    int logIndex = 0;
    for (logIndex = 1; logIndex <= LOGSIZE; logIndex++)
    {
        int i = (screenLogIndex - logIndex + 2 * LOGSIZE) % LOGSIZE;
        unsigned long s = (millis() - screenLogTimes[i]) / 1000L;
        uint8_t sec = (uint8_t)(s % 60L);
        uint8_t min = (uint8_t)(((s - sec) / 60L) % 60L);
        uint8_t hou = (uint8_t)(((s - sec - 60 * min) / 3600L));

        if (hou > 0)
            snprintf(screenLogLine, 20, "%02u:%02u", hou, min);
        else
            snprintf(screenLogLine, 20, "%02u:%02u", min, sec);

        ssd1306_printFixed(0, (LOGSIZE - logIndex + 2) * 8, screenLogLine, STYLE_NORMAL);
        ssd1306_printFixed(35, (LOGSIZE - logIndex + 2) * 8, screenLog[i], STYLE_NORMAL);
    }
    lastPrint = millis();
    return true;
}
