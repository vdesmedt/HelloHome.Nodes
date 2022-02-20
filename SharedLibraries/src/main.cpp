#include <Arduino.h>
#include "HHLogger/HHLogger.h"
#include "HHCentral/HHCentral.h"
#include "HHMessages/HHMessages.h"

HHLogger *logger;
HHCentral *hhCentral;

uint16_t expectedDeviceID = 0xEF30;
SPIFlash flash(SS_FLASHMEM, expectedDeviceID);

void setup()
{
    Serial.begin(115200);
    Serial.print("Start...");

    logger = new HHLogger();
    hhCentral = new HHCentral(logger, NodeType::Simulator, "1.0", HHEnv::Dev);
    hhCentral->connect();

    if (flash.initialize())
    {
        Serial.println("Init OK!");
    }
    else
    {
        Serial.print("Init FAIL, expectedDeviceID(0x");
        Serial.print(expectedDeviceID, HEX);
        Serial.print(") mismatched the read value: 0x");
        Serial.println(flash.readDeviceId(), HEX);
    }

    Serial.println("\n************************");
    Serial.println("Available operations:");
    Serial.println("'c' - read flash chip's deviceID 10 times to ensure chip is present");
    Serial.println("'d' - dump first 256 bytes on the chip");
    Serial.println("'e' - erase entire flash chip");
    Serial.println("'i' - read deviceID");
    Serial.println("************************\n");
    delay(1000);
}

char input = 0;
void loop()
{
    Command *cmd;
    cmd = hhCentral->check();
    if (cmd != nullptr)
    {
        switch (cmd->msgType)
        {
        case CMD_NODECONFIG:
            break;
        }
    }

    if (Serial.available() > 0)
    {
        input = Serial.read();
        if (input == 'd') // d=dump flash area
        {
            Serial.println("Flash content as from 0x8000:");
            uint16_t counter = 0x8000;
            while (counter <= 0x8100)
            {
                Serial.print(flash.readByte(counter++), HEX);
                Serial.print('.');
            }

            Serial.println();
        }
        else if (input == 'c')
        {
            Serial.print("Checking chip is present ... ");
            uint16_t deviceID = 0;
            for (uint8_t i = 0; i < 10; i++)
            {
                uint16_t idNow = flash.readDeviceId();
                if (idNow == 0 || idNow == 0xffff || (i > 0 && idNow != deviceID))
                {
                    deviceID = 0;
                    break;
                }
                deviceID = idNow;
            }
            if (deviceID != 0)
            {
                Serial.print("OK, deviceID=0x");
                Serial.println(deviceID, HEX);
            }
            else
                Serial.println("FAIL, deviceID is inconsistent or 0x0000/0xffff");
        }
        else if (input == 'e')
        {
            Serial.print("Erasing Flash chip ... ");
            flash.chipErase();
            while (flash.busy())
                ;
            Serial.println("DONE");
        }
        else if (input == 'i')
        {
            Serial.print("DeviceID: ");
            Serial.println(flash.readDeviceId(), HEX);
        }
        else if (input >= 48 && input <= 57) // 0-9
        {
            Serial.print("\nWriteByte(");
            Serial.print(input);
            Serial.print(")");
            flash.writeByte(input - 48, millis() % 2 ? 0xaa : 0xbb);
        }
        else if(input == 'r') 
        {
            Serial.print("Node Id:");Serial.println(hhCentral->getRegisterValue(HHRegister::NodeId));
            Serial.print("Net Id:");Serial.println(hhCentral->getRegisterValue(HHRegister::NetworkId));
        }
        else if (input == 'u') 
        {
            hhCentral->setRegisterValue(HHRegister::NodeId, 54);
            hhCentral->setRegisterValue(HHRegister::NetworkId, 55);
            Serial.println("NodeId/Net set to 54/55");
        }
        else if(input == 's')
        {
            hhCentral->saveRegisterToFlash();
            Serial.println("Config stored in flash");
        }
    }
}