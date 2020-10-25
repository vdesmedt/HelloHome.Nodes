#include <Arduino.h>

#define CONFIG_VERSION 1
typedef struct NodeFlashConfig {
  uint8_t confVer; //Config version, used to understand is whazt is red from flash is ok
  uint16_t nodeId;
  uint16_t features;
  uint16_t startCount;  
  uint8_t nodeInfoFreq;
  uint8_t environmentFreq;
} NodeFlashConfig;
NodeFlashConfig flashConfig;

bool sendData(const void *data, size_t dataSize, bool sleep = false);
void(* resetFunc) (void) = 0; //declare reset function @ address 0
void pulse_ISR();
void radioInit(uint16_t nodeId);
bool WaitRf(int milliseconds);

#define DEBUG true
#if DEBUG
  const char MSG_WAIT_RF[] = "Waiting for RF response";
  const char MSG_FLASH_INIT[]  = "Flash init...";
  const char MSG_OK[] = "OK\n";
  const char MSG_NOK[] = "NOK\n";
  const char MSG_NOK_RESTART[] = "Restarting!\n";
  const char MSG_READ_CONFIG[] = "reading config from flash...";
  const char MSG_NOCONFIG_OR_NEWVERSION[] = "No config found or new version detected. Load to default\n";
  const char MSG_CONFIG_FOUND[] = "Config NodeId %d\n";
  const char MSG_NODECONFIG_EXPECTED_D[] = "Expected NodeConfig (%d bytes) but received %d bytes.\n";
  const char MSG_SIGNATURE_MISSMATCH[] = "Config received for another signature.\n";
  const char MSG_NEW_CONFIG_DD[] = "New node/feature config received: %d/%d\n";
  const char MSG_RESTART_COMMAND_RECEIVED[] = "Restart command detected\n";
  const char MSG_UNKNOWN_MSG_TYPE_D[] = "Unkown messaqe type %d received\n";
  const char MSG_RF_INIT_DDS[] = "Rf initialized NodeId/Network/HighPow %d/%d/%s\n";
  const char MSG_SAVED_CONFIG_DDDDDD[] = "Saved config confVersion/features/nodeId/startCount/niFreq/envFreq %d/%d/%d/%d/%d/%d\n";
  const char MSG_BATT_ENABLED[] = "Battery features Enabled\n";
  const char MSG_PULSE_ENABLED[] = "Pulse-%d Enabled\n";
  const char MSG_ENV_ENABLED[] = "Environment Monitoring Enabled\n";
  const char MSG_SEND_MSG_D[] = "Will send Msg (%d)...";
  #define DEB_BUFFER_LEN 100
  #define debug_printa(msg, ...) \
              do { \
                char buffer[DEB_BUFFER_LEN]; \
                snprintf(buffer, DEB_BUFFER_LEN, (const char *)msg, __VA_ARGS__); \
                Serial.print(buffer); \
            } while (0)
  #define debug_print(msg) Serial.print((const char *)msg)
#else
  const char MSG_WAIT_RF[] PROGMEM = "";
  const char MSG_FLASH_INIT[] PROGMEM = "";
  const char MSG_OK[] PROGMEM = "";
  const char MSG_NOK[] PROGMEM = "";
  const char MSG_NOK_RESTART[] PROGMEM = "";
  const char MSG_READ_CONFIG[] PROGMEM = "";
  const char MSG_NOCONFIG_OR_NEWVERSION[] PROGMEM = "";
  const char MSG_CONFIG_FOUND[] = "";
  const char MSG_NODECONFIG_EXPECTED_D[] PROGMEM = "";
  const char MSG_SIGNATURE_MISSMATCH[] PROGMEM = "";
  const char MSG_NEW_CONFIG_DD[] PROGMEM = "";
  const char MSG_RESTART_COMMAND_RECEIVED[] PROGMEM = "";
  const char MSG_UNKNOWN_MSG_TYPE_D[] PROGMEM = "";
  #define debug_printa(msg, ...)
  #define debug_print(msg)
#endif

//Moteino specifics
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif