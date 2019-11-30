#include <Arduino.h>
#include <SI7021.h>
#include <LowPower.h>
#include <RFM69.h>    
#include <RFM69_OTA.h>

#include <version.h>
#include <messages.h>

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
  const char MSG_NODECONFIG_EXPECTED_D[] = "Expected NodeConfig but received %d bytes.\n";
  const char MSG_SIGNATURE_MISSMATCH[] = "Config received for another signature.\n";
  const char MSG_NEW_CONFIG_DD[] = "New node/feature config received: %d/%d\n";
  const char MSG_RESTART_COMMAND_RECEIVED[] = "Restart command detected\n";
  const char MSG_UNKNOWN_MSG_TYPE_D[] = "Unkown messaqe type %d received\n";
  const char MSG_RF_INIT_DDS[] = "Rf initialized NodeId/Network/HighPow %d/%d/%s\n";
  const char MSG_SAVED_CONFIG_DDDDDD[] = "Saved config confVersion/features/nodeId/startCount/niFreq/envFreq %d/%d/%d/%d/%d/%d\n";
  const char MSG_VIN_ENABLED[] = "VIn Enabled\n";
  const char MSG_HAL1_ENABLED[] = "Hal1 Enabled\n";
  const char MSG_HAL2_ENABLED[] = "Hal2 Enabled\n";
  const char MSG_DRY1_ENABLED[] = "Dry1 Enabled\n";
  const char MSG_SI_ENABLED[] = "Si Enabled\n";
  const char MSG_BMP_ENABLED[] = "Bmp Enabled\n";
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


//RFM69
#define RF_ENCRYPT_KEY  "passiondesfruits"
#define RF_GTW_NODE_ID  254
#define RF_IS_RFM69HW_

//SPIFlash
#define FLASH_ADR	0x20000 //First memory availble after reserved for OTA update

//Battery
#define VIN_TRIGGER  A6    
#define VIN_MEASURE  A0
#define VIN_RATIO    2.0 //(4.7 + 4.7) / 4.7
#define VIN_VREF     3.3

//HAL and DRY contact
#define DRY_PIN       4
#define HAL1_PIN      5
#define HAL2_PIN      6

//Moteino specifics
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

RFM69 radio;
SI7021 siSensor;

bool displayOn = false;
bool hal1_interrupt = false;
bool hal2_interrupt = false;
bool dry_interrupt = false;
int wakeUpCount = 0;

PulseReport hal1Msg;
PulseReport hal2Msg;
PulseReport dryMsg;
NodeInfoReport nodeInfoMsg;
EnvironmentReport envMsg;

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

bool sendData(const void *data, size_t dataSize, bool sleep = true);
void(* resetFunc) (void) = 0; //declare reset function @ address 0
void intTest();
void radioInit(uint16_t nodeId);
bool WaitRf(int milliseconds);

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    while(!Serial) delay(10);
  #endif

  //Flash initialization
  debug_print(MSG_FLASH_INIT);
  SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
  if(!flash.initialize()) {
    debug_print(MSG_NOK_RESTART);    
    delay(1000);
    resetFunc();
  } else {
    debug_print(MSG_OK);
  }
  //Retreive unique Id from flash
  flash.readUniqueId();

  //Read & Update config (start count)
  debug_print(MSG_READ_CONFIG);	
  flash.readBytes(FLASH_ADR, &flashConfig, sizeof(NodeFlashConfig));
  flashConfig.startCount++;
  if(CONFIG_VERSION != flashConfig.confVer) {
    debug_print(MSG_NOCONFIG_OR_NEWVERSION);
    //First read or new version
    flashConfig.confVer = CONFIG_VERSION;
    flashConfig.features = 0;
    flashConfig.nodeId = 253;
    flashConfig.startCount = 1;
    flashConfig.environmentFreq = 0;
    flashConfig.nodeInfoFreq = 0;
  } else {
    debug_printa(MSG_CONFIG_FOUND, flashConfig.nodeId);
  }

  //Init radio
  radioInit(flashConfig.nodeId);
  //Send startup report
  NodeStartedReport nodeStartedMsg;
  nodeStartedMsg.startCount = flashConfig.startCount;
  strncpy(nodeStartedMsg.version, VERSION, 7);
  memcpy(nodeStartedMsg.signature, flash.UNIQUEID, 8);
  sendData(&nodeStartedMsg, sizeof(NodeStartedReport), false);
  //Wait for response (config)
  if(!WaitRf(10000)) {
    debug_print(MSG_NOK_RESTART);
    delay(1000); resetFunc();
  }
  
  //Check data if compatible with config
  if(radio.DATALEN != sizeof(NodeConfigCommand) || radio.DATA[0] != (2 + (0 << 2))) {
    debug_printa(MSG_NODECONFIG_EXPECTED_D, radio.DATALEN);
    delay(1000); resetFunc();
  }

  //Put response in config struct & ack  
  NodeConfigCommand receivedConfig;
  memcpy(&receivedConfig, (const void *)radio.DATA, sizeof(NodeConfigCommand));
  if(radio.ACKRequested()) radio.sendACK();
  
  //Check config is for right node by testing signature
  if(memcmp((const void *)&receivedConfig.signature, flash.UNIQUEID, 8)) {
    debug_print(MSG_SIGNATURE_MISSMATCH);debug_print(MSG_NOK_RESTART);
    delay(1000); resetFunc();
  }

  //Update flash config if needed (nodeId or features)
  if(flashConfig.nodeId != receivedConfig.newNodeId || flashConfig.features != receivedConfig.features) {
    debug_printa(MSG_NEW_CONFIG_DD, flashConfig.nodeId, flashConfig.features);
    flashConfig.nodeId = receivedConfig.newNodeId;
    radio.setAddress(flashConfig.nodeId);
    flashConfig.features = receivedConfig.features;
  }
  flashConfig.nodeInfoFreq = receivedConfig.nodeInfoFreq;
  flashConfig.environmentFreq = receivedConfig.environmentFreq;
  flash.blockErase4K(FLASH_ADR);
  flash.writeBytes(FLASH_ADR, &flashConfig, sizeof(NodeFlashConfig));
  debug_printa(MSG_SAVED_CONFIG_DDDDDD, flashConfig.confVer, flashConfig.features, flashConfig.nodeId, flashConfig.startCount, flashConfig.nodeInfoFreq, flashConfig.environmentFreq);

  //Sleep radio
  radio.sleep();

  pinMode(3, INPUT_PULLUP);
  //Configure Pin
  if(receivedConfig.features & FEAT_VIN) {
    debug_print(MSG_VIN_ENABLED);
    pinMode(VIN_TRIGGER, OUTPUT);
    pinMode(VIN_MEASURE, INPUT);
  }
  if(receivedConfig.features & FEAT_HAL1) {
    debug_print(MSG_HAL1_ENABLED);
    pinMode(HAL1_PIN, INPUT_PULLUP);
  }
  if(receivedConfig.features & FEAT_HAL2) {
    debug_print(MSG_HAL2_ENABLED);
    pinMode(HAL2_PIN, INPUT_PULLUP);
  }
  if(receivedConfig.features & FEAT_DRY) {
    debug_print(MSG_DRY1_ENABLED);
    pinMode(DRY_PIN, INPUT_PULLUP);
  }
  
  //Initialize SI7021 sensor if feature enabled
  if(receivedConfig.features & FEAT_SI7021) {
    debug_print(MSG_SI_ENABLED);
    siSensor.begin();
  }
  
  //Initialize messages
  hal1Msg.newPulses = hal2Msg.newPulses = dryMsg.newPulses = 0;
  hal1Msg.portNumber = PORTNUMBER_HAL1;hal2Msg.portNumber=PORTNUMBER_HAL2;dryMsg.portNumber=PORTNUMBER_DRY;
  envMsg.temperature = envMsg.humidity = envMsg.pressure = 0;
  nodeInfoMsg.sendErrorCount = nodeInfoMsg.vIn = 0;
}

void loop() {
  attachInterrupt(digitalPinToInterrupt(3), intTest, FALLING);
  if(DEBUG)
    Serial.flush();

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  detachInterrupt(digitalPinToInterrupt(3));

  wakeUpCount++;

  //Check if message for rf message
  if(radio.receiveDone()) {
    switch(radio.DATA[0]) {
      case 6 :
        debug_print(MSG_RESTART_COMMAND_RECEIVED);
        if(radio.ACKRequested())
          radio.sendACK();
        delay(100);
        resetFunc();        
        break;
      default :
        debug_printa(MSG_UNKNOWN_MSG_TYPE_D, radio.DATA[0]);
        break;
    }
    if(radio.ACKRequested())
      radio.sendACK();
  }

  //Check on pulse counters
  if(hal1_interrupt) {
    hal1Msg.newPulses++;
    if(sendData(&hal1Msg, sizeof(PulseReport)))
      hal1Msg.newPulses = 0;
  }
  if(hal2_interrupt) {
    hal2Msg.newPulses++;
    if(sendData(&hal2Msg, sizeof(PulseReport)))
      hal2Msg.newPulses = 0;
  }
  if(dry_interrupt) {
    dryMsg.newPulses++;
    if(sendData(&dryMsg, sizeof(PulseReport)))
      dryMsg.newPulses = 0;
  }

  //Calculate Temperature and Humidity every environmentFreq wakeup
  debug_printa("WC:%d ->", wakeUpCount);
  if(flashConfig.features & FEAT_SI7021 && flashConfig.environmentFreq > 0 &&  wakeUpCount % flashConfig.environmentFreq == 0) {
    debug_print(" SI7021");
    envMsg.temperature = siSensor.getCelsiusHundredths();
    envMsg.humidity = siSensor.getHumidityPercent();
    sendData((const void*)&envMsg, sizeof(envMsg));
  }

  //Measure battery voltage every 10 minutes
  if(flashConfig.features & FEAT_VIN && flashConfig.nodeInfoFreq > 0 && wakeUpCount % flashConfig.nodeInfoFreq == 0) { //75
    debug_print(" VIN");
    digitalWrite(VIN_TRIGGER, LOW);
    nodeInfoMsg.vIn = ((1.0 * analogRead(VIN_MEASURE)) / 1024.0 * VIN_VREF * VIN_RATIO) * 100;
    digitalWrite(VIN_TRIGGER, HIGH);
    sendData((const void*)&nodeInfoMsg, sizeof(nodeInfoMsg));
  }
  debug_print("\n");

  hal1_interrupt = hal2_interrupt = dry_interrupt = false;
}

void radioInit(uint16_t nodeId) {
  //Initialize Radio
  radio.initialize(RF69_868MHZ, nodeId, RF_NETWORK_ID);
  radio.encrypt(RF_ENCRYPT_KEY);
#ifdef RF_IS_RFM69HW
  radio.setHighPower();
  debug_printa(MSG_RF_INIT_DDS, nodeId, RF_NETWORK_ID, "HW");
#else
  debug_printa(MSG_RF_INIT_DDS, nodeId, RF_NETWORK_ID, "W");
#endif
}

bool WaitRf(int milliseconds) {
  int d = 100;
  int retryCount = milliseconds / d;
  debug_print(MSG_WAIT_RF);
  bool rd = radio.receiveDone();
  while(!rd && retryCount-- > 0) {
    debug_print(".");
    delay(d);
    rd = radio.receiveDone();
  }
  if(!rd) {
    debug_print(MSG_NOK);
    return false;
  }
  debug_print(MSG_OK);
  return true;
}

void intTest()
{
  if(flashConfig.features & FEAT_HAL1 && digitalRead(HAL1_PIN) == LOW) hal1_interrupt = true;
  if(flashConfig.features & FEAT_HAL2 && digitalRead(HAL2_PIN) == LOW) hal2_interrupt = true;
  if(flashConfig.features & FEAT_DRY && digitalRead(DRY_PIN) == LOW)  dry_interrupt = true;
}

bool sendData(const void *data, size_t dataSize, bool sleep) {
  digitalWrite(LED, HIGH);
  debug_printa(MSG_SEND_MSG_D, ((uint8_t*)data)[0]);
  bool success = radio.sendWithRetry(RF_GTW_NODE_ID, data, dataSize, 3, 40);
  if(success) {
    debug_print(MSG_OK);
  } else {
    debug_print(MSG_NOK);
    nodeInfoMsg.sendErrorCount++;
  }
  if(sleep) {
    debug_print("Radio goes to sleep\n");
    radio.sleep();
  }
  digitalWrite(LED, LOW);
  return success;
}
