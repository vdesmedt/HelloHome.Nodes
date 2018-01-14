#include <version.h>

#define DEBUG true
#define debug_print(...) \
            do { if (DEBUG) { \
              char buffer[100]; \
              snprintf(buffer, 100, (const char *)__VA_ARGS__); \
              Serial.print(buffer); \
            } \
          } while (0)

#include <Arduino.h>
#include <SI7021.h>
#include <LowPower.h>
#include <RFM69.h>    //get it here: https://github.com/LowPowerLab/RFM69
#include <SPI.h>      //get it here: https://github.com/LowPowerLab/SPIFlash
#include <SPIFlash.h>
#include <Wire.h>

#include <messages.h>

//RFM69
#define RF_ENCRYPT_KEY  "passiondesfruits"
#define RF_GTW_NODE_ID  254
#define RF_IS_RFM69HW

//SPIFlash
#define FLASH_ADR	32768 //First memory availble after reserver for OTA update

//Battery
#define VIN_TRIGGER  A6    
#define VIN_MEASURE  A0
#define VIN_RATIO    2.0 //(4.7 + 4.7) / 4.7
#define VIN_VREF     3.3

//HAL and DRY contact
#define DRY_PIN       4
#define HAL1_PIN      5
#define HAL2_PIN      6

//Led
#define LED           9

RFM69 radio;
SI7021 siSensor;

bool displayOn = false;
bool hal1_interrupt = false;
bool hal2_interrupt = false;
bool dry_interrupt = false;
int wakeUpCount = 0;

uint8_t* duid;  //Device unique Id (from flash)

PulseReport hal1Msg;
PulseReport hal2Msg;
PulseReport dryMsg;
NodeInfoReport nodeInfoMsg;
EnvironmentReport envMsg;

typedef struct NodeConfig {
  uint8_t sigFirst; //First byte of the uid.  Checked to detect first read
  char version[7];
  unsigned char nodeId;
  uint16_t features;
  unsigned int startCount;
} NodeConfig;
NodeConfig config;

bool sendData(const void *data, size_t dataSize, bool sleep = true);
void(* resetFunc) (void) = 0; //declare reset function @ address 0
void intTest();

void radioInit(int nodeId) {
  //Initialize Radio
  radio.initialize(RF69_868MHZ, nodeId, RF_NETWORK_ID);
  radio.encrypt(RF_ENCRYPT_KEY);
#ifdef RF_IS_RFM69HW
  radio.setHighPower();
#endif
}

bool WaitRf(int milliseconds) {
  int d = 100;
  int retryCount = milliseconds / d;
  debug_print("Waiting for rf replay");
  bool rd = radio.receiveDone();
  while(!rd && retryCount-- > 0) {
    debug_print(".");
    delay(d);
    rd = radio.receiveDone();
  }
  if(!rd) {
    debug_print("Not received :-(\n");
    return false;
  }
  debug_print("Received :-)\n");
  return true;
}

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    while(!Serial) delay(10);
  #endif

  //Flash initialization
  debug_print("Flash initialization...");
  SPIFlash flash(8, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
  if(!flash.initialize()) {
    debug_print("failed. Restarting...\n");
    delay(1000);
    resetFunc();
  } else {
    debug_print("OK\n");
  }
  duid = (uint8_t *)malloc(8);
  duid = flash.readUniqueId();

  //Read & Update config (start count)
  debug_print("Reading config\n");	
  flash.readBytes(FLASH_ADR, (void*)&config, sizeof(NodeConfig));
  if(duid[0] != config.sigFirst || 0 != memcmp(config.version, VERSION, 7)) {
    debug_print("No config found or new version detected. Init config.");
    //First read or new version
    config.sigFirst = duid[0];
    config.features = 0;
    memcpy(config.version, VERSION, 7);
    config.nodeId = 255;
    config.startCount = 0;
  }
  config.startCount++;
  nodeInfoMsg.startCount = config.startCount;
  flash.blockErase4K(FLASH_ADR);
  flash.writeBytes(FLASH_ADR, (const void *)&config, sizeof(NodeConfig));

  //Init radio and send startup report
  radioInit(config.nodeId);
  NodeStartedReport nodeStartedMsg;
  memcpy(nodeStartedMsg.version, VERSION, 7);
  memcpy(nodeStartedMsg.signature, duid, 8);
  sendData(&nodeStartedMsg, sizeof(NodeStartedReport), false);
  if(!WaitRf(10000)) {
    debug_print("No answer received. restarting.");
    delay(1000); resetFunc();
  }
  
  if(radio.DATALEN != sizeof(NodeConfigCommand) || radio.DATA[0] != (2 + (0 << 2))) {
    debug_print("Expected NodeConfig but received %d bytes. Restarting", radio.DATALEN);
    delay(1000); resetFunc();
  }
  
  NodeConfigCommand receivedConfig;
  memcpy(&receivedConfig, (const void *)radio.DATA, sizeof(NodeConfigCommand));
  if(radio.ACKRequested()) radio.sendACK();
  
  if(memcmp((const void *)&receivedConfig.signature, duid, 8)) {
    debug_print("Received config for another sig. restarting.");
    delay(1000); resetFunc();
  }

  //Update config if updates (nodeId or features)
  if(config.nodeId != receivedConfig.newNodeId || config.features != receivedConfig.features) {
    config.nodeId = receivedConfig.newNodeId;
    config.features = receivedConfig.features;
    debug_print("New config detected (node: %d - features : %d)\n", config.nodeId, config.features);
    flash.blockErase4K(FLASH_ADR);
    flash.writeBytes(FLASH_ADR, (const void *)&config, sizeof(NodeConfig));   
    radio.setAddress(config.nodeId);
  }

  //Configure Pin
  if(config.features & FEAT_VIN) {
    pinMode(VIN_TRIGGER, OUTPUT);
    pinMode(VIN_MEASURE, INPUT);
  }
  if(config.features & FEAT_HAL1)
    pinMode(HAL1_PIN, INPUT_PULLUP);
  if(config.features & FEAT_HAL2)
    pinMode(HAL2_PIN, INPUT_PULLUP);
  if(config.features & FEAT_HAL1)
    pinMode(DRY_PIN, INPUT_PULLUP);
  
  //Sleep radio
  radio.sleep();
  //Initialize SI7021 sensor if feature enabled
  if(config.features & FEAT_SI7021)
    siSensor.begin();
  
  //Initialize messages
  hal1Msg.newPulses = hal2Msg.newPulses = dryMsg.newPulses = 0;
  hal1Msg.subNode = 1;hal2Msg.subNode=2;dryMsg.subNode=3;
  envMsg.temperature = envMsg.humidity = envMsg.pressure = 0;
  nodeInfoMsg.sendErrorCount = nodeInfoMsg.vIn = 0;
}

void loop() {
  attachInterrupt(1, intTest, FALLING);
  if(DEBUG)
    Serial.flush();

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  detachInterrupt(1);

  wakeUpCount++;

  //Check if message for rf message
  if(radio.receiveDone()) {

  }

  //Check on pulse counters
  if(hal1_interrupt) {
    hal1Msg.newPulses++;
    if(sendData((const void*)(&hal1Msg), sizeof(hal1Msg)))
      hal1Msg.newPulses = 0;
  }
  if(hal2_interrupt) {
    hal2Msg.newPulses++;
    if(sendData((const void*)(&hal2Msg), sizeof(hal2Msg)))
      hal2Msg.newPulses = 0;
  }
  if(dry_interrupt) {
    dryMsg.newPulses++;
    if(sendData((const void*)(&dryMsg), sizeof(dryMsg)))
      dryMsg.newPulses = 0;
  }

  //Calculate Temperature and Humidity every minutes
  if(wakeUpCount % 8 == 0 && config.features & FEAT_SI7021) {
    envMsg.temperature = siSensor.getCelsiusHundredths();
    envMsg.humidity = siSensor.getHumidityPercent();
    sendData((const void*)&envMsg, sizeof(envMsg));
  }

  //Measure battery voltage every 10 minutes
  if(wakeUpCount % 75 == 0) {
    if(config.features & FEAT_VIN) {
      digitalWrite(VIN_TRIGGER, LOW);
      nodeInfoMsg.vIn = ((1.0 * analogRead(VIN_MEASURE)) / 1024.0 * VIN_VREF * VIN_RATIO) * 100;
      digitalWrite(VIN_TRIGGER, HIGH);
    }
    sendData((const void*)&nodeInfoMsg, sizeof(nodeInfoMsg));
  }

  hal1_interrupt = hal2_interrupt = dry_interrupt = false;
}

void intTest()
{
  if(config.features & FEAT_HAL1 && digitalRead(HAL1_PIN) == LOW) hal1_interrupt = true;
  if(config.features & FEAT_HAL2 && digitalRead(HAL2_PIN) == LOW) hal2_interrupt = true;
  if(config.features & FEAT_DRY && digitalRead(DRY_PIN) == LOW)  dry_interrupt = true;
}

bool sendData(const void *data, size_t dataSize, bool sleep = true) {
  digitalWrite(LED, HIGH);
  bool success = radio.sendWithRetry(RF_GTW_NODE_ID, data, dataSize, 3, 40);
  if(sleep)
    radio.sleep();
  if(!success)
    nodeInfoMsg.sendErrorCount++;
  digitalWrite(LED, LOW);
  return success;
}
