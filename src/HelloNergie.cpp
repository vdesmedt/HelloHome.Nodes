#include <Arduino.h>
#include <SI7021.h>
#include <LowPower.h>
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


//RFM69
#define RF_ENCRYPT_KEY  "passiondesfruits"
#define RF_GTW_NODE_ID  1
#define RF_IS_RFM69HW

//SPIFlash
#define FLASH_ADR	0x20000 //First memory availble after reserved for OTA update

//Battery
#define VIN_TRIGGER  A6    
#define VIN_MEASURE  A0
#define VIN_RATIO    2.0 //(4.7 + 4.7) / 4.7
#define VIN_VREF     3.3

//HAL and DRY contact
#define PULSE_PIN1      4
#define PULSE_PIN2      5
#define PULSE_PIN3      6
#define MIN_PULSE_INTERVAL 200

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

uint8_t newPulse[3] = {0, 0, 0};
unsigned long lastPulse[3] = {0, 0, 0};
bool newPulses = false;

bool displayOn = false;
int wakeUpCount = 0;

PulseReport pulseMsg;
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
void pulse_ISR();
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
    debug_printa(MSG_NODECONFIG_EXPECTED_D, sizeof(NodeConfigCommand), radio.DATALEN);
    if(radio.ACKRequested()) radio.sendACK();
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
  if(receivedConfig.features & FEAT_BATT) {
    debug_print(MSG_BATT_ENABLED);
    pinMode(VIN_TRIGGER, OUTPUT);
    pinMode(VIN_MEASURE, INPUT);
  }
  if(receivedConfig.features & FEAT_PULSE1) {
    debug_printa(MSG_PULSE_ENABLED, 1);
    pinMode(PULSE_PIN1, INPUT_PULLUP);
  }
  if(receivedConfig.features & FEAT_PULSE2) {
    debug_printa(MSG_PULSE_ENABLED, 2);
    pinMode(PULSE_PIN2, INPUT_PULLUP);
  }
  if(receivedConfig.features & FEAT_PULSE3) {
    debug_printa(MSG_PULSE_ENABLED, 3);
    pinMode(PULSE_PIN3, INPUT_PULLUP);
  }
  
  //Initialize SI7021 sensor if feature enabled
  if(receivedConfig.features & FEAT_ENV) {
    debug_print(MSG_ENV_ENABLED);
    siSensor.begin();
  }
  
  //Initialize messages
  pulseMsg.newPulses1 = pulseMsg.newPulses2 = pulseMsg.newPulses3 = 0;
  envMsg.temperature = envMsg.humidity = envMsg.pressure = 0;
  nodeInfoMsg.sendErrorCount = nodeInfoMsg.vIn = 0;
}

void loop() {
  attachInterrupt(digitalPinToInterrupt(3), pulse_ISR, FALLING);
  if(DEBUG) Serial.flush();  //Flush serial before going to sleep
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
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
  detachInterrupt(digitalPinToInterrupt(3));
  debug_printa("WC:%d\n", wakeUpCount);
  if(newPulses) {
    pulseMsg.newPulses1 += newPulse[0];
    pulseMsg.newPulses2 += newPulse[1];
    pulseMsg.newPulses3 += newPulse[2];
    newPulse[0] = newPulse[1] = newPulse[2] = 0;
    newPulses = false;
    pulseMsg.dirty = true;
    debug_printa("New pulses %d %d %d\n", pulseMsg.newPulses1, pulseMsg.newPulses2, pulseMsg.newPulses3);
  }
  attachInterrupt(digitalPinToInterrupt(3), pulse_ISR, FALLING);

  //Calculate Temperature and Humidity every environmentFreq wakeup
  if(flashConfig.features & FEAT_ENV && flashConfig.environmentFreq > 0 &&  wakeUpCount % flashConfig.environmentFreq == 0) {
    envMsg.temperature = siSensor.getCelsiusHundredths();
    envMsg.humidity = siSensor.getHumidityPercent();
    envMsg.dirty = true;
    debug_printa(" SI7021: %d C / %d\n", envMsg.temperature, envMsg.humidity);
  }

  //Measure battery voltage
  if(flashConfig.features & FEAT_BATT && flashConfig.nodeInfoFreq > 0 && wakeUpCount % flashConfig.nodeInfoFreq == 0) { 
    digitalWrite(VIN_TRIGGER, LOW);
    nodeInfoMsg.vIn = ((1.0 * analogRead(VIN_MEASURE)) / 1024.0 * VIN_VREF * VIN_RATIO) * 100;
    digitalWrite(VIN_TRIGGER, HIGH);
    debug_printa(" VIN: %d\n", nodeInfoMsg.vIn);
    nodeInfoMsg.dirty = true;
  }

  //Send dirty messages
  if(pulseMsg.dirty) {
    if(sendData(&pulseMsg, sizeof(PulseReport)-1)) {
      pulseMsg.newPulses1 = pulseMsg.newPulses2 = pulseMsg.newPulses3 = 0;
      pulseMsg.dirty = false;
    }
  }
  if(envMsg.dirty) {
    if(sendData(&envMsg, sizeof(EnvironmentReport)-1)) {
      envMsg.dirty = false;
    }
  }
  if(nodeInfoMsg.dirty) {
    if(sendData(&nodeInfoMsg, sizeof(NodeInfoReport)-1)) {
      nodeInfoMsg.dirty = false;
    }
  }
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
  if(rd) {
    debug_print(MSG_OK);
    return true;
  }
  debug_print(MSG_NOK);
  return false;
}  

void pulse_ISR()
{
  if(flashConfig.features & FEAT_PULSE1 && digitalRead(PULSE_PIN1) == LOW ) {
    newPulse[0]++;
    newPulses = true;
  } 
  if(flashConfig.features & FEAT_PULSE2 && digitalRead(PULSE_PIN2) == LOW ) {
    newPulse[1]++;
    newPulses = true;
  } 
  if(flashConfig.features & FEAT_PULSE3 && digitalRead(PULSE_PIN3) == LOW ) {
    newPulse[2]++;
    newPulses = true;
  } 
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
