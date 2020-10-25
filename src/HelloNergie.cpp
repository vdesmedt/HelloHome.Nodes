#include <Arduino.h>
#include <LowPower.h>
#include <RFM69_OTA.h>
#include "SparkFunBME280.h"
#include <EmonLib.h>

#include <version.h>
#include <HhMessages.h>
#include <HelloNergie.h>
#include <HHCentral.h>

//RFM69
#define RF_ENCRYPT_KEY "passiondesfruits"
#define RF_GTW_NODE_ID 1
#define RF_IS_RFM69HW_

//SPIFlash
#define FLASH_ADR 0x20000 //First memory availble after reserved for OTA update

//Battery
#define VIN_TRIGGER 15
#define VIN_MEASURE A0
#define VIN_RATIO 2.0 //(4.7 + 4.7) / 4.7
#define VIN_VREF 3.3

//HAL and DRY contact
#define PULSE_PIN1 4
#define PULSE_PIN2 5
#define PULSE_PIN3 6
#define MIN_PULSE_INTERVAL 200

//ELEC POWER
bool CT_1_Enabled = false;
#define CT_1 A6
bool CT_2_Enabled = false;
#define CT_2 A7

#define AUTO_DETECT_SIG 7

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
RFM69 radio;
EnergyMonitor emon1;
EnergyMonitor emon2;
BME280 bmeSensor;

uint8_t newPulse[3] = {0, 0, 0};
unsigned long lastPulse[3] = {0, 0, 0};
bool newPulses = false;

bool displayOn = false;

PulseReport pulseMsg;
NodeInfoReport nodeInfoMsg;
EnvironmentReport envMsg;

char floatv[10];

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial)
    delay(10);
#endif

  //Flash initialization
  debug_print(MSG_FLASH_INIT);
  if (!flash.initialize())
  {
    debug_print(MSG_NOK_RESTART);
    delay(1000);
    resetFunc();
  }
  else
  {
    debug_print(MSG_OK);
  }
  //Retreive unique Id from flash
  flash.readUniqueId();

  //Read & Update config (start count)
  debug_print(MSG_READ_CONFIG);
  flash.readBytes(FLASH_ADR, &flashConfig, sizeof(NodeFlashConfig));
  flashConfig.startCount++;
  if (CONFIG_VERSION != flashConfig.confVer)
  {
    debug_print(MSG_NOCONFIG_OR_NEWVERSION);
    //First read or new version
    flashConfig.confVer = CONFIG_VERSION;
    flashConfig.features = 0;
    flashConfig.nodeId = 253;
    flashConfig.startCount = 1;
    flashConfig.environmentFreq = 0;
    flashConfig.nodeInfoFreq = 0;
  }
  else
  {
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
  if (!WaitRf(10000))
  {
    debug_print(MSG_NOK_RESTART);
    delay(1000);
    resetFunc();
  }

  //Check data if compatible with config
  if (radio.DATALEN != sizeof(NodeConfigCommand) || radio.DATA[0] != (2 + (0 << 2)))
  {
    debug_printa(MSG_NODECONFIG_EXPECTED_D, sizeof(NodeConfigCommand), radio.DATALEN);
    if (radio.ACKRequested())
      radio.sendACK();
    delay(1000);
    resetFunc();
  }

  //Put response in config struct & ack
  NodeConfigCommand receivedConfig;
  memcpy(&receivedConfig, (const void *)radio.DATA, sizeof(NodeConfigCommand));
  if (radio.ACKRequested())
    radio.sendACK();

  //Check config is for right node by testing signature
  if (memcmp((const void *)&receivedConfig.signature, flash.UNIQUEID, 8))
  {
    debug_print(MSG_SIGNATURE_MISSMATCH);
    debug_print(MSG_NOK_RESTART);
    delay(1000);
    resetFunc();
  }

  //Update flash config if needed (nodeId or features)
  if (flashConfig.nodeId != receivedConfig.newNodeId || flashConfig.features != receivedConfig.features)
  {
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

  //Configure Pin
  pinMode(3, INPUT_PULLUP); //Interrupt

  if (receivedConfig.features & FEAT_BATT)
  {
    debug_print(MSG_BATT_ENABLED);
    pinMode(VIN_TRIGGER, OUTPUT);
    pinMode(VIN_MEASURE, INPUT);
  }
  if (receivedConfig.features & FEAT_PULSE1)
  {
    debug_printa(MSG_PULSE_ENABLED, 1);
    pinMode(PULSE_PIN1, INPUT_PULLUP);
  }
  if (receivedConfig.features & FEAT_PULSE2)
  {
    debug_printa(MSG_PULSE_ENABLED, 2);
    pinMode(PULSE_PIN2, INPUT_PULLUP);
  }
  if (receivedConfig.features & FEAT_PULSE3)
  {
    debug_printa(MSG_PULSE_ENABLED, 3);
    pinMode(PULSE_PIN3, INPUT_PULLUP);
  }

  //Configure energieMonitor
  pinMode(CT_1, INPUT);
  if (analogRead(CT_1) > 0)
  {
    CT_1_Enabled = true;
    emon1.current(CT_1, 83.37);
    debug_print("CT1 Enabled\n");
  }

  pinMode(CT_2, INPUT);
  if (analogRead(CT_2) > 0)
  {
    CT_2_Enabled = true;
    emon2.current(CT_2, 83.37);
    debug_print("CT2 Enabled\n");
  }

  //Initialize SI7021 sensor if feature enabled
  if (receivedConfig.features & FEAT_ENV)
  {
    bmeSensor.setI2CAddress(0x76);
    bmeSensor.beginI2C();
    debug_print(MSG_ENV_ENABLED);
  }

  //Initialize messages
  pulseMsg.newPulses1 = pulseMsg.newPulses2 = pulseMsg.newPulses3 = 0;
  envMsg.temperature = envMsg.humidity = envMsg.pressure = 0;
  nodeInfoMsg.sendErrorCount = nodeInfoMsg.vIn = 0;

  attachInterrupt(digitalPinToInterrupt(3), pulse_ISR, FALLING);
}

void loop()
{
  //Check if message for rf message
  if (radio.receiveDone())
  {
    switch (radio.DATA[0])
    {
    case 6:
      debug_print(MSG_RESTART_COMMAND_RECEIVED);
      if (radio.ACKRequested())
        radio.sendACK();
      delay(100);
      resetFunc();
      break;
    default:
      debug_printa(MSG_UNKNOWN_MSG_TYPE_D, radio.DATA[0]);
      break;
    }
    CheckForWirelessHEX(radio, flash, true);
    if (radio.ACKRequested())
      radio.sendACK();
  }

  //Check on pulse counters
  detachInterrupt(digitalPinToInterrupt(3));
  if (newPulses)
  {
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
  static unsigned long lastEnv = 0;
  if (flashConfig.features & FEAT_ENV && flashConfig.environmentFreq > 0 && lastEnv + 1000 * flashConfig.environmentFreq < millis())
  {
    lastEnv = millis();
    envMsg.temperature = bmeSensor.readTempC() * 100.0;
    envMsg.humidity = bmeSensor.readFloatHumidity() * 100.0;
    envMsg.pressure = bmeSensor.readFloatPressure() / 10;
    envMsg.dirty = true;
    debug_printa(" BME: %d C / %d %% / %d Hg\n", envMsg.temperature, envMsg.humidity, envMsg.pressure);
  }

  //Measure battery voltage
  static unsigned long lastNodeInf = 0;
  if (flashConfig.features & FEAT_BATT && flashConfig.nodeInfoFreq > 0 && lastNodeInf + 1000 * flashConfig.nodeInfoFreq < millis())
  {
    lastNodeInf = millis();
    digitalWrite(VIN_TRIGGER, LOW);
    nodeInfoMsg.vIn = ((1.0 * analogRead(VIN_MEASURE)) / 1024.0 * VIN_VREF * VIN_RATIO) * 100;
    digitalWrite(VIN_TRIGGER, HIGH);
    debug_printa(" VIN: %d\n", nodeInfoMsg.vIn);
    nodeInfoMsg.dirty = true;
  }

  //Current Sensor
  if (CT_1_Enabled)
  {
    double Irms = emon1.calcIrms(1480); // Calculate Irms only
    debug_printa(" %s A -> %d W\n", dtostrf(Irms, 4, 2, floatv), (int)(Irms * 230));
  }

  //Send dirty messages
  if (pulseMsg.dirty)
  {
    if (sendData(&pulseMsg, sizeof(PulseReport) - 1))
    {
      pulseMsg.newPulses1 = pulseMsg.newPulses2 = pulseMsg.newPulses3 = 0;
      pulseMsg.dirty = false;
    }
  }
  if (envMsg.dirty)
  {
    if (sendData(&envMsg, sizeof(EnvironmentReport) - 1))
    {
      envMsg.dirty = false;
    }
  }
  if (nodeInfoMsg.dirty)
  {
    if (sendData(&nodeInfoMsg, sizeof(NodeInfoReport) - 1))
    {
      nodeInfoMsg.dirty = false;
    }
  }
}

void radioInit(uint16_t nodeId)
{
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

bool WaitRf(int milliseconds)
{
  int d = 100;
  int retryCount = milliseconds / d;
  debug_print(MSG_WAIT_RF);
  bool rd = radio.receiveDone();
  while (!rd && retryCount-- > 0)
  {
    debug_print(".");
    delay(d);
    rd = radio.receiveDone();
  }
  if (rd)
  {
    debug_print(MSG_OK);
    return true;
  }
  debug_print(MSG_NOK);
  return false;
}

void pulse_ISR()
{
  if (flashConfig.features & FEAT_PULSE1 && digitalRead(PULSE_PIN1) == LOW)
  {
    newPulse[0]++;
    newPulses = true;
  }
  if (flashConfig.features & FEAT_PULSE2 && digitalRead(PULSE_PIN2) == LOW)
  {
    newPulse[1]++;
    newPulses = true;
  }
  if (flashConfig.features & FEAT_PULSE3 && digitalRead(PULSE_PIN3) == LOW)
  {
    newPulse[2]++;
    newPulses = true;
  }
}

bool sendData(const void *data, size_t dataSize, bool sleep)
{
  digitalWrite(LED, HIGH);
  debug_printa(MSG_SEND_MSG_D, ((uint8_t *)data)[0]);
  bool success = radio.sendWithRetry(RF_GTW_NODE_ID, data, dataSize, 3, 40);
  if (success)
  {
    debug_print(MSG_OK);
  }
  else
  {
    debug_print(MSG_NOK);
    nodeInfoMsg.sendErrorCount++;
  }
  if (sleep)
  {
    debug_print("Radio goes to sleep\n");
    radio.sleep();
  }
  digitalWrite(LED, LOW);
  return success;
}
