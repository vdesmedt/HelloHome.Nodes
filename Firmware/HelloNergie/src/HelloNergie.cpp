#include <Arduino.h>
#include <LowPower.h>
#include <RFM69_OTA.h>
#include "SparkFunBME280.h"

#include <HHCentral.h>
#include <HHMessages.h>

// Features
#define FEAT_ENV 1
#define FEAT_BATT 2

// Battery
#define VIN_TRIGGER 15
#define VIN_MEASURE A0
#define VIN_RATIO 2.0 //(4.7 + 4.7) / 4.7
#define VIN_VREF 3.3

#define MIN_PULSE_INTERVAL 200

volatile uint8_t newPulse[] = {0, 0, 0, 0};
volatile bool newPulses = false;

HHLogger *logger;
HHCentral *hhCentral;

PulseReport pulseReport1, pulseReport2, pulseReport3, pulseReport4;
PulseReport* pulseReports[] = { &pulseReport1, &pulseReport2, &pulseReport3, &pulseReport4 };
EnvironmentReport envReport;
NodeInfoReport nodeReport;
#ifdef RELEASE
#define NODE_INFO_PERIOD  1000UL * 60UL * 60UL
#define NODE_PULSE_PERIOD 1000UL * 60UL 
#define NODE_ENV_PERIOD   1000UL * 60UL * 10UL
#else
#define NODE_INFO_PERIOD  1000UL * 07UL
#define NODE_PULSE_PERIOD 1000UL * 03UL 
#define NODE_ENV_PERIOD   1000UL * 05UL 
#endif

BME280 bmeSensor;

uint16_t features;

void pulse_ISR();

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  logger = new HHLogger();
#ifdef RELEASE
  hhCentral = new HHCentral(logger, NodeType::HelloNergie, GIT_FLAG, HHEnv::Pro);
#else
  hhCentral = new HHCentral(logger, NodeType::HelloNergie, GIT_FLAG, HHEnv::Dev);
#endif
  HHCErr err = hhCentral->connect();
  if (err != HHCNoErr)
    logger->log("HHCentral connection issue : %d\n", err);

  // Configure Pin
  pinMode(3, INPUT_PULLUP); // Interrupt
  features = hhCentral->Features();

  if (features & FEAT_BATT)
  {
    logger->log("Battmon enabled \n");
    pinMode(VIN_TRIGGER, OUTPUT);
    pinMode(VIN_MEASURE, INPUT);
  }
  for (int i = 0; i < 4; i++)
    pinMode(4+i, INPUT_PULLUP);

  // Initialize BE280 sensor if feature enabled
  if (features & FEAT_ENV)
  {
    bmeSensor.setI2CAddress(0x76);
    if(bmeSensor.beginI2C()) 
    {
      logger->log("BME Enabled\n");
    }
    else 
    {
      features = features & ~FEAT_ENV;
      logger->log("BME Init Failed. Feature disabled.\n");
    }    
  }

  // Initialize messages
  for(int i=0 ; i<4; i++) {
    pulseReports[i]->newPulses = 0;    
    pulseReports[i]->isOffset = false;
    pulseReports[i]->portNumber = 10 + i;
  }

  envReport.temperature = envReport.humidity = envReport.pressure = 0;
  nodeReport.sendErrorCount = nodeReport.vIn = 0;

  attachInterrupt(digitalPinToInterrupt(3), pulse_ISR, FALLING);
}

void loop()
{
  hhCentral->check();
  // Check on pulse counters
  if (newPulses)
  {
    detachInterrupt(digitalPinToInterrupt(3));
    for(int i=0 ; i<4; i++) {
      pulseReports[i]->newPulses += newPulse[i];
      newPulse[i] = 0;
    }
    newPulses = false;
    attachInterrupt(digitalPinToInterrupt(3), pulse_ISR, FALLING);
    logger->log("New pulses %d %d %d %d\n", pulseReport1.newPulses, pulseReport2.newPulses, pulseReport3.newPulses, pulseReport4.newPulses);
  }

  // Measure battery voltage
  static unsigned long lastNodeInfoSent = 0;
  if (millis() - lastNodeInfoSent > NODE_INFO_PERIOD)
  {
    lastNodeInfoSent = millis();
    if (features & FEAT_BATT)
    {
      digitalWrite(VIN_TRIGGER, LOW);      
      nodeReport.vIn = ((double)analogRead(VIN_MEASURE) / 1023.0) * VIN_VREF * VIN_RATIO * 100.0;
      digitalWrite(VIN_TRIGGER, HIGH);
    }
    else
    {
      nodeReport.vIn = 0;
    }
    nodeReport.sendErrorCount = hhCentral->sendErrorCount();
    hhCentral->sendReport(&nodeReport, sizeof(NodeInfoReport));
  }

  //Send new pulses
  static unsigned long lastPulseSent = 0;
  if (millis() - lastPulseSent > NODE_PULSE_PERIOD)
  {
    for (int i = 0; i < 4; i++)
    {
      if (pulseReports[i]->newPulses > 0 && HHCNoErr == hhCentral->sendReport(pulseReports[i], sizeof(PulseReport)))
      {
        pulseReports[i]->newPulses = 0;
        lastPulseSent = millis();
      }
    }
  }

  //Send environment values
  static unsigned long lastEnvSent = 0;
  if(features & FEAT_ENV &&  millis() - lastEnvSent > NODE_ENV_PERIOD)
  {
    BME280_SensorMeasurements *measures = new BME280_SensorMeasurements();
    bmeSensor.readAllMeasurements(measures, 0);
    envReport.humidity = measures->humidity * 100;
    envReport.pressure = (unsigned int)(measures->pressure/10);
    envReport.temperature  = measures->temperature * 100;
    hhCentral->sendReport(&envReport, sizeof(NodeInfoReport));
    lastEnvSent = millis();  //Does not retry every loop if failing...
  }
}

void pulse_ISR()
{
  static unsigned long lastPulseMillis[]{0, 0, 0, 0};
  unsigned long now = millis();

  for (int i = 0; i < 4; i++)
    if (digitalRead(4+i) == LOW && now - lastPulseMillis[i] > MIN_PULSE_INTERVAL)
    {
      newPulse[i]++;
      newPulses = true;
      lastPulseMillis[i] = now;
    }
}