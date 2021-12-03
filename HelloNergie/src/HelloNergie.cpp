#include <Arduino.h>
#include <LowPower.h>
#include <RFM69_OTA.h>
#include "SparkFunBME280.h"

#include <version.h>
#include <HHCentral.h>
#include <HHMessages.h>

//Features
#define FEAT_ENV 1
#define FEAT_BATT 2
#define FEAT_HAL1 4
#define FEAT_HAL2 8
#define FEAT_DRY1 16


//Battery
#define VIN_TRIGGER 15
#define VIN_MEASURE A0
#define VIN_RATIO 2.0 //(4.7 + 4.7) / 4.7
#define VIN_VREF 3.3

//HAL and DRY contact (Normally High, Fall on detect)
#define HAL1_PIN 4
#define HAL2_PIN 5
#define DRY1_PIN 6
#define MIN_PULSE_INTERVAL 200

volatile uint8_t newPulse[] = {0, 0, 0};
volatile bool newPulses = false;

HHLogger *logger;
HHCentral *hhCentral;

PulseReport pulseReportHal1, pulseReportHal2, pulseReportDry1;
EnvironmentReport envReport;
NodeInfoReport nodeReport;
#ifdef RELEASE
#define NODE_INFO_PERIOD 60UL * 60UL * 1000UL
#else
#define NODE_INFO_PERIOD 5UL * 1000UL
#endif

BME280 bmeSensor;

uint16_t features;

void pulse_ISR();

void setup()
{
  Serial.begin(115200);
  #ifdef RELEASE
    logger = new HHLogger(LogMode::Off);
    hhCentral = new HHCentral(logger, NodeType::HelloNergie, "1.0", HHEnv::Pro, true);
  #else
    logger = new HHLogger(LogMode::Text);
    hhCentral = new HHCentral(logger, NodeType::HelloNergie, "1.0", HHEnv::Dev, true);
  #endif
  HHCErr err = hhCentral->connect();
  if(err != HHCNoErr)
    logger->log("HHCentral connection issue : %d\n", err);


  //Configure Pin
  pinMode(3, INPUT_PULLUP); //Interrupt
  features = hhCentral->Features();

  if (features & FEAT_BATT)
  {
    logger->log("Battmon enabled \n");
    pinMode(VIN_TRIGGER, OUTPUT);
    pinMode(VIN_MEASURE, INPUT);
  }
  if (features & FEAT_HAL1)
  {
    logger->log("HAL1 Enabled\n");
    pinMode(HAL1_PIN, INPUT_PULLUP);
  }
  if (features & FEAT_HAL2)
  {
    logger->log("HAL2 Enabled\n");
    pinMode(HAL2_PIN, INPUT_PULLUP);
  }
  if (features & FEAT_DRY1)
  {
    logger->log("DRY1 Enabled");
    pinMode(DRY1_PIN, INPUT_PULLUP);
  }

  //Initialize SI7021 sensor if feature enabled
  if (features & FEAT_ENV)
  {
    bmeSensor.setI2CAddress(0x76);
    bmeSensor.beginI2C();
    logger->log("BME Enabled");
  }

  //Initialize messages
  pulseReportHal1.newPulses = pulseReportHal2.newPulses = pulseReportDry1.newPulses = 0;
  pulseReportHal1.isOffset = pulseReportHal2.isOffset = pulseReportDry1.isOffset = false;
  pulseReportHal1.portNumber = 10;
  pulseReportHal2.portNumber = 11;
  pulseReportDry1.portNumber = 12;

  envReport.temperature = envReport.humidity = envReport.pressure = 0;
  nodeReport.sendErrorCount = nodeReport.vIn = 0;

  attachInterrupt(digitalPinToInterrupt(3), pulse_ISR, FALLING);
}

void loop()
{
  hhCentral->check();
  //Check on pulse counters
  if (newPulses)
  {
    detachInterrupt(digitalPinToInterrupt(3));
    pulseReportHal1.newPulses += newPulse[0];
    pulseReportHal2.newPulses += newPulse[1];
    pulseReportDry1.newPulses += newPulse[2];
    newPulse[0] = newPulse[1] = newPulse[2] = 0;
    newPulses = false;
    attachInterrupt(digitalPinToInterrupt(3), pulse_ISR, FALLING);
    logger->log("New pulses %d %d %d\n", pulseReportHal1.newPulses, pulseReportHal2.newPulses, pulseReportDry1.newPulses);
  }

  //Measure battery voltage
  static unsigned long lastNodeInf = 0;
  if ((millis() - lastNodeInf > NODE_INFO_PERIOD))
  {
    lastNodeInf = millis();
    if(features & FEAT_BATT)
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
    hhCentral->send(&nodeReport);
  }
  if(pulseReportHal1.newPulses > 0) {
    hhCentral->send(&pulseReportHal1);
    pulseReportHal1.newPulses = 0;
  }
  if(pulseReportHal2.newPulses > 0) {
    hhCentral->send(&pulseReportHal2);
    pulseReportHal2.newPulses = 0;
  }
  if(pulseReportDry1.newPulses > 0) {
    hhCentral->send(&pulseReportDry1);
    pulseReportDry1.newPulses = 0;
  }
}

void pulse_ISR()
{
  static unsigned long lastPulseMillis[] { 0, 0, 0 };
  unsigned long now = millis();

  if (features & FEAT_HAL1 && digitalRead(HAL1_PIN) == LOW && now - lastPulseMillis[0] > MIN_PULSE_INTERVAL)
  {
    newPulse[0]++;
    newPulses = true;
    lastPulseMillis[0] = now;
  }
  if (features & FEAT_HAL2 && digitalRead(HAL2_PIN) == LOW && now - lastPulseMillis[1] > MIN_PULSE_INTERVAL)
  {
    newPulse[1]++;
    newPulses = true;
    lastPulseMillis[1] = now;
  }
  if (features & FEAT_DRY1 && digitalRead(DRY1_PIN) == LOW && now - lastPulseMillis[2] > MIN_PULSE_INTERVAL)
  {
    newPulse[2]++;
    newPulses = true;
    lastPulseMillis[2] = now;
  }
}