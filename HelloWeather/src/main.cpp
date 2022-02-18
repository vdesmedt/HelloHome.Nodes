#include <Arduino.h>
#include <LowPower.h>
#include <SparkFunBME280.h>
#include <MCP7941x.h>
#include <HHCentral.h>

// Battery
#define VIN_TRIGGER 4
#define VIN_MEASURE A0
#define VIN_RATIO 2.0 //(10K + 10K) / 10K
#define VIN_VREF 3.3

HHLogger *logger;
HHCentral *hhCentral;

EnvironmentReport envReport;
NodeInfoReport infoReport;

BME280 bmeSensor;
MCP7941x rtc;

time_t nextAlarm;
TimeElements te;

void print(TimeElements &te)
{
    char buffer[30];
    snprintf(buffer, 30, "%02d:%02d:%02d %02d/%02d/%04d (%d)", te.Hour, te.Minute, te.Second, te.Day, te.Month, tmYearToCalendar(te.Year), te.Wday);
    Serial.println(buffer);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(3, INPUT_PULLUP);
  pinMode(VIN_TRIGGER, OUTPUT);
  digitalWrite(VIN_TRIGGER, HIGH);

  pinMode(VIN_MEASURE, INPUT);
#ifdef RELEASE
  logger = new HHLogger(LogMode::Off);
  hhCentral = new HHCentral(logger, NodeType::HelloWeather, GIT_FLAG, HHEnv::Pro);
#else
  logger = new HHLogger(LogMode::Text);
  hhCentral = new HHCentral(logger, NodeType::HelloWeather, GIT_FLAG, HHEnv::Dev);
#endif
  HHCErr err = hhCentral->connect();
  if (err != HHCNoErr)
    logger->log("HHCentral connection issue : %d\n", err);

  hhCentral->setRadioToSleepAfterSend();

  // Setup RTC to interrupt every hours
  te = { .Second = 0, .Minute = 0, .Hour = 18, .Wday = 3, .Day = 18, .Month = 1, .Year = CalendarYrToTm(2022) };
  rtc.setClock(te);
  rtc.startClock();
  nextAlarm = makeTime(te);
  breakTime(nextAlarm, te);
  rtc.setAlarm(MCP7941x::alarm::Alarm0, te, MCP7941x::alarmMask::all);
  rtc.enableAlarm(MCP7941x::alarm::Alarm0);
  rtc.setMfpPolarity(MCP7941x::mfpPolarity::reversePolarity);

  //BME Sensor initiatilization
  bmeSensor.setI2CAddress(0x76);
  if (bmeSensor.beginI2C())
  {
    logger->log("BME Enabled\n");
  }
  else
  {
    logger->log("BME Init Failed. Node stopped.\n");
    while (true)
      ;
  }
  bmeSensor.setMode(MODE_SLEEP);
}

void wakeUp(void)
{
}

void loop()
{
    rtc.getClock(te);        
    nextAlarm = makeTime(te)+20;
    breakTime(nextAlarm, te);
    rtc.setAlarm(MCP7941x::alarm::Alarm0, te, MCP7941x::alarmMask::all);
    rtc.clearInterruptFlag(MCP7941x::alarm::Alarm0);

    bmeSensor.setMode(MODE_FORCED);
    while(bmeSensor.isMeasuring() == false) ; //Wait for sensor to start measurment
    while(bmeSensor.isMeasuring() == true) ; //Hang out while sensor completes the reading    

    envReport.humidity = bmeSensor.readFloatHumidity() * 100;
    envReport.pressure = (unsigned int)(bmeSensor.readFloatPressure()/10);
    envReport.temperature  = bmeSensor.readTempC() * 100;
    hhCentral->send(&envReport);

    digitalWrite(VIN_TRIGGER, LOW);
    delay(1);
    uint16_t vInAdc = 0;
    for(uint8_t i = 0; i<3; i++) {
      vInAdc += analogRead(VIN_MEASURE);
      delayMicroseconds(100);
    }
    vInAdc /= 3;
    Serial.print("vInAdc:");Serial.println(vInAdc);
    infoReport.vIn = (vInAdc * 2 * VIN_VREF) / 10.24;
    infoReport.sendErrorCount = hhCentral->sendErrorCount();
    digitalWrite(VIN_TRIGGER, HIGH);
    Serial.print("VIn:");Serial.println(infoReport.vIn);
    hhCentral->send(&infoReport);

    attachInterrupt(digitalPinToInterrupt(3), wakeUp, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(3));
}

