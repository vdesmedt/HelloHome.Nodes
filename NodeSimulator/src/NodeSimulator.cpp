#include <Arduino.h>
#include <RFM69.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <HHCentral.h>
#include <HhMessages.h>
#include <MemoryFree.h>
#include <OneButton.h>

#define LED 9
#define P_PUSH A0
#define P_PUSH_LED A1
#define P_LCD_DC 5
#define P_LCD_CS 6
#define P_LCD_RST 7
#define P_ENC_A 4
#define P_ENC_B 3
#define P_ENC_PUSH A2

HHLogger *hhLogger;
HHCentral *hhCentral;
U8G2_PCD8544_84X48_1_4W_HW_SPI lcd(U8G2_R0, P_LCD_CS, P_LCD_DC, P_LCD_RST);

ClickEncoder *ce;
OneButton *greenButton;

TimerOne tmr = TimerOne();

PingReport pg_msg;
NodeStartedReport ns_msg;
NodeInfoReport ni_msg;
EnvironmentReport en_msg;
PulseReport pu_msg;
PushButtonPressedReport ps_msg;
SwitchActivatedReport sw_msg;
VarioLevelChangedReport vr_msg;
Report *msg[] = {&pg_msg, &ns_msg, &ni_msg, &en_msg, &pu_msg, &ps_msg, &sw_msg, &vr_msg};

#define MODECOUNT 8
const char *modes[MODECOUNT] = {"PG", "NS", "NI", "EN", "PU", "PS", "SW", "VR"};
int8_t currentMode = 0;
int8_t focus_index = 0;
const char *pressStyles[]{"Clk", "Dbl", "Lng"};

void isr()
{
    ce->service();
}

unsigned long lastSendMilli = 0;
void sendReport()
{

    if (currentMode >= 0)
    {
        switch (currentMode)
        {
        case 0:
            pg_msg.millis = millis();
            break;
        }
        hhCentral->send(msg[currentMode]);
        lastSendMilli = millis();
    }
}

void setup()
{
    Serial.begin(9600);

    lcd.begin();
    lcd.setFont(u8g2_font_chikita_tr);

    lcd.firstPage();
    do
    {
        lcd.drawStr(0, 8, "Connecting...");
    } while (lcd.nextPage());

    greenButton = new OneButton(P_PUSH, true, false);
    greenButton->attachClick(sendReport);

    ce = new ClickEncoder(P_ENC_A, P_ENC_B, P_ENC_PUSH);
    ce->setDoubleClickEnabled(false);
    tmr.initialize(1000);
    tmr.attachInterrupt(isr);

    pinMode(P_PUSH_LED, OUTPUT);
    pinMode(P_PUSH, INPUT_PULLUP);

    pinMode(LED, OUTPUT);
    hhLogger = new HHLogger(LogMode::Text);
    hhCentral = new HHCentral(hhLogger, NodeType::Simulator, "1234567", HHEnv::Dev);
    hhCentral->connect(true);

    strcpy(ns_msg.version, "ABC");
    ni_msg.vIn = 370;
    en_msg.temperature = 2100;
    en_msg.humidity = 6000;
    en_msg.pressure = 10240;
    ps_msg.portNumber = 4;
    ps_msg.pressStyle = PressStyle::SinglePress;
}

#define LOGSIZE 3
#define LOGLINELEN 20
char screenLogLine[LOGLINELEN];
char screenLog[LOGSIZE][LOGLINELEN];
int screenLogIndex = 0;
void addToScreenLog(const char *log)
{
    strncpy(screenLog[screenLogIndex], log, 20);
    screenLogIndex = (screenLogIndex + 1) % LOGSIZE;
}

static char nconv_buffer[10];

char *dtoa(unsigned int v)
{
    snprintf(nconv_buffer, sizeof(nconv_buffer), "%u", v);
    return nconv_buffer;
}

char *dtoa(int v)
{
    snprintf(nconv_buffer, sizeof(nconv_buffer), "%d", v);
    return nconv_buffer;
}

char *dtoa(unsigned long v)
{
    snprintf(nconv_buffer, sizeof(nconv_buffer), "%lu", v);
    return nconv_buffer;
}

char *dtoa(long v)
{
    snprintf(nconv_buffer, sizeof(nconv_buffer), "%ld", v);
    return nconv_buffer;
}

char *dtoa(long v, uint8_t d)
{
    long p = 1;
    for (int i = 0; i < d; i++)
        p *= 10;
    int i = sprintf(nconv_buffer, "%ld.%ld", v / p, v % p) - 1;
    while (nconv_buffer[i] == '0')
        nconv_buffer[i--] = 0;
    if (nconv_buffer[i] == '.')
        nconv_buffer[i] = 0;
    return nconv_buffer;
}

char *btoa(bool b)
{
    snprintf(nconv_buffer, sizeof(nconv_buffer), "%s", b ? "On" : "Off");
    return nconv_buffer;
}

#define C1 0
#define C2 20
#define C3 42
#define C4 62
#define RH 8
#define BH RH - 1
#define BW C3 - C2 - 1
#define B1x C2 - 1
#define B2x C4 - 1
#define B1y 0 * RH + 2
#define B2y 1 * RH + 2
#define B3y 2 * RH + 2

void refresh_lcd()
{
    static unsigned long lastPrint = 0;
    if (lastPrint + 100 > millis())
        return;
    lcd.setFontMode(1);
    lcd.setDrawColor(2);
    lcd.firstPage();
    do
    {
        lcd.drawStr(C1, 1 * RH, "Msg:");
        if (focus_index == 0)
            lcd.drawBox(B1x, B1y, BW, BH);
        lcd.drawStr(C2, 1 * RH, modes[currentMode]);
        switch (currentMode)
        {
        case 1: //Node started
            lcd.drawStr(C1, 2 * RH, "NTy:");
            if (focus_index == 1)
                lcd.drawBox(B1x, B2y, BW, BH);
            lcd.drawStr(C2, 2 * RH, dtoa(ns_msg.nodeType));
            lcd.drawStr(C3, 2 * RH, "Ver:");
            if (focus_index == 2)
                lcd.drawBox(B2x, B2y, BW, BH);
            lcd.drawStr(C4, 2 * RH, ns_msg.version);
            break;
        case 2: //Node Info
            lcd.drawStr(C1, 2 * RH, "SEr:");
            if (focus_index == 1)
                lcd.drawBox(B1x, B2y, BW, BH);
            lcd.drawStr(C2, 2 * RH, dtoa(ni_msg.sendErrorCount));
            lcd.drawStr(C3, 2 * RH, "VIn:");
            if (focus_index == 2)
                lcd.drawBox(B2x, B2y, BW, BH);
            lcd.drawStr(C4, 2 * RH, dtoa(ni_msg.vIn, 2));
            break;
        case 3: //Environmlent
            lcd.drawStr(C1, 2 * RH, "Tmp:");
            if (focus_index == 1)
                lcd.drawBox(B1x, B2y, BW, BH);
            lcd.drawStr(C2, 2 * RH, dtoa(en_msg.temperature, 2));
            lcd.drawStr(C3, 2 * RH, "Hum:");
            if (focus_index == 2)
                lcd.drawBox(B2x, B2y, BW, BH);
            lcd.drawStr(C4, 2 * RH, dtoa(en_msg.humidity, 2));
            lcd.drawStr(C1, 3 * RH, "Atm:");
            if (focus_index == 3)
                lcd.drawBox(B1x, B3y, BW, BH);
            lcd.drawStr(C2, 3 * RH, dtoa(en_msg.pressure, 1));
            break;
        case 4: //Pulse
            lcd.drawStr(C1, 2 * RH, "PNm:");
            if (focus_index == 1)
                lcd.drawBox(B1x, B2y, BW, BH);
            lcd.drawStr(C2, 2 * RH, dtoa(pu_msg.portNumber));
            lcd.drawStr(C1, 3 * RH, "Pul:");
            if (focus_index == 2)
                lcd.drawBox(B1x, B3y, BW, BH);
            lcd.drawStr(C2, 3 * RH, dtoa(pu_msg.newPulses));
            lcd.drawStr(C3, 3 * RH, "Ofs:");
            if (focus_index == 3)
                lcd.drawBox(B2x, B3y, BW, BH);
            lcd.drawStr(C4, 3 * RH, btoa(pu_msg.isOffset));
            break;
        case 5: //Push Button
            lcd.drawStr(C1, 2 * RH, "PId:");
            if (focus_index == 1)
                lcd.drawBox(B1x, B2y, BW, BH);
            lcd.drawStr(C2, 2 * RH, dtoa(ps_msg.portNumber));
            lcd.drawStr(C3, 2 * RH, "Sty:");
            if (focus_index == 2)
                lcd.drawBox(B2x, B2y, BW, BH);
            lcd.drawStr(C4, 2 * RH, pressStyles[ps_msg.pressStyle - 1]);
            break;
        }

        int logIndex = 0;
        for (logIndex = 1; logIndex <= LOGSIZE; logIndex++)
        {
            int i = (screenLogIndex - logIndex + 2 * LOGSIZE) % LOGSIZE;
            lcd.drawStr(0, (LOGSIZE - logIndex + 3) * 8 + 4, screenLog[i]);
        }
    } while (lcd.nextPage());
    lastPrint = millis();
}

int max_focus_index[] = {1, 3, 3, 4, 4, 3, 3, 3};
void loop()
{
    if (ce->getButton() == ClickEncoder::Button::Clicked)
        focus_index = (focus_index + 1) % max_focus_index[currentMode];

    int16_t cev = ce->getValue();
    if (cev != 0)
    {
        if (focus_index == 0)
        {
            currentMode += cev;
            currentMode = currentMode > MODECOUNT ? MODECOUNT : currentMode < 0 ? 0 : currentMode;
        }
        else
        {
            switch (currentMode)
            {
            case 1: //Node startup
                switch (focus_index)
                {
                case 1:
                    ns_msg.nodeType += cev;
                    break;
                case 2:
                    for (int i = 0; i < 3; i++)
                        ns_msg.version[i] += cev;
                    break;
                }
                break;
            case 2: //Node Info
                switch (focus_index)
                {
                case 1:
                    ni_msg.sendErrorCount += cev;
                    break;
                case 2:
                    ni_msg.vIn += cev;
                    break;
                }
                break;
            case 3: //Environment
                switch (focus_index)
                {
                case 1:
                    en_msg.temperature += cev * 10;
                    break;
                case 2:
                    en_msg.humidity += cev * 10;
                    break;
                case 3:
                    en_msg.pressure += cev * 10;
                    break;
                }
            case 4: //Pulse
                switch (focus_index)
                {
                case 1:
                    pu_msg.portNumber += cev;
                    break;
                case 2:
                    pu_msg.newPulses += cev;
                    break;
                case 3:
                    pu_msg.isOffset = !pu_msg.isOffset;
                    break;
                }
            case 5: //Push button
                switch (focus_index)
                {
                case 1:
                    ps_msg.portNumber += cev;
                    break;
                case 2:
                    int cv = ps_msg.pressStyle + cev;
                    cv = cv > 3 ? 3 : cv < 1 ? 1 : cv;
                    ps_msg.pressStyle = (enum PressStyle)cv;
                    break;
                }
            }
        }
    }
    refresh_lcd();
    greenButton->tick();

    Command *cmd = hhCentral->check();
    if (cmd != nullptr)
    {
        hhLogger->log(HHL_RPT_RECEIVED, cmd->msgType);
        switch (cmd->msgType)
        {
        case CMD_SETRELAY:
        {
            SetRelayStateCommand *sr_cmd = (SetRelayStateCommand *)cmd;
            snprintf(screenLogLine, LOGLINELEN, "%lu-Relay %d->%s", millis() - lastSendMilli, sr_cmd->portNumber, sr_cmd->newState == 0 ? "Off" : "On");
            addToScreenLog(screenLogLine);
        }
        break;
        case CMD_PONG:
            snprintf(screenLogLine, LOGLINELEN, "%lu-Pong %i/%i", millis() - lastSendMilli, ((PongCommand *)cmd)->PingRssi, hhCentral->LastRssi());
            addToScreenLog(screenLogLine);
            break;
        }
    }
    static unsigned long fmr = millis();
    if (fmr + 5000 < millis())
    {
        Serial.print("Free Mem:");
        Serial.println(freeMemory());
        fmr = millis();
    }
}