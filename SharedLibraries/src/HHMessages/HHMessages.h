#ifndef hhmessages_h
#define hhmessages_h

#include <Arduino.h>

#define FEAT_ENV 1
#define FEAT_BATT 2
#define FEAT_PULSE1 4
#define FEAT_PULSE2 8
#define FEAT_PULSE3 16

struct BaseMessage
{
    uint8_t msgType;
};

struct Report : BaseMessage
{
};

struct Command : BaseMessage
{
};

#define RPT 0
#define RPT_PING RPT + (0 << 2)
#define RPT_NODESTARTED RPT + (1 << 2)
#define RPT_NODEINFO RPT + (2 << 2)
#define RPT_ENVIRONMENT RPT + (3 << 2)
#define RPT_PULSE RPT + (4 << 2)
#define RPT_BTNPRESS RPT + (5 << 2)
#define RPT_SWITCH RPT + (6 << 2)
#define RPT_VARIO RPT + (7 << 2)
#define RPT_POWER RPT + (8 << 2)
#define RPT_VALOG RPT + (9 << 2)

struct PingReport : Report
{
    PingReport()
    {
        msgType = RPT_PING;
    }
    uint32_t millis;
};

struct NodeStartedReport : Report
{
    NodeStartedReport()
    {
        msgType = RPT_NODESTARTED;
    }
    uint8_t nodeType = 1;
    uint8_t signature[8];
    char version[8];
    uint16_t startCount;
};

struct NodeInfoReport : Report
{
    NodeInfoReport()
    {
        msgType = RPT_NODEINFO;
    }
    uint16_t sendErrorCount;
    uint16_t vIn;
};

struct EnvironmentReport : Report
{
    EnvironmentReport()
    {
        msgType = RPT_ENVIRONMENT;
    }
    int temperature;
    unsigned int humidity;
    unsigned int pressure;
};

struct PulseReport : Report
{
    PulseReport()
    {
        msgType = RPT_PULSE;
    }
    uint8_t portNumber;
    int16_t newPulses;
    bool isOffset;
};

enum PressStyle
{
    SinglePress = 1,
    DoublePress = 2,
    LongPress = 3,
};

struct PushButtonPressedReport : Report
{
    PushButtonPressedReport()
    {
        msgType = RPT_BTNPRESS;
    }
    uint8_t portNumber;
    enum PressStyle pressStyle;
};

struct SwitchActivatedReport : Report
{
    SwitchActivatedReport()
    {
        msgType = RPT_SWITCH;
    }
    uint8_t portNumber;
    uint8_t newState;
};

struct VarioLevelChangedReport : Report
{
    VarioLevelChangedReport()
    {
        msgType = RPT_VARIO;
    }
    uint8_t portNumber;
    uint8_t newLevel;
};

struct PowerConsumptionReport : Report
{
    PowerConsumptionReport()
    {
        msgType = RPT_POWER;
    }
    uint16_t overLastSec;
    float Wh;
};

struct VoltAmperReport : Report
{
    VoltAmperReport()
    {
        msgType = RPT_VALOG;
    }
    int16_t Tension;
    int16_t Current;
};

#define CMD 2
#define CMD_NODECONFIG CMD + (0 << 2)
#define CMD_RESTART CMD + (1 << 2)
#define CMD_SETRELAY CMD + (2 << 2)
#define CMD_PONG CMD + (3 << 2)
#define CMD_LXI CMD + (4 << 2)

struct NodeConfigCommand : Command
{
    NodeConfigCommand()
    {
        msgType = CMD_NODECONFIG;
    }
    uint8_t signature[8];
    uint16_t newNodeId;
    uint16_t features;
    uint8_t nodeInfoFreq;
    uint8_t environmentFreq;
};

struct RestartCommand : Command
{
    RestartCommand()
    {
        msgType = CMD_RESTART;
    }
};

struct SetRelayStateCommand : Command
{
    SetRelayStateCommand()
    {
        msgType = CMD_SETRELAY;
    }
    uint8_t portNumber;
    uint8_t newState;
};

struct PongCommand : Command
{
    PongCommand()
    {
        msgType = CMD_PONG;
    }
    uint32_t millis;
    int16_t PingRssi;
};

struct LxiCommand : Command
{
    LxiCommand()
    {
        msgType = CMD_LXI;
    }
    char Command[20];
};

#endif
