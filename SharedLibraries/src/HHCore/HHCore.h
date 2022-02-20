#ifndef HHCore_h
#define HHCore_h

typedef enum HHRegister
{
    StartCount = 1,
    NetworkId = 2,
    NodeId = 3,
    HighPower = 4,
    Features = 5,
    LogMode = 7,
    NodeInfoPeriod = 10,
    EnvironmentFreq = 20,
} HHRegister;

enum HHEnv
{
    Dev,
    Pro
};

enum NodeType
{
    Default = 0,
    HelloNergie = 1,
    HelloWeather = 2,
    ElectronicLoad = 98,
    Simulator = 99,

};

#endif