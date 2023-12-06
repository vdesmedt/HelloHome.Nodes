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
    NodeInfoReportPeriod = 10,  //Time betwwen two sendings of the node info report (in Seconds)
    EnvironmentReportPeriod = 20, //Time between two sendings of the environmental report (in seconds)    
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