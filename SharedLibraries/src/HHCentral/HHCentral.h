#ifndef hhcentral_h
#define hhcentral_h

#include "HHMessages.h"
#include "HHLogger.h"
#include <RFM69_OTA.h>

#ifdef __AVR_ATmega1284P__
#define FLASH_SS 23 // and FLASH SS on D23
#else
#define FLASH_SS 8 // and FLASH SS on D8
#endif

#define RF_ENCRYPT_KEY "passiondesfruits"
#define RF_GTW_NODE_ID 1
#define RF_IS_RFM69H

#define FLASH_ADR 0x20000
#define CONFIG_VERSION 1
struct NodeConfig
{
    uint8_t confVer; //Config version, used to understand is what is read from flash is ok
    uint16_t nodeId;
    uint16_t features;
    uint16_t startCount;
    uint8_t nodeInfoFreq;
    uint8_t environmentFreq;
};

enum HHEnv
{
    Dev,
    Pro
};

enum NodeType
{
    Default = 0,
    HelloNergie = 1,
    ElectronicLoad = 98,
    Simulator = 99,

};

typedef short HHCErr;
#define HHCNoErr 0
#define HHCErr_FlashInitFailed -1
#define HHCErr_NoResponseFromGateway -2
#define HHCErr_DataIsNoConfig -3
#define HHCErr_SignatureMismatch -4
#define HHCErr_SendFailed -5

class HHCentral
{
public:
    HHCentral(HHLogger *logger, enum NodeType t_nodeType, const char *t_version, enum HHEnv t_environment, bool t_highPower = true);
    HHCErr connect(int timeout = 0);
    HHCErr send(NodeInfoReport *report);
    HHCErr send(EnvironmentReport *report);
    HHCErr send(PulseReport *report);
    HHCErr send(VoltAmperReport *report);
    Command *check();
    uint16_t sendErrorCount() { return m_sendErrorCount; };
    int16_t LastRssi() { return m_lastRssi; };
    uint16_t NodeId() { return m_config.nodeId; };
    uint16_t Features() { return m_config.features; }
    void setRadioToSleepAfterSend() { m_sleepAfterSend = true; }

private:
    bool sendData(const void *data, size_t dataSize);
    bool waitRf(int milliseconds);
    enum HHEnv m_environment = HHEnv::Dev;
    bool m_highPower = true;
    bool m_sleepAfterSend = false;
    uint16_t m_sendErrorCount = 0;
    HHLogger *m_logger;
    NodeConfig m_config;
    RFM69 *m_radio;
    SPIFlash *m_flash;
    const char *m_version;
    int16_t m_lastRssi;
    enum NodeType m_nodeType = NodeType::Default;
    uint8_t msgId = 12;
};

#endif