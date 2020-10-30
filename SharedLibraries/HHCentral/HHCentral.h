#ifndef hhcentral_h
#define hhcentral_h

#include <HhMessages.h>
#include <HHLogger.h>
#include <RFM69_OTA.h>

#ifdef __AVR_ATmega1284P__
#define LED 15      // Moteino MEGAs have LEDs on D15
#define FLASH_SS 23 // and FLASH SS on D23
#else
#define LED 9      // Moteinos have LEDs on D9
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

typedef short HHCErr;
#define HHCNoErr 0
#define HHCErr_FlashInitFailed -1
#define HHCErr_NoResponseFromGateway -2

class HHCentral
{
public:
    HHCentral(HHLogger *logger, const char *t_version, enum HHEnv t_environment);
    HHCErr connect(bool t_highPowerRf = false, int timeout = 0);
    HHCErr send(Report *report);
    HHCErr send(NodeInfoReport *report);
    Command *check();
    uint16_t sendErrorCount();
    int16_t LastRssi() { return m_lastRssi; };

private:
    bool sendData(const void *data, size_t dataSize, bool sleep);
    bool waitRf(int milliseconds);
    enum HHEnv m_environment = HHEnv::Dev;
    uint16_t m_sendErrorCount = 0;
    HHLogger *m_logger;
    NodeConfig m_config;
    RFM69 *m_radio;
    SPIFlash *m_flash;
    const char *m_version;
    int16_t m_lastRssi;
};

#endif