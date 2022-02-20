#ifndef hhcentral_h
#define hhcentral_h

#include <HHCore.h>
#include <HHMessages.h>
#include <HHLogger.h>
#include <RFM69_OTA.h>

#ifdef __AVR_ATmega1284P__
#define FLASH_SS 23 // and FLASH SS on D23
#else
#define FLASH_SS 8 // and FLASH SS on D8
#endif

#define RF_ENCRYPT_KEY "passiondesfruits"
#define RF_GTW_NODE_ID 1
#define RF_IS_RFM69H
#define RF_DEFAULT_NODEID 253

#define FLASH_ADR 0x20000
#define FLASH_ADR_CONFIG 0x8000 // First 0 -> 0x7FFF (32768Kb) used for Dualoptiboot
#define CONFIG_VERSION 1

typedef short HHCErr;
#define HHCNoErr 0
#define HHCErr_FlashInitFailed -1
#define HHCErr_NoResponseFromGateway -2
#define HHCErr_DataIsNoConfig -3
#define HHCErr_SignatureMismatch -4
#define HHCErr_SendFailed -5

struct HHRegisterValue
{
public:
    HHRegister reg;
    int16_t getValue() { return m_value; };
    void setValue(int16_t value) { m_value = value; };
    HHRegisterValue *next;

private:
    int16_t m_value;
};

class HHCentral
{
public:
    HHCentral(HHLogger *logger, enum NodeType t_nodeType, const char *t_version, enum HHEnv t_environment);
    HHCErr connect(int timeout = 0);    
    HHCErr sendReport(Report *t_report);
    Command *check();
    uint16_t sendErrorCount() { return m_sendErrorCount; };
    int16_t LastRssi() { return m_lastRssi; };
    uint16_t NodeId() { return getRegisterValue(HHRegister::NodeId, RF_DEFAULT_NODEID); };
    uint16_t Features() { return getRegisterValue(HHRegister::Features, 0); };
    void setRadioToSleepAfterSend() { m_sleepAfterSend = true; };

    bool loadConfig();
    int16_t getRegisterValue(HHRegister, int16_t = 0);
    void setRegisterValue(HHRegister, int16_t);
    void saveRegisterToFlash();

private:
    size_t getReportSize(Report* t_report);
    bool sendData(const void *data, size_t dataSize);
    bool waitRf(int milliseconds);
    enum HHEnv m_environment = HHEnv::Dev;
    bool m_sleepAfterSend = false;
    uint16_t m_sendErrorCount = 0;
    HHLogger *m_logger;
    RFM69 *m_radio;
    SPIFlash *m_flash;
    const char *m_version;
    int16_t m_lastRssi;
    enum NodeType m_nodeType = NodeType::Default;
    uint8_t msgId = 12;
    bool m_configLoaded = false;
    HHRegisterValue *m_registers = null;
    HHRegisterValue *findRegisterValue(HHRegister reg, int16_t defaultValue = 0);
};

#endif