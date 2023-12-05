#include <stdio.h>
#include <Arduino.h>
#include "HHLogger.h"

HHLogger::HHLogger()
{
    m_msg = (PGM_P *)malloc(HHL_LAST_MSG * sizeof(PGM_P *));
    m_msg[HHL_NOCONFIG_OR_NEWVERSION] = PSTR("\nNo config found/new version detected. Loaded to default");
    m_msg[HHL_RF_INIT_UUS] = PSTR("\nRf initialized NodeId/NetworkId/HP %d/%d/%s");
    m_msg[HHL_NODECONFIG_EXPECTED_DD] = PSTR("\nExpected NodeConfig (%d bytes) but received %d bytes.");
    m_msg[HHL_SIG_MISMATCH] = PSTR("\nSignature mismatch !");
    m_msg[HHL_RESTARTING] = PSTR("\nRestarting !");
    m_msg[HHL_NEW_CONFIG_DD] = PSTR("\nNew nodeId config received: %d");
    m_msg[HHL_OK] = PSTR("OK");
    m_msg[HHL_NOK] = PSTR("NOK");
    m_msg[HHL_RADIO_SLEEP] = PSTR("\nRadio -> Sleep");
    m_msg[HHL_SEND_MSG_D] = PSTR("\nWill send Msg (%d) with msgId(%d)...");
    m_msg[HHL_WAIT_RF] = PSTR("\nWaiting for RF response");
    m_msg[HHL_RPT_RECEIVED] = PSTR("\nReport received (%d)");
    m_msg[HHL_REGISTER_SAVED] = PSTR("\nRegisters saved in flash");
    m_msg[HHL_UNKNOWN_COMMAND_S] = PSTR("\nUnknown command :%s");
    m_msg[HHL_RFM_RECEIVED] = PSTR("\nRFM Received :");
    m_msg[HHL_ACK_SENT] = PSTR("\nACK Sent");
    m_msg[HHL_LOG_LEVEL_D] = PSTR("\nLog Level set to :%d");
    m_msg[HHL_UKN_MSG_SIZE_D] = PSTR("\nUnknown message type %d, please add to get HHCentral::getReportSize");
    m_msg[HHL_REGS_NOT_DIRTY] =PSTR("Register not dirty, no need to write flash");
}

void HHLogger::setLevel(uint8_t logLevel)
{
    m_loglevel = logLevel;
    logInfo(HHL_LOG_LEVEL_D, m_loglevel);
}

static char buffer[DEB_BUFFER_LEN];
void HHLogger::log(const char *format, ...)
{
    if (m_loglevel >= 4)
    {
        va_list argptr;
        va_start(argptr, format);
        vsnprintf(buffer, sizeof(buffer), format, argptr);
        va_end(argptr);
        Serial.print(buffer);
    }
}

void HHLogger::logTrace(int message_number, ...)
{
    if (m_loglevel >= 4)
    {
        va_list argptr;
        va_start(argptr, message_number);
        PGM_P msg = m_msg[message_number];
        vsnprintf_P(buffer, sizeof(buffer), msg, argptr);
        va_end(argptr);
        Serial.print(buffer);
    }
}
void HHLogger::logInfo(int message_number, ...)
{
    if (m_loglevel >= 3)
    {
        va_list argptr;
        va_start(argptr, message_number);
        PGM_P msg = m_msg[message_number];
        vsnprintf_P(buffer, sizeof(buffer), msg, argptr);
        va_end(argptr);
        Serial.print(buffer);
    }
}
void HHLogger::logWarn(int message_number, ...)
{
    if (m_loglevel >= 2)
    {
        va_list argptr;
        va_start(argptr, message_number);
        PGM_P msg = m_msg[message_number];
        vsnprintf_P(buffer, sizeof(buffer), msg, argptr);
        va_end(argptr);
        Serial.print(buffer);
    }
}
void HHLogger::logCritical(int message_number, ...)
{
    if (m_loglevel >= 1)
    {
        va_list argptr;
        va_start(argptr, message_number);
        PGM_P msg = m_msg[message_number];
        vsnprintf_P(buffer, sizeof(buffer), msg, argptr);
        va_end(argptr);
        Serial.print(buffer);
    }
}