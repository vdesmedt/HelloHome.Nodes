#include <stdio.h>
#include <Arduino.h>
#include <HHLogger.h>

HHLogger::HHLogger(enum LogMode logMode)
{
    m_logMode = logMode;
    if (logMode == Text)
    {

        m_msg = (PGM_P *)malloc(HHL_LAST_MSG * sizeof(PGM_P *));
        m_msg[HHL_NOCONFIG_OR_NEWVERSION] = PSTR("No config found/new version detected. Loaded to default\n");
        m_msg[HHL_NOCONFIG_OR_NEWVERSION] = PSTR("No config found or new version detected. Loaded to default\n");
        m_msg[HHL_RF_INIT_UUS] = PSTR("Rf initialized NodeId/NetworkId/HP %d/%d/%s\n");
        m_msg[HHL_NODECONFIG_EXPECTED_DD] = PSTR("Expected NodeConfig (%d bytes) but received %d bytes.\n");
        m_msg[HHL_SIG_MISMATCH] = PSTR("Signature mismatch !\n");
        m_msg[HHL_RESTARTING] = PSTR("Restarting !\n");
        m_msg[HHL_NEW_CONFIG_DD] = PSTR("New node/feature config received: %d/%d\n");
        m_msg[HHL_SAVED_CONFIG_DDDDDD] = PSTR("Saved config confVersion/features/nodeId/startCount/niFreq/envFreq %d/%d/%d/%d/%d/%d\n");
        m_msg[HHL_OK] = PSTR("OK\n");
        m_msg[HHL_NOK] = PSTR("NOK\n");
        m_msg[HHL_RADIO_SLEEP] = PSTR("Radio -> Sleep\n");
        m_msg[HHL_SEND_MSG_D] = PSTR("Will send Msg (%d)...");
        m_msg[HHL_WAIT_RF] = PSTR("Waiting for RF response");
        m_msg[HHL_RPT_RECEIVED] = PSTR("Report received (%d)\n");
    }
}

void HHLogger::log(const char *format, ...)
{
    if (m_logMode != Off)
    {
        char buffer[DEB_BUFFER_LEN];
        va_list argptr;
        va_start(argptr, format);
        vsnprintf(buffer, sizeof(buffer), format, argptr);
        va_end(argptr);
        Serial.print(buffer);
    }
}

void HHLogger::log(int message_number, ...)
{
    switch (m_logMode)
    {
    case Off:
        break;
    case Code:
        Serial.print(F("Log Code:"));
        Serial.println(message_number);
        break;
    case Text:
        char buffer[DEB_BUFFER_LEN];

        va_list argptr;
        va_start(argptr, message_number);
        PGM_P msg = m_msg[message_number];
        vsnprintf_P(buffer, sizeof(buffer), msg, argptr);
        va_end(argptr);

        Serial.print(buffer);
        break;
    }
}