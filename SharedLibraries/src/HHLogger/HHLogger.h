#ifndef HHLogger_h
#define HHLogger_h

#define DEB_BUFFER_LEN 100

#define HHL_NOCONFIG_OR_NEWVERSION 0
#define HHL_RF_INIT_UUS 1
#define HHL_RESTARTING 2
#define HHL_NODECONFIG_EXPECTED_DD 3
#define HHL_SIG_MISMATCH 4
#define HHL_NEW_CONFIG_DD 5
#define HHL_OK 7
#define HHL_NOK 8
#define HHL_RADIO_SLEEP 9
#define HHL_SEND_MSG_D 10
#define HHL_WAIT_RF 11
#define HHL_RPT_RECEIVED 12
#define HHL_REGISTER_SAVED 13
#define HHL_UNKNOWN_COMMAND_S 14
#define HHL_RFM_RECEIVED 15
#define HHL_ACK_SENT 16
#define HHL_LOG_LEVEL_D 17
#define HHL_LAST_MSG 20

class HHLogger
{
public:
    HHLogger();
    void log(const char *format, ...);
    void logTrace(int message_number, ...);
    void logInfo(int message_number, ...);
    void logWarn(int message_number, ...);
    void logCritical(int message_number, ...);
    void setLevel(uint8_t logLevel);
    uint8_t getLevel() { return m_loglevel; };
private:
    uint8_t m_loglevel = 4;
    PGM_P *m_msg;
};
#endif