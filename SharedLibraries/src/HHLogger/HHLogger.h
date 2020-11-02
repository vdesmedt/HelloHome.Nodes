#ifndef HHLogger_h
#define HHLogger_h

#define DEB_BUFFER_LEN 100

#define HHL_NOCONFIG_OR_NEWVERSION 0
#define HHL_RF_INIT_UUS 1
#define HHL_RESTARTING 2
#define HHL_NODECONFIG_EXPECTED_DD 3
#define HHL_SIG_MISMATCH 4
#define HHL_NEW_CONFIG_DD 5
#define HHL_SAVED_CONFIG_DDDDDD 6
#define HHL_OK 7
#define HHL_NOK 8
#define HHL_RADIO_SLEEP 9
#define HHL_SEND_MSG_D 10
#define HHL_WAIT_RF 11
#define HHL_RPT_RECEIVED 12
#define HHL_LAST_MSG 20

enum LogMode
{
    Off,
    Code,
    Text
};

class HHLogger
{
public:
    HHLogger(enum LogMode logMode);
    void log(const char *format, ...);
    void log(int message_number, ...);

private:
    enum LogMode m_logMode;
    PGM_P *m_msg;
};
#endif