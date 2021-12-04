#include "HHCentral.h"
#include <avr/wdt.h>

void resetFunc()
{
    wdt_enable(WDTO_1S);
    while (true)
        ;
}

HHCentral::HHCentral(HHLogger *logger, enum NodeType t_nodeType, const char *t_version, enum HHEnv t_environment, bool t_highPower)
{
    m_logger = logger;
    m_version = t_version;
    m_environment = t_environment;
    m_nodeType = t_nodeType;
    m_highPower = t_highPower;
}

HHCErr HHCentral::connect(int timeout)
{
    m_flash = new SPIFlash(FLASH_SS, 0xEF30);
    if (!m_flash->initialize())
        return HHCErr_FlashInitFailed;
    m_flash->readUniqueId();

    //Load config from flash
    m_flash->readBytes(FLASH_ADR, &m_config, sizeof(NodeConfig));
    m_config.startCount++;
    if (CONFIG_VERSION != m_config.confVer)
    {
        m_logger->log(HHL_NOCONFIG_OR_NEWVERSION);
        //First read or new version
        m_config.confVer = CONFIG_VERSION;
        m_config.features = 0;
        m_config.nodeId = 253;
        m_config.startCount = 1;
        m_config.environmentFreq = 0;
        m_config.nodeInfoFreq = 0;
    }

    //Init radio
    m_radio = new RFM69();
    uint8_t networkId = m_environment == HHEnv::Pro ? 51 : 50;
    m_radio->initialize(RF69_868MHZ, m_config.nodeId, networkId);
    m_radio->encrypt(RF_ENCRYPT_KEY);
    if (m_highPower)
        m_radio->setHighPower();
    m_logger->log(HHL_RF_INIT_UUS, m_config.nodeId, networkId, m_highPower ? "HW" : "H");

    //Send startup report
    NodeStartedReport nodeStartedMsg;
    nodeStartedMsg.msgId = msgId++;
    nodeStartedMsg.startCount = m_config.startCount;
    nodeStartedMsg.nodeType = m_nodeType;
    memcpy(nodeStartedMsg.signature, m_flash->UNIQUEID, 8);
    strncpy(nodeStartedMsg.version, m_version, 7);
    sendData(&nodeStartedMsg, sizeof(NodeStartedReport), false);

    //Wait for response (config)
    if (!waitRf(timeout == 0 ? 10000 : timeout))
    {
        if (timeout > 0)
            return HHCErr_NoResponseFromGateway;
        m_logger->log(HHL_RESTARTING);
        resetFunc();
    }

    //Check data if compatible with config
    if (m_radio->DATALEN != sizeof(NodeConfigCommand) || m_radio->DATA[0] != (2 + (0 << 2)))
    {
        m_logger->log(HHL_NODECONFIG_EXPECTED_DD, sizeof(NodeConfigCommand), m_radio->DATALEN);
        if (m_radio->ACKRequested())
            m_radio->sendACK();
        if (timeout > 0)
            return HHCErr_DataIsNoConfig;
        resetFunc();
    }

    //Put response in config struct & ack
    NodeConfigCommand receivedConfig;
    memcpy(&receivedConfig, (const void *)m_radio->DATA, sizeof(NodeConfigCommand));
    if (m_radio->ACKRequested())
        m_radio->sendACK();

    //Check config is for right node by testing signature
    if (memcmp((const void *)&receivedConfig.signature, m_flash->UNIQUEID, 8))
    {
        m_logger->log(HHL_SIG_MISMATCH);
        if (timeout > 0)
            return HHCErr_SignatureMismatch;
        m_logger->log(HHL_RESTARTING);
        resetFunc();
    }

    //Update flash config if needed (nodeId or features)
    if (m_config.nodeId != receivedConfig.newNodeId || m_config.features != receivedConfig.features)
    {
        m_logger->log(HHL_NEW_CONFIG_DD, m_config.nodeId, m_config.features);
        m_config.nodeId = receivedConfig.newNodeId;
        m_radio->setAddress(m_config.nodeId);
        m_config.features = receivedConfig.features;
    }
    m_config.nodeInfoFreq = receivedConfig.nodeInfoFreq;
    m_config.environmentFreq = receivedConfig.environmentFreq;
    m_flash->blockErase4K(FLASH_ADR);
    m_flash->writeBytes(FLASH_ADR, &m_config, sizeof(NodeConfig));
    m_logger->log(HHL_SAVED_CONFIG_DDDDDD, m_config.confVer, m_config.features, m_config.nodeId, m_config.startCount, m_config.nodeInfoFreq, m_config.environmentFreq);

    return HHCNoErr;
}

HHCErr HHCentral::send(NodeInfoReport *t_report)
{
    t_report->msgId = msgId++;
    bool success = sendData(t_report, sizeof(*t_report), false);
    return success ? HHCNoErr : HHCErr_SendFailed;
}

HHCErr HHCentral::send(EnvironmentReport *t_report)
{
    t_report->msgId = msgId++;
    bool success = sendData(t_report, sizeof(*t_report), false);
    return success ? HHCNoErr : HHCErr_SendFailed;
}

HHCErr HHCentral::send(PulseReport *t_report)
{
    t_report->msgId = msgId++;
    bool success = sendData(t_report, sizeof(*t_report), false);
    return success ? HHCNoErr : HHCErr_SendFailed;
}

SetRelayStateCommand setRelayStateCmd;
RestartCommand restartCmd;
PongCommand pongCommand;
LxiCommand lxiCommand;
PingCommand pingCommand;

Command *HHCentral::check()
{
    Command *cmd = nullptr;
    if (m_radio->receiveDone())
    {
        CheckForWirelessHEX(*m_radio, *m_flash, false);
        m_lastRssi = m_radio->RSSI;
        m_logger->log("Received :");
        for (int i = 0; i < m_radio->DATALEN; i++)
            m_logger->log("%02X-", m_radio->DATA[i]);
        m_logger->log("\n");

        if (m_radio->DATA[0] == restartCmd.msgType)
        {
            memcpy(&restartCmd, (const void *)m_radio->DATA, sizeof(RestartCommand));
            cmd = &restartCmd;
        }
        else if (m_radio->DATA[0] == setRelayStateCmd.msgType)
        {
            memcpy(&setRelayStateCmd, (const void *)m_radio->DATA, sizeof(SetRelayStateCommand));
            cmd = &setRelayStateCmd;
        }
        else if (m_radio->DATA[0] == pongCommand.msgType)
        {
            memcpy(&pongCommand, (const void *)m_radio->DATA, sizeof(PongCommand));
            cmd = &pongCommand;
        }
        else if (m_radio->DATA[0] == lxiCommand.msgType)
        {
            memcpy(&lxiCommand, (const void *)m_radio->DATA, sizeof(LxiCommand));
            cmd = &lxiCommand;
        }
        else if (m_radio->DATA[0] == pingCommand.msgType)
        {
            memcpy(&pingCommand, (const void *)m_radio->DATA, sizeof(PingCommand));
            cmd = &pingCommand;
        }
        if (m_radio->ACKRequested())
            m_radio->sendACK();
        if (cmd == &restartCmd)
        {
            m_logger->log(HHL_RESTARTING);
            resetFunc();
        }
        else if (cmd == &pingCommand)
        {
            PongReport pongReport;
            pongReport.msgId = msgId++;
            pongReport.millisIn = pingCommand.millis;
            pongReport.millisOut = millis();
            pongReport.pingRssi = m_radio->RSSI;
            sendData(&pongReport, sizeof(pongReport), false);
        }
    }
    return cmd;
}

bool HHCentral::sendData(const void *data, size_t dataSize, bool sleep)
{
    digitalWrite(LED, HIGH);
    m_logger->log(HHL_SEND_MSG_D, ((uint8_t *)data)[0], ((uint8_t*)data)[1]);
    bool success = m_radio->sendWithRetry(RF_GTW_NODE_ID, data, dataSize, 3, 40);
    if (success)
    {
        m_logger->log(HHL_OK);
    }
    else
    {
        m_logger->log(HHL_NOK);
        m_sendErrorCount++;
    }
    if (sleep)
    {
        m_logger->log(HHL_RADIO_SLEEP);
        m_radio->sleep();
    }
    digitalWrite(LED, LOW);
    return success;
}

bool HHCentral::waitRf(int milliseconds)
{
    int d = 100;
    int retryCount = milliseconds / d;
    m_logger->log(HHL_WAIT_RF);
    bool rd = m_radio->receiveDone();
    while (!rd && retryCount-- > 0)
    {
        m_logger->log(".");
        delay(d);
        rd = m_radio->receiveDone();
    }
    if (rd)
    {
        m_logger->log(HHL_OK);
        return true;
    }
    m_logger->log(HHL_NOK);
    return false;
}