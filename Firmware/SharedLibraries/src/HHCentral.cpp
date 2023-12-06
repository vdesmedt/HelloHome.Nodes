#include <HHCentral.h>
#include <HHControllerCommand.h>
#include <avr/wdt.h>
#include <MemoryFree.h>

void resetFunc()
{
    wdt_enable(WDTO_1S);
    while (true)
        ;
}

HHCentral::HHCentral(HHLogger *logger, enum NodeType t_nodeType, const char *t_version, enum HHEnv t_environment)
{
    m_logger = logger;
    m_version = t_version;
    m_environment = t_environment;
    m_nodeType = t_nodeType;    
}

// Initialize config from Flash (if present)
// Initialize Radio with config from flash or dfazult (NodeId 253)
// Connect to HelloHome Gateway (Send Startup)
// Retrieve Config (including NodeId)
// Store config in flash
// Puts flash asleep
//If timeout is <= 0 or not specified, will reset the device until response is received
HHCErr HHCentral::connect(int timeout)
{
    m_flash = new SPIFlash(FLASH_SS, 0xEF30);
    if (!m_flash->initialize())
        return HHCErr_FlashInitFailed;
    m_flash->readUniqueId();
    
    m_logger->setLevel(getRegisterValue(HHRegister::LogMode, 4));

    // Init radio from flash/default config
    m_radio = new RFM69();    
    uint8_t rf_netId = m_environment == HHEnv::Pro ? 51 : 50;
    setRegisterValue(HHRegister::NetworkId, rf_netId);
    uint16_t rf_nodeId = getRegisterValue(HHRegister::NodeId, RF_DEFAULT_NODEID);
    m_radio->initialize(RF69_868MHZ, rf_nodeId, rf_netId);
    m_radio->encrypt(RF_ENCRYPT_KEY);
    bool rf_hp = getRegisterValue(HHRegister::HighPower, 1) == 1;
    if (rf_hp)
        m_radio->setHighPower();
    m_logger->logInfo(HHL_RF_INIT_UUS, rf_nodeId, rf_netId, rf_hp ? "HW" : "H");
    uint16_t startCount = getRegisterValue(HHRegister::StartCount, 0);
    
    // Send startup report
    NodeStartedReport nodeStartedMsg;
    nodeStartedMsg.startCount = startCount;
    nodeStartedMsg.nodeType = m_nodeType;
    memcpy(nodeStartedMsg.signature, m_flash->UNIQUEID, 8);
    strncpy(nodeStartedMsg.version, m_version, 7);
    sendReport(&nodeStartedMsg, sizeof(NodeStartedReport));
    setRegisterValue(HHRegister::StartCount, ++startCount);

    // Wait for response (config)
    if (!waitRf(timeout == 0 ? 10000 : timeout))
    {
        if (timeout > 0)
            return HHCErr_NoResponseFromGateway;
        m_logger->logCritical(HHL_RESTARTING);
        resetFunc();
    }

    // Check data if compatible with config
    if (m_radio->DATALEN != sizeof(NodeConfigCommand) || m_radio->DATA[0] != (2 + (0 << 2)))
    {
        if (m_radio->ACKRequested())
            m_radio->sendACK();
        m_logger->logCritical(HHL_NODECONFIG_EXPECTED_DD, sizeof(NodeConfigCommand), m_radio->DATALEN);
        if (timeout > 0)
            return HHCErr_DataIsNoConfig;
        resetFunc();
    }

    // Put response in NodeConfigCommand struct & ack
    NodeConfigCommand receivedConfig;
    memcpy(&receivedConfig, (const void *)m_radio->DATA, sizeof(NodeConfigCommand));
    if (m_radio->ACKRequested())
        m_radio->sendACK();

    // Check config is for right node by testing signature
    if (memcmp((const void *)&receivedConfig.signature, m_flash->UNIQUEID, 8))
    {
        m_logger->logWarn(HHL_SIG_MISMATCH);
        if (timeout > 0)
            return HHCErr_SignatureMismatch;
        m_logger->logCritical(HHL_RESTARTING);
        resetFunc();
    }

    // Update flash config if needed (nodeId)    
    if (rf_nodeId != receivedConfig.newNodeId)
    {
        setRegisterValue(HHRegister::NodeId, receivedConfig.newNodeId);
        m_logger->logInfo(HHL_NEW_CONFIG_DD, receivedConfig.newNodeId);
        m_radio->setAddress(receivedConfig.newNodeId);
    }
    setRegisterValue(HHRegister::Features, receivedConfig.features);
    setRegisterValue(HHRegister::NodeInfoReportPeriod, receivedConfig.nodeInfoFreq);
    setRegisterValue(HHRegister::EnvironmentReportPeriod, receivedConfig.environmentFreq);
    saveRegisterToFlash();
    m_flash->sleep();
    return HHCNoErr;
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
        m_logger->logTrace(HHL_RFM_RECEIVED);
        if(m_logger->getLevel() >= 4) {
            for (int i = 0; i < m_radio->DATALEN; i++)
                m_logger->log("%02X-", m_radio->DATA[i]);
        }

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

        // Acknowledge
        if (m_radio->ACKRequested())
        {
            m_radio->sendACK();
            m_logger->logTrace(HHL_ACK_SENT);
        }

        // Restart ?
        if (cmd == &restartCmd)
        {
            m_logger->logCritical(HHL_RESTARTING);
            resetFunc();
        }
        // Ping ?
        else if (cmd == &pingCommand)
        {
            PongReport pongReport;
            pongReport.msgId = msgId++;
            pongReport.millisIn = pingCommand.millis;
            pongReport.millisOut = millis();
            pongReport.pingRssi = m_radio->RSSI;
            sendData(&pongReport, sizeof(pongReport));
        }
    }
    HHControllerCommand *ctrlcmd = nullptr;
    HHRegister reg;
    HHRegisterValue *regs;
    if (nullptr != (ctrlcmd = HHControllerCommand::receiveDone()))
    {
        switch (ctrlcmd->verb())
        {
        case HHControllerCommand::CommandVerbs::GetRegisters :
            Serial.print(F("\n::REGS"));
            if(!m_configLoaded)
                loadConfig();
            regs = m_registers;
            while(nullptr != regs) {
                Serial.print(":");
                Serial.print(regs->reg);
                regs = regs->next;
            }
            Serial.print(":;");
            break;     
        case HHControllerCommand::CommandVerbs::GetRegister :
            reg = static_cast<HHRegister>(ctrlcmd->params()[0]);
            Serial.print(F("\n::REG:"));
            Serial.print(ctrlcmd->params()[0]);
            Serial.print(":");
            Serial.print(getRegisterValue(reg));
            Serial.print(":;");
            break;   
        case HHControllerCommand::CommandVerbs::SetRegister :
            reg = static_cast<HHRegister>(ctrlcmd->params()[0]);
            setRegisterValue(reg, ctrlcmd->params()[1]);
            break;
        case HHControllerCommand::CommandVerbs::SaveRegister :
            saveRegisterToFlash();
            break;
        case HHControllerCommand::CommandVerbs::Restart :
            m_logger->logCritical(HHL_RESTARTING);
            resetFunc();
            break;
        case HHControllerCommand::CommandVerbs::GetFreeMemory :
            Serial.print(F("\n::MEM:"));
            Serial.print(freeMemory());
            Serial.print(":;");
            break;
        default:
            break;
        }
        delete ctrlcmd;
    }
    return cmd;
}

//SHould I make a version of that method that determine the size based on MsgType ?
HHCErr HHCentral::sendReport(Report *t_report, size_t t_rptSize) 
{
    t_report->msgId = msgId++;
    bool success = sendData(t_report, t_rptSize);
    return success ? HHCNoErr : HHCErr_SendFailed;    
}

bool HHCentral::sendData(const void *data, size_t dataSize)
{
    digitalWrite(LED, HIGH);
    m_logger->logTrace(HHL_SEND_MSG_D, ((uint8_t *)data)[0], ((uint8_t *)data)[1]);
    //for(int i=0 ; i< dataSize ; i++) Serial.print(((uint8_t*)data)[i], HEX);
    bool success = m_radio->sendWithRetry(RF_GTW_NODE_ID, data, dataSize, 3, 40);
    if (success)
    {
        m_logger->logTrace(HHL_OK);
    }
    else
    {
        m_logger->logTrace(HHL_NOK);
        m_sendErrorCount++;
    }
    if (m_sleepAfterSend)
    {
        m_radio->sleep();
        m_logger->logTrace(HHL_RADIO_SLEEP);
    }
    digitalWrite(LED, LOW);
    return success;
}

bool HHCentral::waitRf(int milliseconds)
{
    int d = 100;
    int retryCount = milliseconds / d;
    m_logger->logInfo(HHL_WAIT_RF);
    bool rd = m_radio->receiveDone();
    while (!rd && retryCount-- > 0)
    {
        m_logger->log(".");
        delay(d);
        rd = m_radio->receiveDone();
    }
    if (rd)
    {
        m_logger->logInfo(HHL_OK);
        return true;
    }
    m_logger->logWarn(HHL_NOK);
    return false;
}

bool HHCentral::loadConfig()
{
    char sig[4];
    m_flash->wakeup();
    m_flash->readBytes(FLASH_ADR_CONFIG, sig, 4);
    if (0 == memcmp(sig, "HHCF", 4))
    {
        uint32_t adr = FLASH_ADR_CONFIG + 4;
        m_registers = new HHRegisterValue();
        HHRegisterValue *p = m_registers;
        while (null != p)
        {
            m_flash->readBytes(adr, p, sizeof(HHRegisterValue));
            adr += sizeof(HHRegisterValue);
            if (null != p->next)
                p->next = new HHRegisterValue();
            p = p->next;
        }
        m_configLoaded = true;
        m_flash->sleep();
        return true;
    }
    else
    {
        m_dirty_registers = true;
    }
    m_flash->sleep();
    return false;
}

HHRegisterValue *HHCentral::findRegisterValue(HHRegister reg, int16_t defaultValue)
{
    HHRegisterValue *p = m_registers;
    HHRegisterValue *prev = m_registers;
    while (null != p && p->reg != reg)
    {
        prev = p;
        p = p->next;
    }
    if (null == p)
    {
        p = new HHRegisterValue();
        p->reg = reg;
        p->next = null;
        p->setValue(defaultValue);
        if (null != prev)
        {
            prev->next = p;
        }
        else
        {
            m_registers = p;
        }
        return p;
    }
    return p;
}

int16_t HHCentral::getRegisterValue(HHRegister reg, int16_t defaultValue)
{
    if (!m_configLoaded)
        loadConfig();
    HHRegisterValue *r = findRegisterValue(reg, defaultValue);
    return r->getValue();
}

void HHCentral::setRegisterValue(HHRegister reg, int16_t value)
{
    HHRegisterValue *r = findRegisterValue(reg, 0);
    if(r->getValue() != value) {
        r->setValue(value);
        m_dirty_registers = true;
    }
    if(reg == HHRegister::LogMode) {        
        m_logger->setLevel(value);
        m_logger->logTrace(HHL_LOG_LEVEL_D, value);
    }
}

void HHCentral::saveRegisterToFlash()
{
    if (null == m_registers)
        return;
    if (!m_dirty_registers) {
        m_logger->logTrace(HHL_REGS_NOT_DIRTY);
        return;
    }
    m_flash->wakeup();
    m_flash->blockErase4K(FLASH_ADR_CONFIG);
    while (m_flash->busy())
        ;
    m_flash->writeBytes(FLASH_ADR_CONFIG, "HHCF", 4);
    HHRegisterValue *p = m_registers;
    uint32_t adr = FLASH_ADR_CONFIG + 4;
    while (null != p)
    {
        m_flash->writeBytes(adr, p, sizeof(HHRegisterValue));
        adr += sizeof(HHRegisterValue);
        p = p->next;
    }
    m_flash->sleep();
    m_logger->logTrace(HHL_REGISTER_SAVED);
    m_dirty_registers = false;
}
