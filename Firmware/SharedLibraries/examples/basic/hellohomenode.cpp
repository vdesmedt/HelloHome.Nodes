#include <HHLogger.h>
#include <HHCentral.h>
#include <HHMessages.h>

HHLogger *logger;
HHCentral *hhCentral;

void setup()
{
    logger = new HHLogger();
    hhCentral = new HHCentral(logger, NodeType::Simulator, "1.0", HHEnv::Dev);
    hhCentral->connect(true);
}

void loop()
{
    Command *cmd;
    cmd = hhCentral->check();
    if (cmd != nullptr)
    {
        switch (cmd->msgType)
        {
        case CMD_NODECONFIG:
            break;
        }
    }
}