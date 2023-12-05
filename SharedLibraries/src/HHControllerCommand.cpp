#include "HHControllerCommand.h"
#include <Arduino.h>

HHControllerCommand* HHControllerCommand::receiveDone()
{
    static char buffer[20];
    static uint8_t bi = 0;
    while(Serial.available() && bi < 20)
        buffer[bi++] = Serial.read();
    if(bi >= 20) {
        bi = 0;
        return nullptr;
    }
    if(bi > 2 && buffer[bi-2] == '\r' && buffer[bi-1] == '\n')
    {
        buffer[bi-2] = 0;
        bi = 0;
        return parse(buffer);
    }
    if(bi > 0 && buffer[bi-1] == 0)
    {
        bi = 0;
        return parse(buffer);
    }
    return nullptr;
}

HHControllerCommand* HHControllerCommand::parse(char* commandString)
{
    struct token
    {
        char *content;
        token *next;
    };

    static const char delimit[] = " :";
    token* first = nullptr;
    token** nextPtr = &first;
    char* tk = strtok(commandString, delimit);
    while(tk != NULL)
    {
        *nextPtr = new token();
        (*nextPtr)->content = tk;
        nextPtr = &((*nextPtr)->next);
        tk = strtok(NULL, delimit);
    }

    HHControllerCommand * cmd = nullptr;
    token* ctk = first;
    
    if(0 == strcmp("REG", ctk->content))
    {
        ctk = ctk->next;
        int r = atoi(ctk->content);
        ctk = ctk->next;
        if(0 == strcmp("?", ctk->content))
        {
            //REG:1 ?  -> Query Register 1
            cmd = new HHControllerCommand(CommandVerbs::GetRegister);
            cmd->m_param[0] = r;
        } else {
            //REG:1:5  -> Set value 5 in register 1
            int v = atoi(ctk->content);
            cmd = new HHControllerCommand(CommandVerbs::SetRegister);
            cmd->m_param[0] = r;
            cmd->m_param[1] = v;
        }        
    } else if (0 == strcmp("REGS", ctk->content) && *(ctk->next->content) == '?') {
        cmd = new HHControllerCommand(CommandVerbs::GetRegisters);
    } else if (0 == strcmp("RESTART", ctk->content)) {
        cmd = new HHControllerCommand(CommandVerbs::Restart);
    } else if (0 == strcmp("SAVE", ctk->content)) {
        cmd = new HHControllerCommand(CommandVerbs::SaveRegister);
    } else if (0 == strcmp("MEM", ctk->content) && *(ctk->next->content) == '?') {
        cmd = new HHControllerCommand(CommandVerbs::GetFreeMemory);
    } 
    ctk = first;
    token * prev;
    while(ctk != nullptr) {
        prev = ctk;
        ctk = ctk->next;
        delete prev;
    }

    return cmd;
}

HHControllerCommand::HHControllerCommand(CommandVerbs verb) {
    m_verb = verb;
}