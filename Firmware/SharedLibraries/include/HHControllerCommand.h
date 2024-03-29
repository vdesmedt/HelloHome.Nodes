#ifndef HHControllerCommand_h
#define HHControllerCommand_h

#include <stdint.h>

//REG 12 ?
//REG 12 245
//RESTart
class HHControllerCommand
{
    public :
        typedef enum CommandVerbs 
        {
            GetRegisters = 1,
            GetRegister = 2,
            SetRegister = 3,
            Restart = 4,
            SaveRegister = 5,
            GetFreeMemory = 6,
        } CommandVerbs;

        HHControllerCommand(CommandVerbs verb);
        // Read and Parse a command from Serial
        static HHControllerCommand* receiveDone();
        // Parse a command
        static HHControllerCommand* parse(char* commandString);
        //Properties
        CommandVerbs verb() { return m_verb; }
        int16_t* params() { return m_param; }
    private :
        CommandVerbs m_verb;
        int16_t m_param[2];
};
#endif