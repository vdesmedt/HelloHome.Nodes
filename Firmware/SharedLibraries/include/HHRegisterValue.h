#include <Arduino.h>
#include <HHCore.h>

struct HHRegisterValue
{
public:
    HHRegister reg;
    int16_t getValue() { return m_value; };
    void setValue(int16_t value) { m_value = value; };
    HHRegisterValue *next;
    void markAsDirty() { m_dirty = true; };
    void markAsClean() { m_dirty = false; }
    bool isDirty() { return m_dirty; };
private:
    int16_t m_value;
    bool m_dirty = false;
};