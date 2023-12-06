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
    //Individual register dirtyness was made to distinguish between default value (which need save) and loaded value (which does not)
    bool m_dirty = false;
};