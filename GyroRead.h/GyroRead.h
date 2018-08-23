
#include <Arduino.h>
#include <Wire.h>

template <typename T> unsigned int I2C_writeAnything (const T& value)
{
    Wire.write((byte *) &value, sizeof (value));
    return sizeof (value);
}  // end of I2C_writeAnything

template <typename T> unsigned int I2C_readAnything(T& value)
{
    byte * p = (byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
        *p++ = Wire.read();
    return i;
}  // end of I2C_readAnything

class GyroRead
{
private:
    float angle;
    uint8_t _addr;
    float rtvalue;
public:
    void begin(_addr)
    {
        Wire.begin();
        Wire.beginTransmission(_addr);
    }
    float getAngle()
    {
        if (Wire.requestFrom(_addr, 4) == 4)
            rtvalue= I2C_readAnything (angle);
        return rtvalue;
    }
};
