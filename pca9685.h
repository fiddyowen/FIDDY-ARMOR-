#ifndef PCA9685_H
#define PCA9685_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

class PCA9685 {
public:
    PCA9685(int address = 0x40, int bus = 1);
    void setPWMFreq(float freq);
    void setPWM(int channel, int on, int off);

private:
    int fd;
    int address;
    void writeByte(int reg, int value);
    int readByte(int reg);
};

#endif // PCA9685_H
