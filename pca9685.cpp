#include "pca9685.h"
#include <unistd.h>
#include <cmath>
#include <iostream>

PCA9685::PCA9685(int address, int bus) : address(address) {
    fd = wiringPiI2CSetup(address);
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C communication.\n";
        exit(1);
    }

    // Initialize PCA9685
    writeByte(0x00, 0x00); // MODE1 register
}

void PCA9685::setPWMFreq(float freq) {
    float prescaleval = 25000000.0;    // 25MHz
    prescaleval /= 4096.0;              // 12-bit
    prescaleval /= freq;
    prescaleval -= 1.0;

    int prescale = std::floor(prescaleval + 0.5);

    int oldmode = readByte(0x00);
    int newmode = (oldmode & 0x7F) | 0x10; // sleep
    writeByte(0x00, newmode);              // go to sleep
    writeByte(0xFE, prescale);             // set the prescaler
    writeByte(0x00, oldmode);
    usleep(5000);
    writeByte(0x00, oldmode | 0x80); // wake up
}

void PCA9685::setPWM(int channel, int on, int off) {
    wiringPiI2CWriteReg8(fd, 0x06 + 4 * channel, on & 0xFF);
    wiringPiI2CWriteReg8(fd, 0x07 + 4 * channel, on >> 8);
    wiringPiI2CWriteReg8(fd, 0x08 + 4 * channel, off & 0xFF);
    wiringPiI2CWriteReg8(fd, 0x09 + 4 * channel, off >> 8);
}

void PCA9685::writeByte(int reg, int value) {
    wiringPiI2CWriteReg8(fd, reg, value);
}

int PCA9685::readByte(int reg) {
    return wiringPiI2CReadReg8(fd, reg);
}
