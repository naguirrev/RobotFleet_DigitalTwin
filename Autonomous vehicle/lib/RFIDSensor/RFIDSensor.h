#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

class RFIDSensor {
public:
    RFIDSensor(uint8_t ssPin, uint8_t rstPin);
    void begin();
    bool readCoordinateFromTag(int& row, int& col);
    
private:
    MFRC522 rfid;
    int readPageAsInt(byte page);
};
