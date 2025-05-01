#include "RFIDSensor.h"

RFIDSensor::RFIDSensor(uint8_t ssPin, uint8_t rstPin)
    : rfid(ssPin, rstPin) {}

void RFIDSensor::begin() {
    SPI.begin();            // Initialize SPI bus
    rfid.PCD_Init();        // Initialize MFRC522 RFID reader
    Serial.println(F("Ready to read MIFARE Ultralight C tags."));
}

int RFIDSensor::readPageAsInt(byte page) {
    byte buffer[18];
    byte size = sizeof(buffer);
    
    MFRC522::StatusCode status = rfid.MIFARE_Read(page, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("Error reading page "));
        Serial.print(page);
        Serial.print(F(": "));
        Serial.println(rfid.GetStatusCodeName(status));
        return -1;
    }

    int value = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
    return value;
}

bool RFIDSensor::readCoordinateFromTag(int& row, int& col) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
        
        if (piccType != MFRC522::PICC_TYPE_MIFARE_UL) {
            Serial.println(F("This tag is not a MIFARE Ultralight C."));
            return false;
        }

        Serial.println(F("Reading MIFARE Ultralight C tag..."));

        // Read coordinates: row -> page 4, col -> page 5
        row = readPageAsInt(4);
        col = readPageAsInt(5);
        
        Serial.print("Row: ");
        Serial.print(row);
        Serial.print(" ,Col: ");
        Serial.println(col);

        return true;
    }
    return false;
}
