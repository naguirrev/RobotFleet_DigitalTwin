//Read RFID tag
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

// Pines
//Connect pins to ESP32 30 pines
//https://www.aranacorp.com/es/uso-de-un-modulo-rfid-con-un-esp32/
//MISO 19
//MOSI 23
//SCK 18
//Vcc 3v3
#define SS_PIN 5 //SDA (Slave select)
#define RST_PIN 22 //RESET
#define SS_PIN 5
#define RST_PIN 22

MFRC522 rfid(SS_PIN, RST_PIN); // Instancia de la clase

void printHex(byte *buffer, byte bufferSize);
void readUltralightTag();

void setup() {
  Serial.begin(115200);
  SPI.begin();          // Inicializa el bus SPI
  rfid.PCD_Init();      // Inicializa el lector MFRC522

  Serial.println(F("Ready to read MIFARE Ultralight C tags."));
}

void loop() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    readUltralightTag();
    rfid.PICC_HaltA();         // Finalized connection
    rfid.PCD_StopCrypto1();    // Stop secure communication
  }
}

int readCoordinate(byte page){
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

void readUltralightTag() {
    MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  
    if (piccType != MFRC522::PICC_TYPE_MIFARE_UL) {
      Serial.println(F("This tag is not a MIFARE Ultralight C."));
      return;
    }
  
    Serial.println(F("Reading MIFARE Ultralight C tag..."));
  
    //Read coordinates: row -> page 4, col -> page 5
    int row = readCoordinate(4);
    int col = readCoordinate(5);
    Serial.print("Row: ");
    Serial.print(row);
    Serial.print(" ,Col: ");
    Serial.println(col);
  
  }
