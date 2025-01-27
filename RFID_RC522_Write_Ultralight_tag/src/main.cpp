#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

// Pines
#define SS_PIN 5
#define RST_PIN 22

MFRC522 rfid(SS_PIN, RST_PIN); // Instancia de la clase

void writeUltralightPage(byte page, byte *data);
void printHex(byte *buffer, byte bufferSize);

void setup() {
  Serial.begin(115200);
  SPI.begin();          // Inicializa el bus SPI
  rfid.PCD_Init();      // Inicializa el lector MFRC522

  Serial.println(F("Ready to write to MIFARE Ultralight C tags."));
}

void loop() {
  // Verifica si hay una nueva tarjeta presente
  if (!rfid.PICC_IsNewCardPresent())
    return;

  // Lee la tarjeta presente
  if (!rfid.PICC_ReadCardSerial())
    return;

  // Identifica el tipo de tarjeta
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  if (piccType != MFRC522::PICC_TYPE_MIFARE_UL) {
    Serial.println(F("This tag is not a MIFARE Ultralight C."));
    rfid.PICC_HaltA();
    return;
  }

  Serial.println(F("Detected a MIFARE Ultralight C tag."));

  // Datos a escribir (4 bytes por página)
  byte data[4] = {0xDE, 0xAD, 0xBE, 0xEF};

  // Escribir en la página 4 (página de usuario)
  writeUltralightPage(4, data);

  // Detener la comunicación con la etiqueta
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// Función para escribir datos en una página específica
void writeUltralightPage(byte page, byte *data) {
  if (page < 4 || page > 39) { // Rango típico de páginas de usuario
    Serial.println(F("Error: Page number out of range."));
    return;
  }

  MFRC522::StatusCode status = rfid.MIFARE_Ultralight_Write(page, data, 4);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Failed to write to page "));
    Serial.print(page);
    Serial.print(F(": "));
    Serial.println(rfid.GetStatusCodeName(status));
    return;
  }

  Serial.print(F("Successfully wrote to page "));
  Serial.print(page);
  Serial.print(F(": "));
  printHex(data, 4);
  Serial.println();
}

// Imprime un buffer en formato hexadecimal
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}
