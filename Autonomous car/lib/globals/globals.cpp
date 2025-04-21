#include <globals.h>
#include<LittleFS.h>

void setMode(int m){
  mode = m;
}

bool readFileAsJson(JsonDocument doc, const char* filename){
    File file = LittleFS.open(filename, "r");
    if (!file)
    {
      Serial.println("Error opening config file!");
      return false;
    }

    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.println("JSON parsing failed!");
        return false;
    }
    return true;
}