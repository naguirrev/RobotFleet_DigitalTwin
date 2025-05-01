#include<LittleFS.h>
#include<ArduinoJson.h>

namespace FilesManager {

    void initLittleFS(){
        if (!LittleFS.begin(true)) {
            Serial.println("LittleFS mount failed!");
            return;
        }
        Serial.println("LittleFS mounted successfully");
    }


    bool readFileAsJson(JsonDocument& doc, const char* filename){
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
        Serial.println("File found and readed as json");
        return true;
    }
}