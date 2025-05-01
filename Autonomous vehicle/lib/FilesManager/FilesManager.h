#pragma once
#include <ArduinoJson.h>

namespace FilesManager
{
    void initLittleFS();
    bool readFileAsJson(JsonDocument& doc, const char* filename);
}
