#include<ArduinoJson.h>

int mode;
bool publish;

//FUNCTIONS
void setMode(int m);

bool readFileAsJson(JsonDocument doc, const char* filename);