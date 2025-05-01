#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Configuración Wi-Fi
const char* ssid = "MOVISTAR-WIFI6-E870";
const char* password = "stQkjkzFvNqHj8MNhUXh";

// Configuración MQTT
const char* mqtt_server = "192.168.1.42"; // IP del servidor RabbitMQ
const int mqtt_port = 1883; // Puerto MQTT
const char* mqtt_topic = "test/topic"; // Tema MQTT
const char* mqtt_user = "walle"; // Usuario MQTT
const char* mqtt_password = "walle3000"; // Contraseña MQTT

WiFiClient espClient;
PubSubClient client(espClient);

// Callback para mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el tema: ");
  Serial.println(topic);

  Serial.print("Mensaje: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Conexión al Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi conectado");
}

// Intentar conexión al broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado");
      client.subscribe(mqtt_topic); // Suscribirse al tema (opcional)
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publicar un mensaje
  client.publish(mqtt_topic, "Hola desde ESP32!");
  delay(5000); // Espera 5 segundos entre mensajes
}
