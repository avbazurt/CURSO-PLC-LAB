/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #9: MQTT PZEM-014/016 AC Energy Meter
*/
#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ModbusMaster.h>
#include <TaskScheduler.h>
#include <ArduinoJson.h>

/*
  Configuramos credenciales WiFi
*/
#define ssid "NETLIFE-BAZURTO"
#define password "0990919594"

/*
  Credenciales MQTT
*/
#define ssid "NETLIFE-BAZURTO"
#define password "0990919594"

#define MQTT_HOST "192.168.100.28"
#define MQTT_PORT 1883
#define MQTT_USER "esp32"
#define MQTT_PASS "esp32"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;

/*
  Pines Dedicados modulo PLC-LAB
*/
#define RS485_SERIAL      Serial2

#define MAX485_DE         GPIO_NUM_27          // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module
#define MAX485_RE         GPIO_NUM_26          // Define RE Pin to Arduino pin. Connect RE Pin of Max485 converter module

#define RS485_SERIAL_RX   GPIO_NUM_25
#define RS485_SERIAL_TX   GPIO_NUM_14

/*
  Configuracion RS485
*/
#define PZEM014_MODBUS_ADDR 0x01
#define PZEM014_MODBUS_BAUD 9600

//Estructura datos Electricos Sensor
struct PZEM {
  float voltaje = 0;
  float corriente = 0;
  float potencia = 0;
  float energia = 0;
  float frecuencia = 0;
  float FP = 0;
};

//Definimos la estructura
PZEM Sensor;

//Clase Modbus necesaria
ModbusMaster mbus_sensor;

// TASK
Scheduler runner;

void envioDatosMQTT(void);
Task task_envioDatosMQTT(1000, TASK_FOREVER, &envioDatosMQTT, &runner);

void setup() {
  //Iniciamos el Serial
  Serial.begin(115200);

  //Habilitamos los pines como salida
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);

  //Iniciamos con los pines en bajo
  RS485_switch2RX();

  //Configuramos el Serial para RS485
  RS485_SERIAL.begin(9600, SERIAL_8N1, RS485_SERIAL_RX, RS485_SERIAL_TX);

  //Iniciamos el ModbusMaster
  mbus_sensor.begin(PZEM014_MODBUS_ADDR, RS485_SERIAL);

  //Agregmos los callback para pre y pos Transmision
  mbus_sensor.preTransmission(RS485_switch2TX);
  mbus_sensor.postTransmission(RS485_switch2RX);

  //Configuramos el timer MQTT
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

  //Configuramos los callback MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);

  //Nos conectamos al WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }
  connectToMqtt();
  task_envioDatosMQTT.enable();
}

void loop() {
  runner.execute();
}

void RS485_switch2TX(void)
{
  digitalWrite(MAX485_DE, HIGH);
  digitalWrite(MAX485_RE, HIGH);
}

void RS485_switch2RX(void)
{
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE, LOW);
}

void envioDatosMQTT(void) {
  if (mqttClient.connected()) {
    //Obtenemos la muestra del sensor
    muestreoSensor(mbus_sensor, Sensor);

    //Creo el JSON
    StaticJsonDocument<250> doc;
    doc["voltaje"] = Sensor.voltaje;
    doc["corriente"] = Sensor.corriente;
    doc["potencia"] = Sensor.potencia;
    doc["energia"] = Sensor.energia;
    doc["frecuencia"] = Sensor.frecuencia;
    doc["FP"] = Sensor.FP;
    char buffer[100];
    serializeJson(doc, buffer);

    uint16_t packetIdPub1 = mqttClient.publish("sensorRS485", 1, true, buffer);
    Serial.print("Publishing at QoS 1, packetId: ");
    Serial.println(packetIdPub1);
  }
}

void muestreoSensor(ModbusMaster & mbus_sensor, PZEM & Sensor)
{
  uint8_t res;

  // La trama MODBUS mostrada en el folleto se traduce en esta petición de
  // biblioteca:
  res = mbus_sensor.readInputRegisters(0x0000, 9);

  uint32_t tempdouble = 0x00000000;

  Sensor.voltaje = mbus_sensor.getResponseBuffer(0x0000) / 10.0;
  tempdouble = mbus_sensor.getResponseBuffer(0x0002) + mbus_sensor.getResponseBuffer(0x0001);
  Sensor.corriente = tempdouble / 1000.00;

  tempdouble = mbus_sensor.getResponseBuffer(0x0004)  + mbus_sensor.getResponseBuffer(0x0003);
  Sensor.potencia = tempdouble / 10.0;

  tempdouble = mbus_sensor.getResponseBuffer(0x0006) + mbus_sensor.getResponseBuffer(0x0005);
  Sensor.energia = tempdouble;

  Sensor.frecuencia = mbus_sensor.getResponseBuffer(0x0007) / 10.0;
  Sensor.FP = mbus_sensor.getResponseBuffer(0x0008) / 100.00;
}

/*
   Callback MQTT
*/

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
