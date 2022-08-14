/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #8: API Rest PZEM-014/016 AC Energy Meter
*/

//Librerias necesarias
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <TaskScheduler.h>

/*
  Configuramos credenciales WiFi
*/
#define ssid "NETLIFE-BAZURTO"
#define password "0990919594"

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

// WEB SERVER
AsyncWebServer server(80);

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// TASK
Scheduler runner;

void muestreoCallback(void);
Task task_muestreoCallback(500, TASK_FOREVER, &muestreoCallback, &runner);

void printIP(void);
Task task_printIP(5000, TASK_FOREVER, &printIP, &runner);

void setup() {
  //Iniciamos el Serial
  Serial.begin(115200);

  //Nos conectamos al WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }

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


  // Configuro la REST API
  server.on("/all", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<400> doc;
    doc["voltaje"] = Sensor.voltaje;
    doc["corriente"] = Sensor.corriente;
    doc["potencia"] = Sensor.potencia;
    doc["energia"] = Sensor.energia;
    doc["frecuencia"] = Sensor.frecuencia;
    doc["FP"] = Sensor.FP;
    char buffer[100];
    serializeJson(doc, buffer);
    request->send(200, "application/json", buffer);
  });

  server.on("/voltaje", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<400> doc;
    doc["voltaje"] = Sensor.voltaje;
    char buffer[100];
    serializeJson(doc, buffer);
    request->send(200, "application/json", buffer);
  });

  server.on("/corriente", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<400> doc;
    doc["corriente"] = Sensor.corriente;
    char buffer[100];
    serializeJson(doc, buffer);
    request->send(200, "application/json", buffer);
  });

  server.on("/potencia", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<400> doc;
    doc["potencia"] = Sensor.potencia;
    char buffer[100];
    serializeJson(doc, buffer);
    request->send(200, "application/json", buffer);
  });

  server.on("/energia", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<400> doc;
    doc["energia"] = Sensor.energia;
    char buffer[100];
    serializeJson(doc, buffer);
    request->send(200, "application/json", buffer);
  });

  server.on("/frecuencia", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<400> doc;
    doc["frecuencia"] = Sensor.frecuencia;
    char buffer[100];
    serializeJson(doc, buffer);
    request->send(200, "application/json", buffer);
  });

  // Iniciamos la REST API
  server.onNotFound(notFound);
  server.begin();

  // Iniciamos el task
  task_muestreoCallback.enable();
  task_printIP.enable();
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

void printIP(void) {
  IPAddress IP = WiFi.localIP();
  Serial.println("AP IP address: ");
  Serial.println(IP);
  Serial.println(" ");
}

void muestreoCallback(void)
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
