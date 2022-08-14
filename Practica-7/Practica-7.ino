/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #7: WebServer PZEM-014/016 AC Energy Meter
*/

//Librerias necesarias
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ModbusMaster.h>

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

//WEB SERVER
AsyncWebServer server(80);

char* formato_html =
  "<!DOCTYPE html>"
  "<html lang=\"en\">"
  "<style>"
  "table, th, td, tr {"
  "  border:1px solid black;"
  "  width: 1000px;"
  "}"
  "</style>"
  "<head>"
  "    <meta charset=\"UTF-8\">"
  "    <meta http-equiv=\"X-UA-Compatible\" content=\"IE=edge\">"
  "    <META HTTP-EQUIV='Refresh' CONTENT='1'>"
  "    <title>ESP32 WebServer</title>"
  "    <style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }</style>"
  "</head>"
  "<body>"
  "<h2>PZEM014T RS485</h2>"
  "<table>"
  "<tr>"
  "  <th>Variable</th>"
  "  <th>Valor</th>"
  "  <th>Unidad</th>"
  "</tr>"
  "<tr>"
  "  <th>Voltaje</th>"
  "  <td>%f</td>"
  "  <td>V</td>"
  "</tr>"
  "<tr>"
  "  <th>Corriente</th>"
  "  <td>%f</td>"
  "  <td>A</td>"
  "</tr>"
  "<tr>"
  "  <th>Potencia</th>"
  "  <td>%f</td>"
  "  <td>W</td>"
  "</tr>"
  "<tr>"
  "  <th>Energia</th>"
  "  <td>%f</td>"
  "  <td>KwH</td>"
  "</tr>"
  "<tr>"
  "  <th>Frecuencia</th>"
  "  <td>%f</td>"
  "  <td>Hz</td>"
  "</tr>"
  "<tr>"
  "  <th>Factor Potencia</th>"
  "  <td>%f</td>"
  "  <td></td>"
  "</tr>"
  "</table>"
  "<p>Los datos se actualizan cada 1 segundo.</p>"
  "</body>"
  "</html>";

char buffer_html[3000];

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}


void setup()
{
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

  //Configuramos el Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    muestreoSensor(mbus_sensor, Sensor);
    //Mostramos los datos
    sprintf(buffer_html,
            formato_html,
            Sensor.voltaje,
            Sensor.corriente,
            Sensor.potencia,
            Sensor.energia,
            Sensor.frecuencia,
            Sensor.FP);
    request->send_P(200, "text/html", buffer_html);
  });

  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  IPAddress IP = WiFi.localIP();
  Serial.println("AP IP address: ");
  Serial.println(IP);
  Serial.println(" ");
  delay(5000);
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
