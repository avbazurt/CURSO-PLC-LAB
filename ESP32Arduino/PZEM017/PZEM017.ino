/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/ESP32-PLC
  PZEM-017 DC Energy Meter
*/

//Librerias necesarias
#include <ModbusMaster.h>
#include "HAL_Config.h"

ModbusMaster mbus_sensor;

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

void setup()
{
  //Iniciamos el Serial
  Serial.begin(115200);

  //Habilitamos los pines como salida
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);

  //Iniciamos con los pines en bajo
  RS485_switch2RX();

  //Configuramos el Serial para RS485
  RS485_SERIAL.begin(9600, SERIAL_8N2, RS485_SERIAL_RX, RS485_SERIAL_TX);

  //Iniciamos el ModbusMaster
  mbus_sensor.begin(RK520_MODBUS_ADDR, RS485_SERIAL);

  //Agregmos los callback para pre y pos Transmision
  mbus_sensor.preTransmission(RS485_switch2TX);
  mbus_sensor.postTransmission(RS485_switch2RX);

}

void loop()
{
  uint8_t res;
  // La trama MODBUS mostrada en el folleto se traduce en esta petición de
  // biblioteca:
  res = mbus_sensor.readInputRegisters(0x0000, 6);

  uint32_t tempdouble = 0x00000000;

  Sensor.voltaje = mbus_sensor.getResponseBuffer(0x0000) / 100.0;
  Sensor.corriente = mbus_sensor.getResponseBuffer(0x0001)/100.0;

  tempdouble = (mbus_sensor.getResponseBuffer(0x0003)<< 16)  + mbus_sensor.getResponseBuffer(0x0002);
  Sensor.potencia = tempdouble / 10.0;

  tempdouble = (mbus_sensor.getResponseBuffer(0x0005)<<16) + mbus_sensor.getResponseBuffer(0x0004);
  Sensor.energia = tempdouble;


  //Mostramos por pantalla los resultados
  Serial.print(Sensor.voltaje, 1);
  Serial.print("V ");
  Serial.print(Sensor.frecuencia, 1);
  Serial.print("Hz ");
  Serial.print(Sensor.corriente, 3);
  Serial.print("A ");
  Serial.print(Sensor.potencia, 1);
  Serial.print("W ");
  Serial.print(Sensor.FP, 2);
  Serial.print("pf ");
  Serial.print(Sensor.energia, 0);
  Serial.print("Wh ");
  Serial.println();
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
