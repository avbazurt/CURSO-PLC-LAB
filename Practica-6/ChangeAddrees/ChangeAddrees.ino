/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #6: Change Address PZEM-014/016 AC Energy Meter
*/

//Librerias necesarias
#include <ModbusMaster.h>

/*
  Pines Dedicados modulo PLC-LAB
*/
#define RS485_SERIAL      Serial2

#define MAX485_DE         GPIO_NUM_27          // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module
#define MAX485_RE         GPIO_NUM_26          // Define RE Pin to Arduino pin. Connect RE Pin of Max485 converter module

#define RS485_SERIAL_RX   GPIO_NUM_25
#define RS485_SERIAL_TX   GPIO_NUM_14

#define PZEM014_MODBUS_ADDR 0x02
#define PZEM014_MODBUS_BAUD 9600

// Nueva direccion
#define PZEM014_NEW_MODBUS_ADDR 0x01


//Clase Modbus necesaria
ModbusMaster mbus_sensor;

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

  delay(500);
  
  //Procedemos a cambiar el address
  changeAddress(PZEM014_MODBUS_ADDR, PZEM014_NEW_MODBUS_ADDR);
}

void loop() {
  // put your main code here, to run repeatedly:

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

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr)
{
  static uint8_t SlaveParameter = 0x06;
  static uint16_t registerAddress = 0x0002;
  uint16_t u16CRC = 0xFFFF;
  u16CRC = crc16_update(u16CRC, OldslaveAddr);
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));

  Serial.println("Cambiando direccion exclavo");
  RS485_switch2TX();

  RS485_SERIAL.write(OldslaveAddr);
  RS485_SERIAL.write(SlaveParameter);
  RS485_SERIAL.write(highByte(registerAddress));
  RS485_SERIAL.write(lowByte(registerAddress));
  RS485_SERIAL.write(highByte(NewslaveAddr));
  RS485_SERIAL.write(lowByte(NewslaveAddr));
  RS485_SERIAL.write(lowByte(u16CRC));
  RS485_SERIAL.write(highByte(u16CRC));
  delay(10);
  RS485_switch2RX();
  delay(100);
  while (RS485_SERIAL.available())
  {
    Serial.print(char(RS485_SERIAL.read()), HEX);
    Serial.print(" ");
  }
}
