/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #4: Envio de multiple datos entre un master a un esclavo
  de master (Dos ESP32)

  Slave
*/

//Pines Dedicados modulo PLC-LAB
#define RS485_SERIAL      Serial2

#define RS485_DE         GPIO_NUM_27          // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module
#define RS485_RE         GPIO_NUM_26          // Define RE Pin to Arduino pin. Connect RE Pin of Max485 converter module

#define RS485_SERIAL_RX   GPIO_NUM_25
#define RS485_SERIAL_TX   GPIO_NUM_14

void setup()
{
  //Serial para monitorear datos por monitor serial
  Serial.begin(115200);

  //Serial para transmision datos por RS485
  RS485_SERIAL.begin(9600, SERIAL_8N1, RS485_SERIAL_RX, RS485_SERIAL_TX);

  //Habilitamos los pines como salida
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);

  //Para configurar este equipo como slave debemos dejar en bajo el pin Enable
  digitalWrite(RS485_DE, LOW);

  //Un pequeño delay
  delay(10);
}


void loop()
{
  while (RS485_SERIAL.available())
  {
    int ADC_Value = RS485_SERIAL.parseInt();
    Serial.printf("Dato recibido: %d\n",ADC_Value);
  }
}
