/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #4: Envio de multiple datos entre un master a un esclavo
  de master (Dos ESP32)

  Master
*/

//Pines Dedicados modulo PLC-LAB
#define RS485_SERIAL      Serial2

#define RS485_DE         GPIO_NUM_27          // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module
#define RS485_RE         GPIO_NUM_26          // Define RE Pin to Arduino pin. Connect RE Pin of Max485 converter module

#define RS485_SERIAL_RX   GPIO_NUM_25
#define RS485_SERIAL_TX   GPIO_NUM_14

#define Analog_pin_1     GPIO_NUM_36
#define Analog_pin_2     GPIO_NUM_39


void setup()
{
  //Serial para monitorear datos por monitor serial
  Serial.begin(115200);

  //Serial para transmision datos por RS485
  RS485_SERIAL.begin(9600, SERIAL_8N1, RS485_SERIAL_RX, RS485_SERIAL_TX);

  //Habilitamos los pines como salida
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);

  //Para configurar este equipo como master debemos dejar en alto el pin Enable
  digitalWrite(RS485_DE, HIGH);

  //Un pequeño delay
  delay(10);
}


void loop()
{
  //Obtenemos el dato de los potenciometros
  int ADC_Value1 = random(0,4052);//analogRead(Analog_pin_1);
  int ADC_Value2 = random(0,4052);//analogRead(Analog_pin_2);

  //Colocamos los valores en un array
  char msg[15];
  sprintf(msg,"%04d-%04d\n",ADC_Value1,ADC_Value2);

  //Serial Write RS-485 Bus
  RS485_SERIAL.println(msg);

  //Mostramos por monitor serial para verificar el envio
  Serial.printf("Dato enviado: %s\n", msg);

  delay(100);
}
