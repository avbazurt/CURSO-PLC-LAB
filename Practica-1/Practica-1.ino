/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #1: Entradas y Salidas Digitales
*/

//Entradas
#define INPUT_1   GPIO_NUM_23
#define INPUT_2   GPIO_NUM_22

//Salidas
#define LED_1   GPIO_NUM_23
#define LED_2   GPIO_NUM_22

//Constantes
bool status_input_1 = false;
bool status_input_2 = false;


void setup() {
  /*
  Definimos los pines para sus propositos
  */

  //Entradas
  pinMode(INPUT_1,INPUT_PULLDOWN);
  pinMode(INPUT_2,INPUT_PULLDOWN);

  //Salidas
  pinMode(LED_1,OUTPUT);
  pinMode(LED_2,OUTPUT);

}

void loop() {
  //Obtenemos la lectura de las entradas digitales
  status_input_1 = digitalRead(status_input_1);
  status_input_2 = digitalRead(status_input_2);
  
  //Activamos o desactivamos las salidas
  digitalWrite(LED_1,status_input_1);
  digitalWrite(LED_2,status_input_2);

  //Un pequeño delay
  delay(500);
}
