
/*
  Autor: Vidal Bazurto (avbazurt@espol.edu.ec)
  GitHub: https://github.com/avbazurt/CURSO-PLC-LAB
  Practica #2: Entradas Analogicas
*/

//Entrada
#define INPUT_ANALOG   GPIO_NUM_36

//Salidas
#define LED_1   GPIO_NUM_23
#define LED_2   GPIO_NUM_22
#define LED_3   GPIO_NUM_21
#define LED_4   GPIO_NUM_19


//Constantes
bool medicion_adc;


void setup() {
  /*
    Definimos los pines para sus propositos
  */

  //Entradas
  //Para las entradas analogicas no se debe configurar

  //Salidas
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);

}

void loop() {
  //Leemos el valor de nuestra entrada, analogRead nos devolvera un valor entre 0-4095
  medicion_adc = analogRead(INPUT_ANALOG);

  //Segun la medicion encendemos los led
  if (medicion_adc > 4000) {
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
  }
  else if (medicion_adc > 3000) {
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
  }
  else if (medicion_adc > 2000) {
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_4, HIGH);
  }
  else if (medicion_adc > 1000) {
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, LOW);
    digitalWrite(LED_4, HIGH);
  }
  else {
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, LOW);
    digitalWrite(LED_4, LOW);
  }
  
  //Un pequeño delay
  delay(500);
}
