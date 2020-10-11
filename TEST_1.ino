#include <Adafruit_MLX90614.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Servo.h> 

//----------------------EJERCICIO PUERTAS ULTRASONICO
Servo servoRightT;
Servo servoLeftT;


//Variables to store positions of the servos, the distance of the ultrasonic, a flag that
//allows to rectify the change of distance and starts a variable of minimum seconds to two
int16_t posRightT, posLeftT, distanceT, initialDistanceT, continuousSecondsT = 0;
bool flagT = false;
const uint8_t minimalSecondsT = 4;
//loopTimeT and waitingDoorClosingT define the time (in milliseconds) that the events of
//your function will last, timeElapsedT and timeElapsedDoorClosingT are variables
//that will store the elapsed time
const uint8_t loopTimeT = 200;
unsigned long timeElapsedT = 0;
const uint16_t waitingDoorClosingT = 1000;
unsigned long timeElapsedDoorClosingT = 0;
//------------------------EJERCICIO PUERTAS ULTRASONICO

//------------------LCD---------

//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x27,16,2);  //
//-------------------------LCD---------------

//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------
// Variables de retardo para antirebote
volatile unsigned long last_micros;
long debouncing_time = 250; //Debouncing Time in Milliseconds

// Pin y Bandera de cambio de estado, para seleccionar Grados Celsius o Fahrenheit
const byte interruptPin = 2;
volatile byte select = LOW;
volatile byte measuring = HIGH;

// Creamos un cáracter personalizado
# define Circle 0
byte customChar[] = {
  B01110,
  B01010,
  B01110,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

// Se declara el objeto
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------

void setup() {
  Serial.begin(9600);


//------------------EJERCICIO PUERTA ULTRASONICO
 //The pins of the servos are defined
  servoLeftT.attach(3);
  servoRightT.attach(5);
  //The servos, initially, will move 90 degrees
  servoLeftT.write(90);
  servoRightT.write(90);

  //The variables invoke the millis() action
  timeElapsedT = millis();
  timeElapsedDoorClosingT = millis();
//-------------------EJERCICIO PUERTA ULTRASONICO



  
//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------
// Pin de interrupcion para cambiar de grados Celsius a Farenheit
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), SelectTemp, LOW);

  // Inicializa la LCD y el MLX90614
  lcd.init();
  mlx.begin();

  // Se crea el cáracter personalizado
  lcd.createChar(Circle, customChar);

  // Enciende la retroiluminación e imprime un mensaje.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("MLX90614 Sensor");
  delay(800);
  lcd.setCursor(0, 1);
  lcd.print("Measuring...");
  delay(800);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("--- Celsius ---");
  delay(800);

//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------

  //----------------------LCD-----------------------
  // Inicializar el LCD
  //lcd.init();
  
  //Encender la luz de fondo.
  lcd.backlight();
  //------------------------LCD----------------
  /*
  // Escribimos el Mensaje en el LCD en una posición  central.
  lcd.setCursor(10, 0);
  lcd.print("FATIMA");
  lcd.setCursor(10, 1);
  lcd.print("TE EXTRANIO");*/
}//FIN DE SETUP
float temperatura;
void loop() {
  //desplazamos una posición a la izquierda
  lcd.scrollDisplayLeft(); 
  //delay(500);
 
 // delay(1000);

//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------

      lcd.setCursor(0, 0);
      lcd.print("Ambient: "); lcd.print(mlx.readAmbientTempC());
      lcd.write(Circle); lcd.print("C");
      lcd.setCursor(0, 1);
      temperatura = mlx.readObjectTempC()+2;
      lcd.print("Object:  "); lcd.print(temperatura);
      lcd.write(Circle); lcd.print("C");
      Serial.print("Object:  ");
      Serial.println(temperatura);
      delay(250);


//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------

//-----------------------------------EJERCICIO PUERTA ULTRASONICO
  //A variable that calls the millis() function is created, then the function overflow is managed
  unsigned long currentMillisLoopT = millis();
  if ((unsigned long)(currentMillisLoopT - timeElapsedT) >= loopTimeT) {
    //The distance is obtained in real time and stored in distanceT to be compared with
    //the return value sumary(), if both distances are equal and the flag is true then
    //the door is opened, otherwise, call the function beforeCloseDoor()
    distanceT = temperatura;
    if (distanceT == summary()) {
      if (distanceT <= 38 && distanceT >= 34.6 && flagT == false)
        openDoor();
      else if (distanceT <= 34.6 && distanceT >22 && flagT == true)
        beforeCloseDoor();
    }
    timeElapsedT = millis();
  }

//-------------------------------EJERCICIO PUERTA ULTRASONICO
  
}// FIN DE LOOP



//-----------------------------------EJERCICIO PUERTA ULTRASONICO
//This function returns the average of 4 readings of the distance, its purpose is to have
//a more accurate data of the measurement
uint8_t summary() {
  uint8_t sumT = 0;
  for (uint8_t iT = 0; iT < 3; iT ++) {
    sumT = sumT + (distanceT = temperatura);
    delay(50);
  }
  initialDistanceT = sumT / 3;
  return (initialDistanceT);
}

//openDoor() generates 2 events, one is in the servomotors to change its position
//(both in opposite way) and another event changes the state of the LEDs
void openDoor() {
  flagT = true;
  posLeftT = 90;
  Serial.print("abriendo XD");
  for (posRightT = 90; posRightT >= 0; posRightT -= 1) {
    if (posLeftT <= 180) {
      posLeftT++;
      servoLeftT.write(posLeftT);
    }
    servoRightT.write(posRightT);
    delay(15);
  }

}

//A timeout of +-3 seconds is granted and calls the closeDoor() function
//You can modify the wait time in the variable minimalSecondsT
void beforeCloseDoor() {
  unsigned long currentMillisDoorT = millis();
  if ((unsigned long)(currentMillisDoorT - timeElapsedDoorClosingT) >= waitingDoorClosingT ) {
    continuousSecondsT++;
    if (continuousSecondsT == minimalSecondsT)
      closeDoor();
    timeElapsedDoorClosingT = millis();
  }
}

//This works in a manner contrary to the openDoor() function
void closeDoor() {
  flagT = false;
  posLeftT = 180;
  Serial.print("Cerrando XD");
  for (posRightT = 0; posRightT <= 90; posRightT += 1) {
    if (posLeftT >= 90) {
      posLeftT--;
      servoLeftT.write(posLeftT);
    }
    servoRightT.write(posRightT);
    delay(15);
  }
  continuousSecondsT = 0;
  
}

//-------------------------------EJERCICIO PUERTA ULTRASONICO

//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------
// Rutina de interrupción para cambiar el antirebote y cambio de estado de las escalas
void SelectTemp() {
  if ((long)(micros() - last_micros) >= debouncing_time * 1000) {
    last_micros = micros();
    select = !select;
    measuring = LOW;
  }
}
//----------------------------------EJERCICIO_MLX90614_TEST TERMOMETRO LASER -----------------------------------
