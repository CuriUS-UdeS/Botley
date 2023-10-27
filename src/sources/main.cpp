#include <Arduino.h>
#include <librobus.h>
#include <QTRSensors.h>

// Line Sensor Properties
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

// sensors 1 through 8 are connected to analog inputs 0 through 7, respectively
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// PID Properties
const double KP = 0.0003;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 3500; //au millieu du board capteur 4 & 5, lit dequoi entre 0 et 7000 nous on veut [etre au centre donc 3500
const unsigned char MAX_SPEED = 0.8;

//moteurs
const int gauche = 0;
const int droite = 1;

/*
int detectNoir(int InputCapteur){
  //fonction pour savoir si un capteur du suiveur de ligne voit du noir
  float valeurMax = 750;
  float valeurMin = 550;
  
  if (valeurMin <= InputCapteur <= valeurMax){
    return 1;
  }
  else{
    return 0;
  }

return 0;
}

int Suiveur_Ligne (){
  float OUT = 0.0;
  int valeurSuiveurLigne1 = analogRead (SuiveurLigne1);
  int valeurSuiveurLigne2 = analogRead (SuiveurLigne2);
  int valeurSuiveurLigne3 = analogRead (SuiveurLigne3);
  int valeurSuiveurLigne4 = analogRead (SuiveurLigne4);
  int valeurSuiveurLigne5 = analogRead (SuiveurLigne5);
  int valeurSuiveurLigne6 = analogRead (SuiveurLigne6);
  int valeurSuiveurLigne7 = analogRead (SuiveurLigne7);
  int valeurSuiveurLigne8 = analogRead (SuiveurLigne8);

  int S1 = detectNoir(valeurSuiveurLigne1);
  int S2 = detectNoir(valeurSuiveurLigne2);
  int S3 = detectNoir(valeurSuiveurLigne3);
  int S4 = detectNoir(valeurSuiveurLigne4);
  int S5 = detectNoir(valeurSuiveurLigne5);
  int S6 = detectNoir(valeurSuiveurLigne6);
  int S7 = detectNoir(valeurSuiveurLigne7);
  int S8 = detectNoir(valeurSuiveurLigne8);

  //Trouve la position des bits ex: 1 0 0 0 0 0 0 0 ou 1 1 0 0 0 0 0 0
  int position8bits[] = {0, 0, 0 , 0 , 0 , 0 , 0 , 0}; 
  if (S8 = 1){
    position8bits[0] = 1;
  } 
  if (S7 = 1){
    position8bits[1] = 1;
  }
  if (S6 = 1){
    position8bits[2] = 1;
  } 
  if (S5 = 1){
    position8bits[3] = 1;
  } 
  if (S4 = 1){
    position8bits[4] = 1;
  }
  if (S3 = 1){
    position8bits[5] = 1;
  } 
  if (S2 = 1){
    position8bits[6] = 1;
  } 
  if (S1 = 1){
    position8bits[7] = 1;
  } 

  //si la valeur est bizzard (ex il detect deux lignes)
  if ((position8bits != 2^0) &&  (position8bits != 2^0 + 2^1) &&
      (position8bits != 2^1) &&  (position8bits != 2^2 + 2^1) &&
      (position8bits != 2^2) &&  (position8bits != 2^3 + 2^2) &&
      (position8bits != 2^3) &&  (position8bits != 2^4 + 2^3) &&
      (position8bits != 2^4) &&  (position8bits != 2^5 + 2^4) &&
      (position8bits != 2^5) &&  (position8bits != 2^6 + 2^5) &&
      (position8bits != 2^6) &&  (position8bits != 2^7 + 2^6) &&
      (position8bits != 2^7)
    )
  {
    OUT = 99;
  }
  


return OUT;
}
*/

void calibrateLineSensor(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off  LED to indicate we are through with calibration

  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }

//reste a effacer et hard coder les valeurs min et max de detection
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

void setup() {
  BoardInit();
  //initialise line sensor array
  calibrateLineSensor();
  delay(500);
}

void loop() {
  // Get line position
  unsigned int position = qtra.readLine(sensorValues); //lit dequoi entre 0 et 7000 nous on veut [etre au centre donc 3500

  Serial.print(position);
  Serial.println();




  /* CODE POUR QUAND J'AI LE ROBOT
  // Compute error
  int error = GOAL - position;

  // Compute motor adjustment
  int adjustment = KP*error + KD*(error - lastError);
 
  // Store error for next increment
  lastError = error;
  
  //set les vitesses du moteur
  MOTOR_SetSpeed(gauche, (constrain(MAX_SPEED - adjustment, 0, MAX_SPEED)));
  MOTOR_SetSpeed(droite, (constrain(MAX_SPEED + adjustment, 0, MAX_SPEED)));
 */

  delay(1000);
}