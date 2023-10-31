#include <Arduino.h>
#include <librobus.h>
#include <QTRSensors.h>

// Line Sensor Properties
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

// sensors 1 through 8 are connected to analog inputs 0 through 7, respectively
QTRSensorsAnalog qtra((unsigned char[]) {A1, A2, A4, A5, A6, A7, A8, A9}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


// PID Properties
const double KP   = 0.00008;
const double KI   = 0.00003;
const double KD   = 0.12  ; //0.000001;

float error      = 0;
float adjustment  = 0;
//derivate
double lastError  = 0.0;
//integral
double integral   = 0.0;
float integralMin = -10000.0;
float integralMax = 10000.0;

const float GOAL = 3500; //au millieu du board capteur 4 & 5, lit dequoi entre 0 et 7000 nous on veut [etre au centre donc 3500
const double MAX_SPEED = 0.5;
const double MIN_SPEED = 0.0;


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
}

void manualCalibrateLineSensor(){
  Serial.print("   manuel calib   ");
  unsigned int minValues[NUM_SENSORS] = {33, 32, 31, 33, 30, 28, 30, 27,};
  unsigned int maxValues[NUM_SENSORS] = {880, 701, 745, 743, 713, 798, 693, 837}; 
  
  qtra.calibrate(); 

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    //Serial.print(i);
    qtra.calibratedMinimumOn[i] = minValues[i];
   //Serial.print("   finforloop1   ");
  }
  
  for (int j = 0; j < NUM_SENSORS; j++)
  {
    
    //Serial.print(j);
    qtra.calibratedMaximumOn[j] = maxValues[j];
    //Serial.print("   finforloop2   ");
  }

}



void displayValues(){
  Serial.print("   display calib   ");
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.println();
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.println();
  }
}

void litAnalogIn(){
  //fonction pour tester les valeurs 
  int A = analogRead(A1);
  int B = analogRead(A2);
  int C = analogRead(A4);
  int D = analogRead(A5);
  int E = analogRead(A6);
  int F = analogRead(A7);
  int G = analogRead(A8);
  int H = analogRead(A9);

  Serial.print("A: "); Serial.println(A);
  Serial.print("B: "); Serial.println(B);
  Serial.print("C: "); Serial.println(C);
  Serial.print("D: "); Serial.println(D);
  Serial.print("E: "); Serial.println(E);
  Serial.print("F: "); Serial.println(F);
  Serial.print("G: "); Serial.println(G);
  Serial.print("H: "); Serial.println(H);
  Serial.println();

}

void PIDLigne(){
  //acumulate error if not in weird state

  if (abs(error) > 3000){
    integral = 0.0;
  }
  else if (abs(error) < 3000){
    integral += error;
  }
  //Constrait integral
  integral = constrain (integral, integralMin, integralMax);


  //calculate adjustment (new motor speed)
  adjustment = KP*error + KI*(integral) + KD*(error - lastError);
  
  // Store error for next increment
  lastError = error;

  //MOTOR_SetSpeed(gauche, 0.4);
  //MOTOR_SetSpeed(droite, 0.4);
  //set les vitesses du moteur
  MOTOR_SetSpeed(gauche, (constrain(MAX_SPEED - adjustment, MIN_SPEED, MAX_SPEED)));
  MOTOR_SetSpeed(droite, (constrain(MAX_SPEED + adjustment, MIN_SPEED, MAX_SPEED)));


}
void setup() {
  Serial.begin(9600);
  Serial.print("allo je commence");
 
  BoardInit();
  Serial.println(" Board init finit");
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droite, 0);
 
 //Calibration du capteur
  //calibrateLineSensor(); Serial.println("Displaying values"); displayValues(); //Automatique

  manualCalibrateLineSensor();        //Manuel
  Serial.println("Calibrate finit");  //Debug
  
  //displayValues();
}

void loop() {
  // Get line position
  //litAnalogIn();
  
  //uncomment
    int position = qtra.readLine(sensorValues); //lit dequoi entre 0 et 7000 nous on veut [etre au centre donc 3500
      //Serial.print("position: ");
      //Serial.println(position);
    // Compute error
    error = GOAL - position;
    PIDLigne();
    Serial.println(error);
  //CODE POUR PID

  // Compute motor adjustment
 /* 
  if (position == 7000){
    MOTOR_SetSpeed(gauche, 0.1);
    MOTOR_SetSpeed(droite, 0.2);
  }
  else if (position == 0){
    MOTOR_SetSpeed(gauche, 0.2);
    MOTOR_SetSpeed(droite, 0.1);
  }
  else  if(position > 0 && position < 7000) {
    

  }
*/
  
  


  /*
  Serial.print("Integral: ");
  Serial.println(integral);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Adjustment: ");
  Serial.println(adjustment);
  */

  }