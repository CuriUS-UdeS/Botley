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

//@brief suiveurDeLigne : Tous les variables globales pour la fonction de PID qui suit ligne (PIDLigne)
struct suiveurDeLigne{
//@brief error : a quelle point on est off de la ligne
//@brief adjustement : correction a ajouter a la vitesse du moteur
//@brief lastError : memorisation pour le calcul de la derivée 

// PID Properties
const double KP   = 0.00008;
const double KI   = 0.00003;
const double KD   = 0.12  ; //mis haut pour avoir reactivite beaucoup trop intense pour faire des tournants incroyable youpi

//init des variable pour le calcul de l'erreur
float error       = 0;
float adjustment  = 0;

//derivate
double lastError  = 0.0;

//integral
double integral   = 0.0;
float integralMin = -10000.0; //juste pour eviter que lintegrale depasse
float integralMax = 10000.0; //juste pour eviter que lintegrale depasse

const float GOAL = 3500; //au millieu du board capteur 4 & 5, lit dequoi entre 0 et 7000 nous on veut [etre au centre donc 3500
const double MAX_SPEED = 0.5;
const double MIN_SPEED = 0.0;

//position de la ligne
int position; //valeur entre 0 et 7000

//valeurs pour fonction de calibration
const int Manuel       = 0; //= 0 met les valeurs pour le defi du combattant dans la calibration
const int Automatique  = 1; //= 1 call fonction pour trouver automatiquement des valeurs min et max  pour le capteur
const int Raw          = 2; //= 2 call fonction pour le debug et test de fonctionnement des capteurs
};
suiveurDeLigne FollowLine;

//moteurs
const int gauche = 0;
const int droite = 1;

//@brief displayValues : Fonction pour afficher les valeurs du capteur obtenue dans la calibration
void displayValues(){
  // print the calibration minimum values measured when emitters were on
  Serial.print("   display calib   ");
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

//@brief AutomaticCalibrateLineSensorln : Fonction pour trouver, sauvegarder et imprimer les valeurs min et max des capteurs de lignes
void AutomaticCalibrateLineSensor(){
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

}

//@brief ManualCalibrateLineSensor() : Fonction pour mettre directement les valeurs pour le defi du parcours dans la calibration du capteur
void ManualCalibrateLineSensor(){
  //valeurs min et max pour les capteurs 
  unsigned int minValues[NUM_SENSORS] = {33, 32, 31, 33, 30, 28, 30, 27,};
  unsigned int maxValues[NUM_SENSORS] = {880, 701, 745, 743, 713, 798, 693, 837}; 
  
  //initialise les valeurs de la librairie 
  qtra.calibrate(); 

  //ecrit manuellement toute les valeurs min des capteurs pour le calcul de la position de la  ligne
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtra.calibratedMinimumOn[i] = minValues[i];
  }
  
  //ecrit manuellement toute les valeurs max des capteurs pour le calcul de la position de la  ligne
  for (int j = 0; j < NUM_SENSORS; j++)
  {
    qtra.calibratedMaximumOn[j] = maxValues[j];
  }

}

//@brief litAnalogIn : fonction debug pour lire et imprimer les valeurs brutes des capteurs 
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

//@brief Calibrate_Line_Sensor : Manuel -> AutoMan = 0 ------ Automatique -> AutoMan = 1 ------- Raw -> AutoMan = 2
void Calibrate_Line_Sensor(int AutoMan) { // Automatique = 1 Manuel = 0 (pour le tuning pour le defi du combattant prendre Manuel)
  
  if (AutoMan == 0){
    ManualCalibrateLineSensor();
  }
  if (AutoMan == 1){   
    AutomaticCalibrateLineSensor();
  }
  if (AutoMan == 2){
    litAnalogIn();
  }
  Serial.println("Calition for the line follower finished ");
return;
}

//@brief ImIOnLine : fonction qui detect s'il est sur une bonne 
int ImIOnLine(){
  int line = qtra.readLine(sensorValues);

  if (0 < line && line < 7000){
    return 1;
  }
  else{
    return 0;
  }
}

void PIDLigne(){
  //lire capteur
  FollowLine.position = qtra.readLine(sensorValues); //lit dequoi entre 0 et 7000 nous on veut [etre au centre donc 3500
  
  //calculate error
  FollowLine.error = FollowLine.GOAL - FollowLine.position;
  
  //acumulate error if not in weird state
  if (abs(FollowLine.error) > 3000){
    FollowLine.integral = 0.0;
  }
  else if (abs(FollowLine.error) < 3000){
    FollowLine.integral += FollowLine.error;
  }

  //Constrait integral
  FollowLine.integral = constrain (FollowLine.integral, FollowLine.integralMin, FollowLine.integralMax);

  //calculate adjustment to motor avec PID
  FollowLine.adjustment = FollowLine.KP*FollowLine.error + FollowLine.KI*(FollowLine.integral) + FollowLine.KD*(FollowLine.error - FollowLine.lastError);
  
  // Store error for next increment
  FollowLine.lastError = FollowLine.error;

  //set les nouvelles vitesses du moteur
  MOTOR_SetSpeed(gauche, (constrain(FollowLine.MAX_SPEED - FollowLine.adjustment, FollowLine.MIN_SPEED, FollowLine.MAX_SPEED)));
  MOTOR_SetSpeed(droite, (constrain(FollowLine.MAX_SPEED + FollowLine.adjustment, FollowLine.MIN_SPEED, FollowLine.MAX_SPEED)));
}

void setup() {
  //Beginning
  Serial.begin(9600);
  Serial.print("Début du setup");
 
 //init arduino X
  BoardInit();
  Serial.println(" Board init finit");

  //S'assure que les motuers sont a 0
  MOTOR_SetSpeed(gauche, 0);
  MOTOR_SetSpeed(droite, 0);
 
 //Calibre le capteur de suiveur de ligne 
  Calibrate_Line_Sensor(FollowLine.Manuel);
  
}


void loop() {

//condition pour comencer le suiveur de ligne ex: la couleur
    //PIDLigne();


//test fonction 
int test = ImIOnLine ();

    if (test == 1) {
      Serial.println ("im on the line!") ;
    }
    else {
      Serial.println ("im not on the line");
    }
    int valuesrandom= qtra.readLine(sensorValues);
    Serial.println(valuesrandom);
      
}
   