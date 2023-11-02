#include <Arduino.h>
#include <LibRobus.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <math.h>
#include <limits.h>
#include <SharpIR.h>
#include <QTRSensors.h>

// PINS CONSTANTS

const int RIGHT_RED_IR_PIN = 40;
const int RIGHT_GREEN_IR_PIN = 41;
const int LEFT_RED_IR_PIN = 42;
const int LEFT_GREEN_IR_PIN = 43;

const int kHzWhistlePin=A10;

uint16_t r, g, b, c;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ROBOT CONSTANTS

const int gauche = 0;
const int droite = 1;
#define WHEEL_DIAMETER  3
#define PULSE_PER_ROTATION 3200
#define WHEEL_BASE_WIDTH 7.375
#define WHEELS_RADIUS 7.375/2

// distance from wall
#define GREEN_DISTANCE 29
#define YELLOW_DISTANCE 60
#define BLUE_DISTANCE 13

// WALL PID CONSTANTS AND VARIABLES
SharpIR sharp(A3, 1080);

const float KP = 0.003; 
const float KI = 0.000;
const float KD = 0.00001;
const float MAX_SPEED_IN_INCHES = 34.5;
const float MAX_SPEED = 0.5;
const float DISTANCE_PER_PULSE = M_PI*WHEEL_DIAMETER/PULSE_PER_ROTATION; // La distance en pouce fait avec un pulse
float integralLineFollowing = 0;
float lastErrorLineFollowing = 0;
uint16_t lastPv = -1;

// GLOBAL VARIABLES
int actualStep = 1;
int previousStep = 0;
String actualColor;
float startDistance;
bool isRunning = true;

// MOTORS PID VALUES

struct PIDValues {
    float kp;
    float ki;
    float antiWindupGain;
    float kd;
    float dt;
    float ti;
    float error;
    float sp;
    float pv;
    float output;
    float integral;
};

// Line Sensor Properties
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

// sensors 1 through 8 are connected to analog inputs 0 through 7, respectively
QTRSensorsAnalog qtra((unsigned char[]) {A1, A2, A4, A5, A6, A7, A8, A9}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

PIDValues rightPIDValues = {6.65, 0.8, 0.0};
PIDValues leftPIDValues = {6.65, 0.8, 0.0};
PIDValues mainPIDValues;

struct Robot {
    float orientation;
    float leftMotorSpeed;
    float rightMotorSpeed;
    float xPosition;
    float yPosition;
    float initialXPosition;
    float initialYPosition;
    float initialOrientation;
};

Robot bot;

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
    const double MAX_SPEED = 0.4;
    const double MIN_SPEED = 0.0;

    //position de la ligne
    int position; //valeur entre 0 et 7000

    //valeurs pour fonction de calibration
    const int Manuel       = 0; //= 0 met les valeurs pour le defi du combattant dans la calibration
    const int Automatique  = 1; //= 1 call fonction pour trouver automatiquement des valeurs min et max  pour le capteur
    const int Raw          = 2; //= 2 call fonction pour le debug et test de fonctionnement des capteurs
};

suiveurDeLigne FollowLine;

//tout pour fonction verre 
struct StructPourCup{
  const int Yellow  = 1;
  const int Green   = 0;
  float posInitialCupKiller = 25; 
  bool cupDetected = false;
  bool isArmExtended = false;
  float initialTime = millis();
  const float maxRetractTime = 1000.0;
};
StructPourCup cup;

bool DetectWhistle()
{
  bool isRunning = false;
  int A = analogRead(kHzWhistlePin); 
  if (A > 590)
    isRunning = true;
  return isRunning;
}

void RetractArm(){
  SERVO_Enable(0);
  SERVO_SetAngle(0,110);
}
void ExtendArm(bool RightorLeft){
  
  if(RightorLeft == 0){
    SERVO_Enable(0);
    SERVO_SetAngle(0, 0);
  }

  
  if(RightorLeft == 1){
    SERVO_Enable(0);
    SERVO_SetAngle(0, 180);
  }

  cup.isArmExtended = true;
}
void CupKiller (int color){//Color: green = 0 yellow = 1
  //faire distance pis cehck quand 50cm du verre 
// position relative du robot : xPosition

  int A = 1;
  //Check pour verre
  if (color == cup.Green){
    A = digitalRead(42);
  //sort le bras s'il y a un verre
  }
  if(color == cup.Yellow){
    A = digitalRead(40);
  }
  if (A == 0){
    cup.cupDetected = 1;
  }
  
  if (cup.cupDetected == 1){
    if (color == cup.Green){
      ExtendArm(cup.Green);//positions on ete guess alors pt a changer
      }
    if (color == cup.Yellow){
    ExtendArm(cup.Yellow);
    }
  }

return;
}

float getDistanceTraveledRight() {
    static float pastPulse = 0.0;
    float presentPulse = ENCODER_Read(1);
    float distanceTraveled = (presentPulse-pastPulse)*DISTANCE_PER_PULSE;
    pastPulse = presentPulse;
    return distanceTraveled;
}

float getDistanceTraveledLeft() {
    static float pastPulse = 0.0;
    float presentPulse = ENCODER_Read(0);
    float distanceTraveled = (presentPulse-pastPulse)*DISTANCE_PER_PULSE;
    pastPulse = presentPulse;
    return distanceTraveled;
}

float getDistanceTraveled(float leftDistance, float rightDistance) {
    return (rightDistance+leftDistance)/2;
}

float getAngleVariation(float leftDistance, float rightDistance) {
    return (rightDistance - leftDistance)/(2*WHEELS_RADIUS);
}

void displayPIDValues(PIDValues values)
{
    /*
    Serial.print("Kp : ");
    Serial.print(values.kp, 5);
    Serial.print("\t Ki : ");
    Serial.print(values.ki, 5);
    Serial.print("\t Integral : ");
    Serial.print(values.integral, 5);
    Serial.print("\t Kd : ");
    Serial.print(values.kd, 5);
    Serial.print("\t Ti : ");
    Serial.print(values.ti, 5);
    Serial.print("\t dt : ");
    Serial.print(values.dt, 5);
    Serial.print("\t Sp : ");
    Serial.print(values.sp, 5);
    Serial.print("\t Pv : ");
    */

    //Serial.println(values.pv, 5);

    /*
    Serial.print("\t Error : ");
    Serial.print(values.error, 5);
    Serial.print("\t Output : ");
    Serial.print(values.output, 5);
    Serial.print("\t New speed : ");
    Serial.print(constrain((values.pv/MAX_SPEED_IN_INCHES + values.output/MAX_SPEED_IN_INCHES),0.1,0.6), 5);
    Serial.println();
    */
}

void turn360(float speed, float initialAngle)
{

}


void turn60(uint8_t direction) {
  uint8_t gauche = 0; // Moteur2
  uint8_t droite = 1; // Moteur1
  int rotation = 1260;
  bool turnCompleted = false;
  
  while (!turnCompleted) {
      // Read current encoder positions
      float pCurrentd = ENCODER_Read(droite);
      float pCurrentg = ENCODER_Read(gauche);

      // Apply corrections to motor speeds

      if(direction == 0) {
        MOTOR_SetSpeed(droite, 0.3);
        MOTOR_SetSpeed(gauche, -0.3);
      } else {
        MOTOR_SetSpeed(droite, -0.3);
        MOTOR_SetSpeed(gauche, 0.3);
      }
      
      // Check if the robot has turned the desired angle
      if (fabs(pCurrentg) >= rotation && fabs(pCurrentd) >= rotation) {
          // Set the flag to indicate that the turn has been completed
          turnCompleted = true;
          
          // Stop the motors
          MOTOR_SetSpeed(droite, 0);
          MOTOR_SetSpeed(gauche, 0);
          
      }
  }

  delay(10);
}

//@brief displayValues : Fonction pour afficher les valeurs du capteur obtenue dans la calibration
void displayValues(){
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

//@brief ManualCalibrateLineSensor() : WHITE Fonction pour mettre directement les valeurs pour le defi du parcours dans la calibration du capteur
void ManualCalibrateLineSensorWhite(){
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

//@brief ManualCalibrateLineSensor() : GREEN Fonction pour mettre directement les valeurs pour le defi du parcours dans la calibration du capteur
void ManualCalibrateLineSensorGreen(){
  //valeurs min et max pour les capteurs 
  unsigned int minValues[NUM_SENSORS] = {150, 43, 47, 90, 41, 35, 37, 34};
  unsigned int maxValues[NUM_SENSORS] = {940, 721, 841, 904, 882, 760, 804, 767};
  
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

//@brief ManualCalibrateLineSensor() : YELLOW Fonction pour mettre directement les valeurs pour le defi du parcours dans la calibration du capteur
void ManualCalibrateLineSensorYellow(){
  //valeurs min et max pour les capteurs 
  unsigned int minValues[NUM_SENSORS] = {41, 38, 38, 42, 35, 30, 33, 30};
  unsigned int maxValues[NUM_SENSORS] = {885, 676, 859, 907, 864, 822, 460, 724};
  
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
void Calibrate_Line_Sensor(int AutoMan, int color) { // Automatique = 1 Manuel = 0 (pour le tuning pour le defi du combattant prendre Manuel)
  
  if (AutoMan == 0){
    if (color == 0)
      ManualCalibrateLineSensorWhite();
    if (color == 1)
      ManualCalibrateLineSensorGreen();
    if (color == 2)
      ManualCalibrateLineSensorYellow();
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

String detectColor() {
    r = tcs.read16(TCS34725_RDATAL);
    g = tcs.read16(TCS34725_GDATAL);
    b = tcs.read16(TCS34725_BDATAL);
    c = tcs.read16(TCS34725_CDATAL);

    tcs.getRawData(&r, &g, &b, &c);
    
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    Serial.println(" ");
    
  
       // Add a delay to control the update rate (in milliseconds)
    delay(100);
    
  
    if (r > 70 && r < 90 && g > 65 && g < 75 && b > 65 && b < 80 && c > 230 && c < 250){
        Serial.println("red");
        return "red";
    }
    else if (r > 145 && r < 160 && g > 140 && g < 150 && b > 85 && b < 105 && c > 400 && c < 445)
    {
        Serial.println("yellow");
        return "yellow";
    }
    else if (r > 40 && r < 50 && g > 70 && g < 80 && b > 70 && b < 90 && c > 200 && c < 240)
    {
        Serial.println("blue");
        return "blue";
    }
    
    else if (r > 45 && r < 60 && g > 75 && g < 90 && b > 65 && b < 90 && c > 220 && c < 245)
    {
        Serial.println("green");
        return "green";
    }
    
    else if (r > 175 && r < 190 && g > 185 && g < 200 && b > 165 && b < 180 && c > 540 && c < 590)
    {
        Serial.println("white");
        return "white";
    }

    else if (r > 55 && r < 75 && g > 75 && g < 95 && b > 75 && b < 90 && c > 225 && c < 280)
    {
        Serial.println("carpet");
        return "carpet";
    }
    else {
        Serial.println("wth");
        return "wth";
    }
}
// Function to update the robot's position and orientation
void updateRobotPositionAndOrientation() {
    float leftDistance = getDistanceTraveledLeft();
    float rightDistance = getDistanceTraveledRight();
    // Calculate the change in orientation
    float angleVariation = getAngleVariation(leftDistance,rightDistance);
    float newOrientation = bot.orientation + angleVariation;

    // Update the robot's orientation
    bot.orientation = newOrientation;
    
    // Normalize the orientation to keep it between -PI and PI
    /*
    while (bot.orientation > M_PI) bot.orientation -= 2 * M_PI;
    while (bot.orientation < -M_PI) bot.orientation += 2 * M_PI;
    */
    
    // Calculate the average distance traveled
    float avgDistance = (leftDistance + rightDistance) / 2.0;

    // Update the robot's position
    bot.xPosition += avgDistance * cos(bot.orientation);
    bot.yPosition += avgDistance * sin(bot.orientation);
    
}


void followWall(float sp, float speed, float distance, int sens) {  //@brief sPour : setpoint en %, MaxSpeed est la vitesse voulue
    
    float deltaX = bot.xPosition - bot.initialXPosition;
    float deltaY = bot.yPosition - bot.initialYPosition;

    int pv = sharp.distance();

    float error = sp - pv;

    float output = KP*error + KI*lastErrorLineFollowing + KD*(error - lastErrorLineFollowing);
     
    if (error < 50){
        lastErrorLineFollowing += error;
    }
    if (error > 50){
        lastErrorLineFollowing = 0;
    }
    lastErrorLineFollowing = constrain(lastErrorLineFollowing, -1000, 1000);

    
    MOTOR_SetSpeed(0, constrain(speed - output, 0, 1));
    MOTOR_SetSpeed(1, constrain((speed + output), 0, 1));

    switch (sens)
            {
            case 0:
                if(abs(deltaX) > distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            case 1:
                if(abs(deltaX) < distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            case 2:
                if(abs(deltaY) > distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            case 3:
                if(abs(deltaY) > distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            default:
                break;
            }
    updateRobotPositionAndOrientation();
}

float getLeftSpeed() {
    static unsigned long lastTime = 0;
    static float lastPulse = 0.0;
    unsigned long currentTime = micros();
    float currentPulse = ENCODER_Read(0);
    
    // Calculate the differences
    unsigned long timeDifference = currentTime - lastTime;
    if (currentTime < lastTime) { // Handle overflow
        timeDifference = currentTime + (ULONG_MAX - lastTime) + 1;
    }
    float pulseDifference = currentPulse - lastPulse;
    
    // Calculate speed, checking for division by zero
    float speed = 0.0;
    if (timeDifference > 0) {
        speed = (pulseDifference * DISTANCE_PER_PULSE * 1000000.0) / timeDifference;
    }
    
    // Update the static variables for the next call
    lastTime = currentTime;
    lastPulse = currentPulse;
    
    return speed; // Speed is in inches per second, assuming DISTANCE_PER_PULSE is in inches.
}

float getRightSpeed() {
    static unsigned long lastTime = 0;
    static float lastPulse = 0.0;
    unsigned long currentTime = micros();
    float currentPulse = ENCODER_Read(1);
    
    // Calculate the differences
    unsigned long timeDifference = currentTime - lastTime;
    if (currentTime < lastTime) { // Handle overflow
        timeDifference = currentTime + (ULONG_MAX - lastTime) + 1;
    }
    float pulseDifference = currentPulse - lastPulse;
    
    // Calculate speed, checking for division by zero
    float speed = 0.0;
    if (timeDifference > 0) {
        speed = (pulseDifference * DISTANCE_PER_PULSE * 1000000.0) / timeDifference;
    }
    
    // Update the static variables for the next call
    lastTime = currentTime;
    lastPulse = currentPulse;
    
    return speed; // Speed is in inches per second, assuming DISTANCE_PER_PULSE is in inches.
}

void calculPID(PIDValues &values)
{
    values.dt = (millis() - values.ti)/1000;
    values.error = values.sp - values.pv;
    
    //if(abs(values.error) < 1)
        values.integral += values.error * values.dt;
    //if(abs(values.error) > 1)
    //    values.integral = 1;
    
    values.integral = constrain(values.integral, -10,10);

    if(abs(values.error) < 1)
        values.integral += values.error * values.dt;
    if(abs(values.error) > 1)
        values.integral = 0;

    /*
    if (values.pv < values.sp && values.pv > 3.45) {
        values.integral += values.error * values.dt;

    if (values.pv > values.sp) {
        values.integral -= (values.pv - values.sp) * values.antiWindupGain;
    }
    else if (values.pv < 3.45) {
        values.integral += (3.45 - values.pv) * values.antiWindupGain;
    }
    */

    values.output = (values.kp*values.error) + (values.integral * values.ki);
    values.ti = millis();

    updateRobotPositionAndOrientation();
}

void leftPID(float sp) {
    leftPIDValues.sp = sp;
    leftPIDValues.pv = getLeftSpeed();
    calculPID(leftPIDValues);
    MOTOR_SetSpeed(0, constrain((leftPIDValues.pv/MAX_SPEED_IN_INCHES + leftPIDValues.output/MAX_SPEED_IN_INCHES),0.1,0.8));
}

void rightPID(float sp) {
    rightPIDValues.sp = sp;
    rightPIDValues.pv = getRightSpeed();
    calculPID(rightPIDValues);
    MOTOR_SetSpeed(1, constrain((rightPIDValues.pv/MAX_SPEED_IN_INCHES + rightPIDValues.output/MAX_SPEED_IN_INCHES),0.1,0.8));
}

float getDistanceTraveled(int encoder_channel) {
    return ENCODER_Read(encoder_channel) * DISTANCE_PER_PULSE;
}

void goForward(float speed, float distance) {
    // Reset encoders
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    float distanceTraveledLeft = 0;
    float distanceTraveledRight = 0;
    
    while ((distanceTraveledLeft + distanceTraveledRight) / 2 < distance) {
        distanceTraveledLeft = getDistanceTraveled(0);
        distanceTraveledRight = getDistanceTraveled(1);

        leftPID(speed);
        rightPID(speed);
    }

    // Stop the motors once the target distance is reached
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}

void turnArc(float rightSpeed, float leftSpeed, float distance) {
    float distanceTraveledLeft = 0;
    float distanceTraveledRight = 0;
    
    while ((distanceTraveledLeft + distanceTraveledRight) / 2 < distance) {
        distanceTraveledLeft = getDistanceTraveled(0);
        distanceTraveledRight = getDistanceTraveled(1);

        leftPID(leftSpeed);
        rightPID(rightSpeed);
    }
}

String findColor() {
    if(ROBUS_ReadIR(3) > 120)
        return "green";
    return "yellow";
}

void initializeServos() {
    SERVO_Enable(0);
    SERVO_Enable(1);
    SERVO_SetAngle(1,160);
    SERVO_SetAngle(0,110);
    delay(500);
    SERVO_Disable(0);
    SERVO_Disable(1);
}

void deployArm(){
    SERVO_Enable(0);
    SERVO_SetAngle(0,180);
}

void retractArm(){
    SERVO_SetAngle(0,95);
    delay(500);
    SERVO_Disable(0);
}

void cupKiller() {
    deployArm();
    retractArm();
}

void captureBall(){
    SERVO_Enable(1);
    SERVO_SetAngle(1,0);
    delay(1000);
    SERVO_Disable(1);
}

void goForwardWithAcceleration(int maxSpeed, float distance, float acceleration, float deceleration, bool isDeceleration) {
    for (int i = 1; i <= (maxSpeed); i++) {
        goForward(i, acceleration);
    }

    // 1 boucle -- DistanceWanted - (Maxspeed*distanceWanted/16)

    goForward(maxSpeed,  distance - ((acceleration + deceleration)*maxSpeed));

    if(isDeceleration) {
        for (int j = maxSpeed; j > -1; j--) {
            goForward(j, deceleration);
        }
        MOTOR_SetSpeed(0,0);
        MOTOR_SetSpeed(1,0);
    }
}

void updateRobotState() {
    float leftDistance = getDistanceTraveledLeft();
    float rightDistance = getDistanceTraveledRight();
    float angleVariation = getAngleVariation(leftDistance,rightDistance);
    float newOrientation = bot.orientation + angleVariation;
    float rightSpeed = getRightSpeed();
    float leftSpeed = getLeftSpeed();

    bot.orientation = newOrientation;
    bot.rightMotorSpeed = rightSpeed;
    bot.leftMotorSpeed = leftSpeed;
}

void move(float radius, float cruisingSpeed, float distance,float finishAngle, int direction, int sens, bool lineCondition) {
    float leftSpeedSetpoint, rightSpeedSetpoint;
    float deltaX = bot.xPosition - bot.initialXPosition;
    float deltaY = bot.yPosition - bot.initialYPosition;
    float deltaOrientation = abs(bot.orientation - bot.initialOrientation);

    if (radius == 0) {
        // Going straight
        if(lineCondition) {
            if(ImIOnLine() == 1)
                actualStep++;
        } else {
            switch (sens)
            {
            case 0:
                if(abs(deltaX) > distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            case 1:
                if(abs(deltaX) < distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            case 2:
                if(abs(deltaY) > distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            case 3:
                if(abs(deltaY) > distance) {
                    previousStep = actualStep;
                    actualStep++;
                }
                break;
            default:
                break;
            }
        }
        
        leftSpeedSetpoint = rightSpeedSetpoint = cruisingSpeed;
    } else {
        if(direction == 1 && deltaOrientation >= finishAngle) {
            previousStep = actualStep;
            actualStep++;
        } else if(direction == 0 && deltaOrientation >= finishAngle) {
            previousStep = actualStep;
            actualStep++;
        }
        // Performing an arc
        // Calculate wheel speeds based on the radius and desired center speed
        float leftRadius = 1;
        float rightRadius = 1;

        if(direction == 0) {
            leftRadius = radius - WHEEL_BASE_WIDTH / 2;
            rightRadius = radius + WHEEL_BASE_WIDTH / 2;
        } else {
            leftRadius = radius + WHEEL_BASE_WIDTH / 2;
            rightRadius = radius - WHEEL_BASE_WIDTH / 2;
        }
        // Determine the speed ratio based on the radii
        float speedRatio = leftRadius / rightRadius;

        // Set the wheel speeds proportionally to the speed ratio
        leftSpeedSetpoint = cruisingSpeed * speedRatio;
        rightSpeedSetpoint = cruisingSpeed;
    }
        /*
        if (direction == 0) {
            float deltaSpeed = rightSpeedSetpoint - leftSpeedSetpoint;
            float actualSpeed = 1;
            while(actualSpeed < rightSpeedSetpoint)
            {   
                rightPID(actualSpeed);
                if(actualSpeed >= deltaSpeed) {
                    leftPID(actualSpeed - deltaSpeed);
                }
                actualSpeed = actualSpeed + 0.005;
            }
        }
        */
        // Cruising speed
        leftPID(leftSpeedSetpoint);
        rightPID(rightSpeedSetpoint);
        updateRobotPositionAndOrientation();
}

void setup() {
    BoardInit();
    RetractArm();
    /*
    initializeServos();
    pinMode(RIGHT_GREEN_IR_PIN, INPUT);
    pinMode(RIGHT_RED_IR_PIN, INPUT);
    pinMode(LEFT_GREEN_IR_PIN, INPUT);
    pinMode(LEFT_RED_IR_PIN, INPUT);

    Calibrate_Line_Sensor(FollowLine.Manuel, 0);
    */
   delay(500);
}

void loop() {
    //float actualTime = millis();

    if(previousStep - actualStep != 0) {
        bot.initialXPosition = bot.xPosition;
        bot.initialYPosition = bot.yPosition;
        bot.initialOrientation = bot.orientation;
        previousStep = actualStep;

        if(actualStep == 9) {
            Calibrate_Line_Sensor(FollowLine.Manuel, 0);
        }
    }

    /*
    if(cup.isArmExtended && actualTime - cup.initialTime >= cup.maxRetractTime) {
        RetractArm();
    }
    */

    Serial.println(actualStep);

    switch (actualStep)
    {   
        case 1:
            if(DetectWhistle()) {
                Serial.println(DetectWhistle());
            }
        case 2:
            startDistance = ROBUS_ReadIR(3);
            if(startDistance < 120) {
                startDistance = YELLOW_DISTANCE;
                Calibrate_Line_Sensor(FollowLine.Manuel, 2);
            } else if(startDistance > 120) {
                startDistance = GREEN_DISTANCE;
                Calibrate_Line_Sensor(FollowLine.Manuel, 1);
            }
            bot.orientation = 0;
            actualStep++;
            break;
        case 3:
            move(0, 7, 24, 0, 0, 2, true);
            break;
        case 4:
            move(17, 10, 0, M_PI/2 , 1, 0, false);
            break;
        case 5:
            move(0, 12, 24, 0, 0, 2, false);
            break;
        case 6:
            move(17, 10, 0, M_PI/2 , 1, 0, false);
            break;
        case 7:
            followWall(startDistance, 0.4, 48, 0);
            if(startDistance == GREEN_DISTANCE){
                CupKiller(cup.Green);
            } else if(startDistance == YELLOW_DISTANCE){
                CupKiller(cup.Yellow);
            }
            break;
        case 8:
            move(0, 12, 43, 0, 0, 0, false);
            if(startDistance == GREEN_DISTANCE){
                CupKiller(cup.Green);
            } else if(startDistance == YELLOW_DISTANCE){
                CupKiller(cup.Yellow);
            }
            break;
        case 9:
            move(0, 8, 10, 0, 0, 0, false);
            break;
        case 10:
            if(ImIOnLine() == 1) {
                PIDLigne();
                actualStep++;
            }
            break;
        case 11:
            if(ImIOnLine() == 1)
                PIDLigne();
            else 
                move(3, 10, 0, M_PI/4 , 1, 0, false);
            break;
        case 12:
            break;
        default:
            break;
    }
}
