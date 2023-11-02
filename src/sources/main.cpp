#include <Arduino.h>
#include <librobus.h>

const int kHzWhistlePin=A0; 
const int BruitAmbiantPin=A1;

const int MAP_WIDTH = 3;
const int MAP_LENGTH = 11;

const int LEFT_IR_PIN = 24;
const int RIGHT_IR_PIN = 26;

int coordX = 1;
int coordY = 0;
int orientation = 2;

// Définition des constantes du PID (0.0007)
#define KP_RIGHT 0.006
#define KP_LEFT 0.014
#define KI 0.00
#define KD 0.1

// Définition des autres constantes
#define WHEEL_DIAMETER 7.62 // 3 pouces convertis en cm
#define PULSES_PER_REVOLUTION 3200
#define MAX_SPEED 0.3
#define MIN_SPEED 0.1

<<<<<<< Updated upstream
float error_previous = 0;
float integral = 0;
=======
// distance from wall
#define GREEN_DISTANCE 26
#define YELLOW_DISTANCE 60
#define BLUE_DISTANCE 13
>>>>>>> Stashed changes

bool isRunning = false;
bool isMazeDone = false;

<<<<<<< Updated upstream
// Une tile se compose de 4 murs, représentés par 0, 1 ou 2.
struct tile {
  int top;
  int down;
  int left;
  int right;
=======
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
int actualStep = 0;
int previousStep = 0;
String actualColor;
float startDistance;
bool isRunning = true;
bool isRaceModeActivated = false;

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
>>>>>>> Stashed changes
};

// Déclaration d'une matrice 2 dimensions contenant une tile pour chaque coordonnées
tile TileMap[MAP_WIDTH][MAP_LENGTH];

<<<<<<< Updated upstream
int CheckForFrontWall() {
  switch (orientation)
    {
    case 0:
      return TileMap[coordX][coordY].left;
      break;
    case 1:
      return TileMap[coordX][coordY].right;
      break;
    case 2:
      return TileMap[coordX][coordY].top;
      break;
    case 3:
      return TileMap[coordX][coordY].down;
      break;
    default:
      Serial.println("Impossible to check for wall !");
      break;
  }
}

void AddWall(int type) {
  if(type == 1) {
    switch (orientation)
    {
    case 0:
      TileMap[coordX][coordY].left = type;
      TileMap[coordX - 1][coordY].right = type;
      break;
    case 1:
      TileMap[coordX][coordY].right = type;
      TileMap[coordX + 1][coordY].left = type;
      break;
    case 2:
      TileMap[coordX][coordY].top = type;
      TileMap[coordX][coordY + 1].down = type;
      break;
    case 3:
      TileMap[coordX][coordY].down = type;
      TileMap[coordX][coordY - 1].top = type;
      break;
    default:
      Serial.println("Can't create a new wall ! ");
      break;
    }
  } else {
    switch (orientation)
    {
    case 0:
      TileMap[coordX][coordY].right = type;
      TileMap[coordX + 1][coordY].left = type;
      break;
    case 1:
      TileMap[coordX][coordY].left = type;
      TileMap[coordX - 1][coordY].right = type;
      break;
    case 2:
      TileMap[coordX][coordY].down = type;
      TileMap[coordX][coordY - 1].top = type;
      break;
    case 3:
      TileMap[coordX][coordY].top = type;
      TileMap[coordX][coordY + 1].down = type;
      break;
    default:
      Serial.println("Can't create a new wall ! ");
      break;
=======
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
    const double MAX_SPEED = 0.2;
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
  const float maxRetractTime = 3000.0;
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

void DetectBall(int distance)
{
    Serial.println(sharpForward.distance());
    if(sharpForward.distance() <= distance) {
        actualStep++;
    }
}

void descendCup(){ //yes astral
    SERVO_Enable(1);
    SERVO_SetAngle(1,70);
    delay(500);
}

void dropCup(){
    SERVO_Enable(1);
    SERVO_SetAngle(1,0);

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
  
  if (cup.cupDetected == 1 && cup.isArmExtended == 0){
    cup.initialTime = millis();
    if (color == cup.Green){
      ExtendArm(cup.Green);//positions on ete guess alors pt a changer
      }
    else if (color == cup.Yellow){
        ExtendArm(cup.Yellow);
>>>>>>> Stashed changes
    }
  }
}

<<<<<<< Updated upstream
bool DetectWall() {
  // Function to add a new wall if the 2 IR sensors detect something
  bool isIRLeft = !digitalRead(LEFT_IR_PIN);
  bool isIRRigth = !digitalRead(RIGHT_IR_PIN);

  if(isIRLeft && isIRRigth) {
    AddWall(1);
    return true;
  } else {
    return false;
  }
=======

float getDistanceTraveledRight() {
    static float pastPulse = 0.0;
    float presentPulse = ENCODER_Read(1);
    float distanceTraveled = (presentPulse-pastPulse)*DISTANCE_PER_PULSE;
    pastPulse = presentPulse;
    return distanceTraveled;
>>>>>>> Stashed changes
}

void TurnLeft()
{
   switch (orientation)
  {
  case 0:
    orientation = 3;
    break;
  case 1:
    orientation = 2;
    break;
  case 2:
    orientation = 0;
    break;
  case 3:
    orientation = 1;
    break;
  default:
    break;
  }
    uint8_t gauche = 0; // Moteur2
    uint8_t droite = 1; // Moteur1
    int rotation = 1885;
    bool turnCompleted = false;
 
    ENCODER_ReadReset(gauche);
    ENCODER_ReadReset(droite);
 
    Serial.println("Turning right !");
    
    while (!turnCompleted) {
        // Read current encoder positions
        float pCurrentd = ENCODER_Read(droite);
        float pCurrentg = ENCODER_Read(gauche);
        // Apply corrections to motor speeds
        MOTOR_SetSpeed(droite, 0.2);
        MOTOR_SetSpeed(gauche, -0.2);
        // Check if the robot has turned the desired angle
        if (fabs(pCurrentg) >= rotation && fabs(pCurrentd) >= rotation) {
            // Set the flag to indicate that the turn has been completed
            turnCompleted = true;
            
            // Stop the motors
            MOTOR_SetSpeed(droite, 0);
            MOTOR_SetSpeed(gauche, 0);
            
            Serial.println("Turn completed!");
        }
        delay(5);
    }
}

void TurnAround()
{
  switch (orientation)
  {
  case 0:
    orientation = 1;
    break;
  case 1:
    orientation = 0;
    break;
  case 2:
    orientation = 3;
    break;
  case 3:
    orientation = 2;
    break;
  default:
    break;
  }
    uint8_t gauche = 0; // Moteur2
    uint8_t droite = 1; // Moteur1
    int rotation = 3870;
    bool turnCompleted = false;
 
    ENCODER_ReadReset(gauche);
    ENCODER_ReadReset(droite);
 
    Serial.println("Turning right !");
    
    while (!turnCompleted) {
        // Read current encoder positions
        float pCurrentd = ENCODER_Read(droite);
        float pCurrentg = ENCODER_Read(gauche);
        // Apply corrections to motor speeds
        MOTOR_SetSpeed(droite, -0.2);
        MOTOR_SetSpeed(gauche, 0.2);
        // Check if the robot has turned the desired angle
        if (fabs(pCurrentg) >= rotation && fabs(pCurrentd) >= rotation) {
            // Set the flag to indicate that the turn has been completed
            turnCompleted = true;
            
            // Stop the motors
            MOTOR_SetSpeed(droite, 0);
            MOTOR_SetSpeed(gauche, 0);
            
            Serial.println("Turn completed!");
        }
    }
  delay(10);
}

void TurnRight()
{
  switch (orientation)
  {
  case 0:
    orientation = 2;
    break;
  case 1:
    orientation = 3;
    break;
  case 2:
    orientation = 1;
    break;
  case 3:
    orientation = 0;
    break;
  default:
    break;
  }
  uint8_t gauche = 0; // Moteur2
  uint8_t droite = 1; // Moteur1
  int rotation = 1890;
  bool turnCompleted = false;

  ENCODER_ReadReset(gauche);
  ENCODER_ReadReset(droite);

  Serial.println("Turning right !");
  
  while (!turnCompleted) {
      // Read current encoder positions
      float pCurrentd = ENCODER_Read(droite);
      float pCurrentg = ENCODER_Read(gauche);

      // Apply corrections to motor speeds
      MOTOR_SetSpeed(droite, -0.2);
      MOTOR_SetSpeed(gauche, 0.2);
      // Check if the robot has turned the desired angle
      if (fabs(pCurrentg) >= rotation && fabs(pCurrentd) >= rotation) {
          // Set the flag to indicate that the turn has been completed
          turnCompleted = true;
          
          // Stop the motors
          MOTOR_SetSpeed(droite, 0);
          MOTOR_SetSpeed(gauche, 0);
          
          Serial.println("Turn completed!");
      }
  }

  delay(10);
}


void GoForward(float distance_cm) {

  if(!isMazeDone) {
      switch (orientation)
  {
  case 0:
    coordX--;
    AddWall(2);
    break;
  case 1:
    coordX++;
    AddWall(2);
    break;
  case 2:
    coordY++;
    AddWall(2);
    break;
  case 3:
    coordY--;
    AddWall(2);
    break;
  default:
    break;
  }
  }
  
    int target_pulses = (distance_cm / (PI * WHEEL_DIAMETER)) * PULSES_PER_REVOLUTION;
    int acceleration_point = target_pulses * 0.25;
    int deceleration_point = target_pulses * 0.75;

<<<<<<< Updated upstream
=======
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

void turn90(int direction){
    float deltaOrientation = abs(bot.orientation - bot.initialOrientation);
    if(direction == 0) {
        MOTOR_SetSpeed(0,-0.2);
        MOTOR_SetSpeed(1,0.2);
        if(deltaOrientation >= M_PI/2) {
            actualStep++;
        }
    } else {
        MOTOR_SetSpeed(0,0.2);
        MOTOR_SetSpeed(1,-0.2);
        if(deltaOrientation >= M_PI/2) {
            actualStep++;
        }
    }

    updateRobotPositionAndOrientation();
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

  updateRobotPositionAndOrientation();
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
>>>>>>> Stashed changes
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    int compteur = 0;
    integral = 0;

    while(ENCODER_Read(0) < target_pulses && ENCODER_Read(1) < target_pulses) {
        compteur++;
        
<<<<<<< Updated upstream
        float error_left = target_pulses - ENCODER_Read(0);
        float error_right = target_pulses - ENCODER_Read(1);
        integral += (error_left + error_right) / 2;
=======
        leftSpeedSetpoint = rightSpeedSetpoint = cruisingSpeed;
    } else {
        if(direction == 1 && deltaOrientation >= finishAngle) {
            previousStep = actualStep;
            actualStep++;
        } else if(direction == 0 && deltaOrientation <= finishAngle) {
            previousStep = actualStep;
            actualStep++;
        }
        // Performing an arc
        // Calculate wheel speeds based on the radius and desired center speed
        float leftRadius = 1;
        float rightRadius = 1;
>>>>>>> Stashed changes

        float pid_left = KP_LEFT * error_left + KI * integral + KD * (error_left - error_previous);
        float pid_right = KP_RIGHT * error_right + KI * integral + KD * (error_right - error_previous);

        // Acceleration
        if (ENCODER_Read(0) < acceleration_point) {
            float factor = sin(PI/2 * (float)ENCODER_Read(0) / acceleration_point);
            if(!isMazeDone) {
              pid_left *= factor * MAX_SPEED;
              pid_right *= factor * MAX_SPEED;
            } else {
              pid_left *= factor * 0.8;
              pid_right *= factor * 0.8;
            }
        }
        // Deceleration
        else if (ENCODER_Read(0) > deceleration_point) {
            float factor = sin(PI/2 + PI/2 * (float)(ENCODER_Read(0) - deceleration_point) / (target_pulses - deceleration_point));
            if(!isMazeDone) {
              pid_left *= factor * MAX_SPEED;
              pid_right *= factor * MAX_SPEED;
            } else {
              pid_left *= factor * 0.8;
              pid_right *= factor * 0.8;
            }
        }
        else {
            if(!isMazeDone) {
              pid_left *= MAX_SPEED;
              pid_right *= MAX_SPEED;
            } else {
              pid_left *= 0.8;
              pid_right *= 0.8;
            }
            
        }

        float correction_left = constrain(pid_left, MIN_SPEED, MAX_SPEED);
        float correction_right = constrain(pid_right, MIN_SPEED, MAX_SPEED);

        MOTOR_SetSpeed(0, correction_left);
        MOTOR_SetSpeed(1, correction_right);

        error_previous = (error_left + error_right) / 2;

        // Serial prints for debugging
        // ... [Your debugging prints]

        delay(10); // Delay for readability
    }

    MOTOR_SetSpeed(0, 0); // Stop the left motor
    MOTOR_SetSpeed(1, 0); // Stop the right motor
}

void Turn(int newOrientation) {
  switch (orientation)
  {
  case 0:
    switch (newOrientation)
    {
    case 0:
      break;
    case 1:
      TurnAround();
      break;
    case 2:
      TurnRight();
      break;
    case 3:
      TurnLeft();
      break;
    default:
      Serial.println("IMPOSSIBLE TO TURN !");
      break;
    }
    break;
  case 1:
    switch (newOrientation)
    {
    case 0:
      TurnAround();
      break;
    case 1:
      break;
    case 2:
      TurnLeft();
      break;
    case 3:
      TurnRight();
      break;
    default:
      Serial.println("IMPOSSIBLE TO TURN !");
      break;
    }
    break;
  case 2:
    switch (newOrientation)
    {
    case 0:
      TurnLeft();
      break;
    case 1:
      TurnRight();
      break;
    case 2:
      break;
    case 3:
      TurnAround();
      break;
    default:
      Serial.println("IMPOSSIBLE TO TURN !");
      break;
    }
    break;
  case 3:
    switch (newOrientation)
    {
    case 0:
      TurnRight();
      break;
    case 1:
      TurnLeft();
      break;
    case 2:
      TurnAround();
      break;
    case 3:
      break;
    default:
      Serial.println("IMPOSSIBLE TO TURN !");
      break;
    }
    break;
  default:
    Serial.println("IMPOSSIBLE TO TURN !");
    break;
  }
}

bool DetectWhistle()
{
  bool isRunning = false;

  //lire valeur du microphone
    float MicrophoneValue=analogRead(kHzWhistlePin);
 
    //lire valeur du bruit ambiant
    float BruitAmbiantValue=analogRead(BruitAmbiantPin);
 
    float difference = (MicrophoneValue-BruitAmbiantValue); 
 
    if (difference > 50)  
    {
      isRunning = true;
      Serial.println("Whistle detected, GOGOGO !");
    }

    return isRunning;

}

void PrintRobotState() {
  Serial.print("Coordinates : (");
  Serial.print(coordX);
  Serial.print(", ");
  Serial.print(coordY);
  Serial.print(")");

  switch (orientation)
  {
  case 0:
    Serial.print("   Orientation : left");
    break;
  case 1:
    Serial.print("   Orientation : right");
    break;
  case 2:
    Serial.print("   Orientation : Top");
    break;
  case 3:
    Serial.print("   Orientation : down");
    break;
  default:
    Serial.print("   Orientation : ERROR");
    break;
  }
}

// Fonction permettant d'afficher la map dans la console, elle sert seulement pour tester et visualiser.
void PrintMap() {
  Serial.println("");

  for (int y = MAP_LENGTH - 1; y >= 0; y--) { // Start from the bottom row and go upwards
    // Print the top row of each tile
    for (int x = 0; x < MAP_WIDTH; x++) {
      if (TileMap[x][y].top == 2) {
        Serial.print("+###+");
      } else if (TileMap[x][y].top == 1) {
        Serial.print("+---+");
      } else {
        Serial.print("+   +");
      }
    }
    Serial.println(); // Move to the next row

    // Print the middle row of each tile (including walls on the left and right)
    for (int x = 0; x < MAP_WIDTH; x++) {
      if (TileMap[x][y].left == 2) {
        Serial.print("#");
      } else if (TileMap[x][y].left == 1) {
        Serial.print("|");
      } else {
        Serial.print(" ");
      }
      Serial.print("   ");
      if (TileMap[x][y].right == 2) {
        Serial.print("#");
      } else if (TileMap[x][y].right == 1) {
        Serial.print("|");
      } else {
        Serial.print(" ");
      }
    }
    Serial.println(); // Move to the next row

    // Print the bottom row of each tile
    for (int x = 0; x < MAP_WIDTH; x++) {
      if (TileMap[x][y].down == 2) {
        Serial.print("+###+");
      } else if (TileMap[x][y].down == 1) {
        Serial.print("+---+");
      } else {
        Serial.print("+   +");
      }
    }
    Serial.println(); // Move to the next row
  }
}

int FindTheWay() {
  /*
  int frontWall = CheckForFrontWall();

  if(frontWall == 0) {
    return orientation;
  }
  */

  if(TileMap[coordX][coordY].top == 0) {
    return 2;
  } else if (TileMap[coordX][coordY].left == 0) {
    return 0;
  } else if (TileMap[coordX][coordY].right == 0) {
    return 1;
  } else if (TileMap[coordX][coordY].down == 0) {
    return 3;
  }

  if(TileMap[coordX][coordY].top == 2) {
    return 2;
  } else if (TileMap[coordX][coordY].left == 2) {
    return 0;
  } else if (TileMap[coordX][coordY].right == 2) {
    return 1;
  } else if (TileMap[coordX][coordY].down == 2) {
    return 3;
  } else {
    return -1;
  }
}

void GenerateMap()
{
  for(int x = 0; x < MAP_WIDTH; x++)
  {
    for(int y = 0; y < MAP_LENGTH; y++)
    {
      // Set the left side to have only left edges
      if (x == 0) {
        TileMap[x][y].left = true;
      } else {
        TileMap[x][y].left = false;
      }
      
      // Set the right side to have only right edges
      if (x == MAP_WIDTH - 1) {
        TileMap[x][y].right = true;
      } else {
        TileMap[x][y].right = false;
      }
      
      // Set the top side to have only top edges
      if (y == 0) {
        TileMap[x][y].down = true;
      } else {
        TileMap[x][y].down = false;
      }
      
      // Set the bottom side to have only bottom edges
      if (y == MAP_LENGTH - 1) {
        TileMap[x][y].top = true;
      } else {
        TileMap[x][y].top = false;
      }

      if(y != 0 && y%2 != 0)
      {
        TileMap[x][y].left = true;
        TileMap[x][y].right = true;
      }
    }
  }
}

void Reset() {
  isRunning = false;
  isMazeDone = false;
  orientation = 2;
  coordX = 1;
  coordY = 0;

  for (int i = 0; i < MAP_WIDTH; i++) {
    for (int j = 0; j < MAP_LENGTH; j++) {
        if (TileMap[i][j].top == 2) {
            TileMap[i][j].top = 0;
        }
        if (TileMap[i][j].down == 2) {
            TileMap[i][j].down = 0;
        }
        if (TileMap[i][j].left == 2) {
            TileMap[i][j].left = 0;
        }
        if (TileMap[i][j].right == 2) {
            TileMap[i][j].right = 0;
        }
    }
  }
}

void setup() {
<<<<<<< Updated upstream
  BoardInit();
  GenerateMap();

  // Initialize the IR pins
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  /*
  TileMap[2][0].top = 1;
  TileMap[0][2].top = 1;
  TileMap[1][2].right = 1;
  TileMap[1][1].top = 1;
  TileMap[1][2].down = 1;
  TileMap[1][4].top = 1;
  TileMap[1][4].right = 1;
  TileMap[0][4].down = 1;
  TileMap[0][6].top = 1;
  TileMap[1][6].down = 1;
  TileMap[1][6].right = 1;
  TileMap[1][8].left = 1;
  TileMap[1][8].top = 1;
  TileMap[2][8].down = 1;
  */

  TileMap[1][0].top = 1;
  TileMap[1][1].down = 1;
  TileMap[1][1].top = 1;
  TileMap[1][2].down = 1;

  TileMap[1][2].top = 1;
  TileMap[1][3].down = 1;
  TileMap[1][3].top = 1;
  TileMap[1][4].down = 1;

  TileMap[1][6].top = 1;
  TileMap[1][7].down = 1;
  TileMap[1][7].top = 1;
  TileMap[1][8].down = 1;
  TileMap[1][8].top = 1;
  TileMap[1][9].down = 1;

  TileMap[1][10].left = 1;
  TileMap[1][10].right = 1;

  PrintMap();

  delay(500);
}

void loop() {
  // Check if the robot has reached the end (coordY == 9)
  if (coordY == 10 && coordX == 1) {
    isMazeDone = true;
    Turn(3);
    delay(600);
    GoForward(3000);
  }
=======
    BoardInit();
    RetractArm();
    SERVO_Enable(1);
    SERVO_SetAngle(1,150);
    SERVO_Enable(0);
    SERVO_SetAngle(1,135);
    Calibrate_Line_Sensor(FollowLine.Manuel, 0);
    delay(500);
}

void loop() {

    float actualTime = millis();
>>>>>>> Stashed changes

  /*
  if(ROBUS_IsBumper(0) == true && ROBUS_IsBumper(1) == true)
    Reset();
  */

<<<<<<< Updated upstream
  if(DetectWhistle() == true)
    isRunning = true;

  // If the maze is not yet complete
  if (!isMazeDone && isRunning) {
    // Perform actions in a specific order:
    int newOrientation;

    // Detect if there's a wall or a tape in front of the robot and update the map
    while(DetectWall() == true || CheckForFrontWall() == 1) {
      // Find the new orientation after detecting walls
      newOrientation = FindTheWay();
      if(newOrientation != -1) {
        Turn(newOrientation);
        delay(600);
      }
=======
        
        if(actualStep == 23) {
            Calibrate_Line_Sensor(FollowLine.Manuel, 0);
        }
>>>>>>> Stashed changes
    }
    // If a valid path is found, adjust the orientation and move forward
    if (newOrientation != -1) {
      Serial.println("Go forward 50 cm");
      if (!isMazeDone)
        GoForward(50);
      else
        GoForward(5000);
    }
<<<<<<< Updated upstream
  }
  
}
=======

    if(isRaceModeActivated == false){
        switch (actualStep)
        {   
            case 0:
                if(DetectWhistle()){
                    actualStep++;
                }
                break; 
            case 1:
                actualStep++;
                break;
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
                move(16, 10, 0, (M_PI/2 + 0.03) , 1, 0, false);
                break;
            case 5:
                if(startDistance == GREEN_DISTANCE)
                    move(0, 12, 24, 0, 0, 2, false);
                else
                    move(0, 12, 37.5, 0, 0, 2, false);
                break;
            case 6:
                move(16, 10, 0, (M_PI/2 + 0.03), 1, 0, false);
                break;
            case 7:
                move(0, 14, 105, 0, 0, 0, false);
                if(startDistance == GREEN_DISTANCE){
                    CupKiller(cup.Green);
                } else if(startDistance == YELLOW_DISTANCE){
                    CupKiller(cup.Yellow);
                }
                break;
            case 8:
                move(16, 10, 0, M_PI/2 , 1, 0, false);
                break;
            case 9:
                move(0, 12, 24, 0, 0, 2, false);
                break;
            case 10:
                move(16, 10, 0, M_PI/4 , 1, 0, false);
                break;
            case 11:
                move(16, 10, 0, (M_PI/4-0.05) , 1, 0, false);
                break;
            case 12:
                move(0, 20, 100, 0, 0, 0, false);
                break;
            case 13:
                actualStep++;
                break;
            case 14:
                actualStep++;
                break;
            case 15:
                cup.isArmExtended = false;
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
            case 16:
                move(0, 7, 24, 0, 0, 2, true);
                break;
            case 17:
                move(16, 10, 0, M_PI/2 , 1, 0, false);
                break;
            case 18:
                if(startDistance == GREEN_DISTANCE)
                    move(0, 12, 24, 0, 0, 2, false);
                else if (startDistance == YELLOW_DISTANCE)
                    move(0, 12, 42, 0, 0, 2, false);
                break;
            case 19:
                move(17, 10, 0, M_PI/2 , 1, 0, false);
                break;
            case 20:
                move(0, 10, 2, 0, 0, 0, false);
                break;
            case 21:
                followWall(startDistance, 0.4, 48, 0); 
                if(startDistance == GREEN_DISTANCE){
                    CupKiller(cup.Green);
                } else if(startDistance == YELLOW_DISTANCE){
                    CupKiller(cup.Yellow);
                }
                break;
            case 22:
                move(0, 12, 43, 0, 0, 0, false);
                if(startDistance == GREEN_DISTANCE){
                    CupKiller(cup.Green);
                } else if(startDistance == YELLOW_DISTANCE){
                    CupKiller(cup.Yellow);
                }
                break;
            case 23:
                move(0, 4, 8, 0, 0, 0, false);
                break;
            case 24:
                actualStep++;
                break;
            case 25:
                PIDLigne();
                DetectBall(14);
                break;
            case 26:
                descendCup();
                actualStep ++;
                break;
            case 27:
                turn90(0);
                break;
            case 28:
                dropCup();
                actualStep ++;
                break;
            case 29:
                turn90(1);
                break;
            case 30:
                PIDLigne();
                break;
            default:
                break;
        }
    } else if(isRaceModeActivated == true) {
        startDistance = YELLOW_DISTANCE;
        switch (actualStep)
        {   
            case 0:
                if(ROBUS_IsBumper(3))
                    actualStep++;
                break; 
            case 1:
                actualStep++;
                break;
            case 2:
                Calibrate_Line_Sensor(FollowLine.Manuel, cup.Yellow);
                bot.orientation = 0;
                actualStep++;
                break;
            case 3:
                move(0, 7, 24, 0, 0, 2, true);
                break;
            case 4:
                move(16, 10, 0, (M_PI/2 + 0.03) , 1, 0, false);
                break;
            case 5:
                if(startDistance == GREEN_DISTANCE)
                    move(0, 12, 24, 0, 0, 2, false);
                else
                    move(0, 12, 37.5, 0, 0, 2, false);
                break;
            case 6:
                move(16, 10, 0, (M_PI/2 + 0.03), 1, 0, false);
                break;
            case 7:
                move(0, 14, 105, 0, 0, 0, false);
                break;
            case 8:
                move(16, 10, 0, M_PI/2 , 1, 0, false);
                break;
            case 9:
                move(0, 12, 24, 0, 0, 2, false);
                break;
            case 10:
                move(16, 10, 0, M_PI/4 , 1, 0, false);
                break;
            case 11:
                move(16, 10, 0, (M_PI/4-0.05) , 1, 0, false);
                break;
            case 12:
                move(0, 20, 100, 0, 0, 0, false);
                break;
            case 13:
                actualStep = 2;
                previousStep = 2;
                break;
            default:
                break;
        }
    }


    /*
    switch (actualStep)
    {   
        case 0:
            if(DetectWhistle()){
                actualStep++;
            }
            break; 
        case 1:
            actualStep++;
            break;
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
            move(16, 10, 0, M_PI/2 , 1, 0, false);
            break;
        case 5:
            if(startDistance == GREEN_DISTANCE)
                move(0, 12, 24, 0, 0, 2, false);
            else if (startDistance == YELLOW_DISTANCE)
                move(0, 12, 42, 0, 0, 2, false);
            break;
        case 6:
            move(17, 10, 0, M_PI/2 , 1, 0, false);
            break;
        case 7:
            move(0, 10, 2, 0, 0, 0, false);
            break;
        case 8:
            followWall(startDistance, 0.4, 48, 0); 
            if(startDistance == GREEN_DISTANCE){
                CupKiller(cup.Green);
            } else if(startDistance == YELLOW_DISTANCE){
                CupKiller(cup.Yellow);
            }
            break;
        case 9:
            move(0, 12, 43, 0, 0, 0, false);
            if(startDistance == GREEN_DISTANCE){
                CupKiller(cup.Green);
            } else if(startDistance == YELLOW_DISTANCE){
                CupKiller(cup.Yellow);
            }
            break;
        case 10:
            move(0, 4, 8, 0, 0, 0, false);
            break;
        case 11:
            if(ImIOnLine() == 1) {
                PIDLigne();
                actualStep++;
            }
            break;
        case 12:
            PIDLigne();
            DetectBall(14);
            break;
        case 13:
            descendCup();
            actualStep ++;
            break;
        case 14:
            turn90(0);
            break;
        case 15:
            dropCup();
            actualStep ++;
            break;
        case 16:
            turn90(1);
            break;
        case 17:
            move(0, 4, 7, 0, 0, 2, false);
            break;
        case 18:
            if(ImIOnLine() == 0){
                PIDLigne();
            } else {
                actualStep++;
            }
            break;
        case 19:
            move(16, 10, 0, M_PI/4 , 1, 0, false);
            break;
        case 20:
            move(0, 14, 45, 0, 0, 0, false);
            break;
        case 21:
            followWall(startDistance, 0.4, 48, 2); 
            break;
        default:
            break;
    }
    */
}
>>>>>>> Stashed changes
