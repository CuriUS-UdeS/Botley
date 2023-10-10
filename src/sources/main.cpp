#include <Arduino.h>
#include <librobus.h>

/*
//Variables globals
const int MAP_WIDTH = 3;
const int MAP_LENGTH = 10;
int test = 0;
*/
float integrale =0; 

/*
// Une tile se compose de 4 murs, ce sont des valeurs vrai ou faux.
struct tile {
  bool top;
  bool down;
  bool left;
  bool right;
};


// Déclaration d'une matrice 2 dimensions contenant une tile pour chaque coordonnées
tile TileMap[MAP_WIDTH][MAP_LENGTH];


*/
int valeurEncodeur(float distance)
//trouve le nombre de pulse pour une distance donner
{
  float pas = 0.00007552376; //m
  int pulse = 0;

  pulse = (distance / pas);

return pulse;
}

float PID (float distance)
{
  uint8_t gauche = 0; //Moteur2
  uint8_t droite = 1; //Moteur1
  float correction = 0; 
  float kP = 0.4;
  float kI = 0;
  int32_t pulseActuelGauche = ENCODER_Read(gauche);
  int32_t pulseActuelDroite = ENCODER_Read(droite);
  int pulseFinal = valeurEncodeur(distance);

  double erreur = (pulseActuelDroite/pulseFinal - pulseActuelGauche/pulseFinal);
  integrale += erreur;
  correction = correction + (erreur * kP) + (integrale*kI);
  if (correction > 1)
    {
      correction = 1;
    }
  if (correction < 0)
    {
      correction = 0;
    }

    Serial.print("kp: ");
    Serial.println(kP);

    Serial.print("droite: ");
    Serial.println(pulseActuelDroite);
    
    Serial.print("gauche: ");
    Serial.println(pulseActuelGauche);
    
    Serial.print("denominateur: ");
    Serial.println(pulseFinal);

    Serial.print("Error: ");
    Serial.println(erreur);


    Serial.print("Correction: ");

    Serial.println(correction);


return correction;
}

float limitSpeed(float speed, float maxSpeed)
{
    if (speed > maxSpeed) {
        return maxSpeed;
    } else if (speed < - maxSpeed) {
        return -maxSpeed;
    } else {
        return speed;
    }
}

float acc(float wantedSpeed, float acceleration)
{
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  unsigned long interval = 200;
  float speed = 0;


  while (speed < wantedSpeed)
  {
    unsigned long elapsedTime = currentMillis - previousMillis;
    if (elapsedTime >= interval)
    {
    speed += acceleration;
    
    previousMillis = currentMillis;
    }
  }

}

void moveForward(float targetDistance, float desiredSpeed) 
{
    Serial.println("Moving Forward!");

    uint8_t gauche = 0; // Moteur2
    uint8_t droite = 1; // Moteur1

    float kP = 1; // Proportional constant
    float kI = 0.4; // Integral constant

    // Reset encoders for both motors
    ENCODER_ReadReset(gauche);
    ENCODER_ReadReset(droite);

    acc(desiredSpeed, 0.2);

    int targetPulses = valeurEncodeur(targetDistance);
    int currentPulses = 0;
    float errorSum = 0.0;

    while (currentPulses < targetPulses) {
        currentPulses = ENCODER_Read(droite); // Read encoder value for the right wheel

        // Calculate error
        int error = targetPulses - currentPulses;
        errorSum += error;

        // Calculate PID output (without derivative)
        float pidOutput = (kP * error) + (kI * errorSum);

        // Limit the PID output based on the desired speed
        pidOutput = limitSpeed(pidOutput, desiredSpeed);

        // Set motor speed for the right wheel (Moteur1) using the PID output
        MOTOR_SetSpeed(droite, pidOutput);

        // Set motor speed for the left wheel (Moteur2) to follow the right wheel's speed
        MOTOR_SetSpeed(gauche, desiredSpeed);
    }

    // Stop both motors when the target distance is reached
    MOTOR_SetSpeed(droite, 0.0);
    MOTOR_SetSpeed(gauche, 0.0);

    // Reset the integral term for the next call
    errorSum = 0.0;
}



void stop()
{
  uint8_t gauche = 0; //Moteur2
  uint8_t droite = 1; //Moteur1
  float stop = 0;
  Serial.println("Stopping");
  MOTOR_SetSpeed(gauche, stop); 
  MOTOR_SetSpeed(droite, stop);
}
void goForward(float vitesse)
{
  integrale = 0;
  uint8_t gauche = 0; //Moteur2
  uint8_t droite = 1; //Moteur1 
  float vitesseGauche = vitesse; 
  float vitesseDroite = vitesse;
  int pulseVoulu = valeurEncodeur (0.5);
  int pulseActuelDroite = ENCODER_Read(droite);

  //met pulse a 0 avant de commencer a tourner
  ENCODER_Reset (gauche);
  ENCODER_Reset (droite);
  Serial.println("Moving forward !");
  MOTOR_SetSpeed(droite, vitesseDroite);
  MOTOR_SetSpeed(gauche, vitesseGauche);
  unsigned long previousMillis = 0;
  unsigned long pidInterval = 2000;
  unsigned long lastPidExecution = 0;


  //jusqu'a ce qu'il finit son deplacement fait execute PID

  while (pulseActuelDroite < pulseVoulu)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - lastPidExecution >= pidInterval)
      {    
          lastPidExecution = currentMillis;
          vitesseGauche = vitesseGauche + PID(0.5);
          Serial.print ("PID fait \n");
          MOTOR_SetSpeed(gauche, vitesseGauche);
          pulseActuelDroite = ENCODER_Read(droite);
      }    
    }
    
  stop ();

}

void turnRight()
{
  Serial.println("Turning right !");
}

void turnLeft()
{
  Serial.println("Turning left !");
}

void turnAround()
{
  Serial.println("Turning around !");
}

/* DEBUG
      void testEncodeur()
      {
        uint8_t gauche = 0; //Moteur2
        uint8_t droite = 1; //Moteur1
        int printTest = ENCODER_Read(gauche);
        Serial.print (printTest);

        Serial.print ("\n");
        ENCODER_Reset (gauche);

        printTest = ENCODER_Read(gauche);
        Serial.print (printTest);
        Serial.print ("\n");
        MOTOR_SetSpeed (gauche, 0.3);
        delay(1000);

        MOTOR_SetSpeed (gauche, 0);
        printTest = ENCODER_Read(gauche);
        Serial.print (printTest);
        Serial.print ("\n");
      }
*/

bool DetectWhistle()
{
  Serial.println("Whistle detected, GOGOGO !");
} 





/*
// Fonction permettant d'afficher la map dans la console, elle sert seulement pour tester et visualiser.
void PrintMap() {
  Serial.println("");

  for (int y = MAP_LENGTH - 1; y >= 0; y--) { // Start from the bottom row and go upwards
    // Print the top row of each tile
    for (int x = 0; x < MAP_WIDTH; x++) {
      if (TileMap[x][y].top) {
        Serial.print("+---+");
      } else {
        Serial.print("+   +");
      }
    }
    Serial.println(); // Move to the next row

    // Print the middle row of each tile (including walls on the left and right)
    for (int x = 0; x < MAP_WIDTH; x++) {
      if (TileMap[x][y].left) {
        Serial.print("|");
      } else {
        Serial.print(" ");
      }
      Serial.print("   ");
      if (TileMap[x][y].right) {
        Serial.print("|");
      } else {
        Serial.print(" ");
      }
    }
    Serial.println(); // Move to the next row

    // Print the bottom row of each tile
    for (int x = 0; x < MAP_WIDTH; x++) {
      if (TileMap[x][y].down) {
        Serial.print("+---+");
      } else {
        Serial.print("+   +");
      }
    }
    Serial.println(); // Move to the next row
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
*/
void setup() {
  BoardInit();
  //GenerateMap();
  //PrintMap();
  delay(500);
}



void loop()
{
  moveForward(0.3, 0.2);
  delay(1000);

}