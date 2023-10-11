#include <Arduino.h>
#include <librobus.h>

/*
//Variables globals
const int MAP_WIDTH = 3;
const int MAP_LENGTH = 10;
int test = 0;
*/
float integrale = 0;

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
// trouve le nombre de pulse pour une distance donner
{
  float pas = 0.00007552376; // m
  int pulse = 0;

  pulse = (distance / pas);

  return pulse;
}

float PID(float distance)
{
  uint8_t gauche = 0; // Moteur2
  uint8_t droite = 1; // Moteur1
  float correction = 0;
  float kP = 0.4;
  int32_t pulseActuelGauche = ENCODER_Read(gauche);
  int32_t pulseActuelDroite = ENCODER_Read(droite);
  int pulseFinal = valeurEncodeur(distance);

  double erreur = (pulseActuelDroite / pulseFinal - pulseActuelGauche / pulseFinal);
  correction = correction + (erreur * kP);
  if (correction > 1)
  {
    correction = 1;
  }
  if (correction < 0)
  {
    correction = 0;
  }

  return correction;
}

float limitSpeed(float speed)
{
  if (speed > 1)
  {
    return 1;
  }

  else if (speed < -1)
  {
    return -1;
  }
  else
  {
    return speed;
  }
}

void stop()
{
  uint8_t gauche = 0; // Moteur2
  uint8_t droite = 1; // Moteur1
  float stop = 0;
  Serial.println("Stopping");
  MOTOR_SetSpeed(gauche, stop);
  MOTOR_SetSpeed(droite, stop);
}

void moveForward(float targetDistance, float desiredSpeed)
{
  Serial.println("Moving Forward!");

  uint8_t gauche = 0; // Moteur2
  uint8_t droite = 1; // Moteur1

  float speed = 0;
  float step = (desiredSpeed / 100);

  float kP = 0.1; // Gain
  float kI = 0.0; //integrale
  float accumulatedError = 0.0;

  // Reset encoders for both motors
  ENCODER_ReadReset(gauche);
  ENCODER_ReadReset(droite);

  int targetPulses = valeurEncodeur(targetDistance);
  int32_t currentPulses = 0;
  int32_t pulseActuelGauche = 0;
  int32_t pulseActuelDroite = 0;

  Serial.println(targetPulses);

  float error = 0;
  float pidOutput = 0;

  while (currentPulses < (targetPulses))
  {

    currentPulses = ENCODER_Read(droite); // lit enc roue droite
    pulseActuelGauche = ENCODER_Read(gauche);
    pulseActuelDroite = ENCODER_Read(droite);
    /*
    Serial.println ("current: ");
    Serial.println (currentPulses);
    Serial.println ("Target: ");
    Serial.println (targetPulses);
    */

    Serial.println("gauche: ");
    Serial.println(pulseActuelGauche);
    Serial.println("droite: ");
    Serial.println(pulseActuelDroite);
    Serial.println(" -- ");

    // Calculate error
    error = (pulseActuelDroite/targetPulses - pulseActuelGauche/targetPulses);
    pidOutput = speed + (kP * error) + (kI * accumulatedError);
    accumulatedError += error;

    // limite la vitesse
    pidOutput = limitSpeed(pidOutput);

    MOTOR_SetSpeed(droite, speed);
    MOTOR_SetSpeed(gauche, pidOutput);

   
    // increment speed
    if (currentPulses < (targetPulses * (0.25)))
    {

      if (speed < desiredSpeed)
      {
        speed = speed + step;
      }

      else if (speed > desiredSpeed)
      {
        speed = desiredSpeed;
      }
    }
    else if ((currentPulses >= (targetPulses * (0.25))) && ((currentPulses <= (targetPulses * (0.75)))))
    {
      speed = desiredSpeed;
    }

    else if (currentPulses > (targetPulses * 0.75))
    {

      if (speed > 0.15)
      {
        speed = speed - step;
      }
      else
      {
        speed = 0.15;
      }
    delay(600);
    }
  }

  //reset laccumulation derreur pour pas que ca affecte prochain mouvement 
  accumulatedError = 0;
  // Stop when the target distance is reached
   stop();
}
/*
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
*/
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

bool DetectWhistle()
{
  Serial.println("Whistle detected, GOGOGO !");
  return 0;
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
void setup()
{
  BoardInit();
  // GenerateMap();
  // PrintMap();
  delay(500);
}

void loop()
{
  moveForward(3, 0.6);
  delay(1000);
}