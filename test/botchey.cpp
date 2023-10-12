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
#define KP_RIGHT 0.02
#define KP_LEFT 0.006
#define KI 0.00
#define KD 0.1

// Définition des autres constantes
#define WHEEL_DIAMETER 7.62 // 3 pouces convertis en cm
#define PULSES_PER_REVOLUTION 3200
#define MAX_SPEED 0.3
#define MIN_SPEED 0.1

float error_previous = 0;
float integral = 0;

bool isRunning = false;
bool isMazeDone = false;

// Une tile se compose de 4 murs, représentés par 0, 1 ou 2.
struct tile {
  int top;
  int down;
  int left;
  int right;
};

// Déclaration d'une matrice 2 dimensions contenant une tile pour chaque coordonnées
tile TileMap[MAP_WIDTH][MAP_LENGTH];

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
    }
  }
}

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
    int rotation = 3855;
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
  int rotation = 1840;
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

    ENCODER_Reset(0);
    ENCODER_Reset(1);

    int compteur = 0;
    integral = 0;

    while(ENCODER_Read(0) < target_pulses && ENCODER_Read(1) < target_pulses) {
        compteur++;
        
        float error_left = target_pulses - ENCODER_Read(0);
        float error_right = target_pulses - ENCODER_Read(1);
        integral += (error_left + error_right) / 2;

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

  /*
  if(ROBUS_IsBumper(0) == true && ROBUS_IsBumper(1) == true)
    Reset();
  */

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
    }
    // If a valid path is found, adjust the orientation and move forward
    if (newOrientation != -1) {
      GoForward(50);
    }
  }
}