#include <Arduino.h>
#include <librobus.h>

const int MAP_WIDTH = 3;
const int MAP_LENGTH = 10;

const int LEFT_IR_PIN = 24;
const int RIGHT_IR_PIN = 26;

int coordX = 1;
int coordY = 0;
int orientation = 2;

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

void TurnRight()
{
  MOTOR_SetSpeed(0, 0.2);
  MOTOR_SetSpeed(1, -0.2);
  delay(1000);
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
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
  Serial.println("Turning right !");
}

void TurnLeft()
{
  MOTOR_SetSpeed(0, -0.2);
  MOTOR_SetSpeed(1, 0.2);
  delay(1000);
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
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
  Serial.println("Turning left !");
}

void TurnAround()
{
  MOTOR_SetSpeed(0, 0.2);
  MOTOR_SetSpeed(1, -0.2);
  delay(2000);
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
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
  Serial.println("Turning around !");
}

void GoForward()
{
  MOTOR_SetSpeed(0, 0.2);
  MOTOR_SetSpeed(1, 0.2);
  delay(4600);
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
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
  Serial.println("Moving forward !");
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
  Serial.println("Whistle detected, GOGOGO !");
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

void setup() {
  BoardInit();
  GenerateMap();

  // Initialize the IR pins
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  // Hard code of one particular map
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

  delay(500);
}

void loop() {

  // Check if the robot has reached the end (coordY == 9)
  if (coordY == 9) {
    PrintRobotState();
    PrintMap();
    isMazeDone = true;
    delay(150000);
  }

  // If the maze is not yet complete
  if (!isMazeDone) {
    PrintRobotState();
    PrintMap();

    // Perform actions in a specific order:

    int newOrientation;

    // Detect if there's a wall or a tape in front of the robot and update the map
    while(DetectWall() == true || CheckForFrontWall() == 1) {
      // Find the new orientation after detecting walls
      newOrientation = FindTheWay();
      if (newOrientation != -1)
        Turn(newOrientation);
    }
    // If a valid path is found, adjust the orientation and move forward
    if (newOrientation != -1) {
      GoForward();
    }
  }

  // Delay for 1 second before the next iteration
  //delay(2000);
}
