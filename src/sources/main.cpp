#include <Arduino.h>
#include <librobus.h>

const int MAP_WIDTH = 3;
const int MAP_LENGTH = 10;

int test = 0;

struct tile {
  bool top;
  bool down;
  bool left;
  bool right;
};

tile TileMap[MAP_WIDTH][MAP_LENGTH];

void TurnRight()
{
  Serial.println("Turning right !");
}

void TurnLeft()
{
  Serial.println("Turning left !");
}

void TurnAround()
{
  Serial.println("Turning around !");
}

void GoForward()
{
  Serial.println("Moving forward one tile !");
}

bool DetectWhistle()
{
  Serial.println("Whistle detected, GOGOGO !");
}

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

void setup() {
  BoardInit();
  GenerateMap();
  TileMap[1][7].down = true;
  TileMap[1][6].top = true;
  PrintMap();
  delay(500);
}

void loop() {
  //PrintTile(TileMap[0][5]);
  delay(500);
}

