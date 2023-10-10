#include <Arduino.h>
#include <libRobus.h>
 
const int kHzWhistlePin=A0; 
const int BruitAmbiantPin=A1;

<<<<<<< Updated upstream
const int MAP_WIDTH = 3;
const int MAP_LENGTH = 10;

int test = 0;

// Une tile se compose de 4 murs, ce sont des valeurs vrai ou faux.
struct tile {
  bool top;
  bool down;
  bool left;
  bool right;
};

// Déclaration d'une matrice 2 dimensions contenant une tile pour chaque coordonnées
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
  Serial.println("Moving forward !");
}

bool DetectWhistle()
{
  Serial.println("Whistle detected, GOGOGO !");
}

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

void setup() {
  BoardInit();
  GenerateMap();
  PrintMap();
  delay(500);
}

void loop() {
  delay(500);
}

=======
 
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


void setup()
{
  //Initialise comms pour debuggage
  Serial.begin(9600); //Devrait-on mettre un temps plus long pour assurer un bon setup?
 
  //Instaurer les pin du arduino qui servent d'INPUT
  pinMode(kHzWhistlePin, INPUT);
  pinMode(BruitAmbiantPin, INPUT);
}
 
void loop ()
{
  bool x = DetectWhistle();
  Serial.println(x);

}




/*
void loop()
{
    //lire valeur du microphone
    float MicrophoneValue=analogRead(kHzWhistlePin);
 
    //lire valeur du bruit ambiant
    float BruitAmbiantValue=analogRead(BruitAmbiantPin);
 
    float difference = (MicrophoneValue-BruitAmbiantValue); 
 
    if (difference > 50)  
    {
      
      Serial.println("GOGOGOG");
    }
  
    Serial.println("valeur 5kHz");
    Serial.println(MicrophoneValue);
    Serial.println("Valeur Bruit ambiant");
    Serial.println(BruitAmbiantValue);
    Serial.println("Difference"); 
    Serial.println(difference);
  
 delay (500);
} */




/*
#include <Arduino.h>
#include <libRobus.h>
 
const int kHzWhistlePin=A0; 
const int BruitAmbiantPin=A1;
const int threshold = 50; //La valeur ou on veut dire que le 5khz est detecte
const int numReadings = 100; //numero de lectures
bool isRunning = true;

int readings1[numReadings];
int readings2[numReadings];
int readIndex = 0;
int total1 = 0;
int total2 = 0;
int average1 = 0;
int average2 = 0;
 
void setup()
{
  //Initialise comms pour debuggage
  Serial.begin(9600); //Devrait-on mettre un temps plus long pour assurer un bon setup?
 
  //Instaurer les pin du arduino qui servent d'INPUT
  pinMode(kHzWhistlePin, INPUT);
  pinMode(BruitAmbiantPin, INPUT);

  //initialiser la matrice a 0
  for (int i=0; i<numReadings; i++){
    readings1[i] = 0;
    readings2[i] = 0;
  }
}
 
void loop()
{
  //Enleve derniere lecture
  total1 = total1 - readings1[readIndex];
  total2 = total2 - readings2[readIndex];

  //Lecture de nouvelle valeurs
  readings1[readIndex] = analogRead(kHzWhistlePin);
  readings2[readIndex] = analogRead(BruitAmbiantPin);
  total1 = total1 + readings1[readIndex];
  total2 = total2 + readings2[readIndex];

  Serial.println("total1 is:");
  Serial.println(total1);
  Serial.println("total2 is:");
  Serial.println(total2);

  //Avance a nouvelle position de lecture
  readIndex = readIndex + 1;
  if(readIndex >= numReadings){
    readIndex = 0;
  }

  //calcul des moyenne
  average1 = total1 / numReadings;
  average2 = total2 / numReadings;

  Serial.println("average1 is:");
  Serial.println(average1);
  Serial.println("average2 is:");
  Serial.println(average2);
  Serial.println("NEW TEST \n");

  //check for the 5khz signal based on the averages
  if (average1 > threshold || average2 > threshold){
    Serial.println("5khz frequency");
    delay (500);
  }


delay(1500); 
 
}*/
>>>>>>> Stashed changes
