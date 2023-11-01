#include <Arduino.h>
#include <LibRobus.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <math.h>
#include <limits.h>

// PINS CONSTANTS

const int RIGHT_RED_IR_PIN = 40;
const int RIGHT_GREEN_IR_PIN = 41;
const int LEFT_RED_IR_PIN = 42;
const int LEFT_GREEN_IR_PIN = 43;

uint16_t r, g, b, c;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ROBOT CONSTANTS

#define WHEEL_DIAMETER  3
#define PULSE_PER_ROTATION 3200
#define WHEEL_BASE_WIDTH 7.375
#define WHEELS_RADIUS 7.375/2

// distance from wall
#define GREEN_DISTANCE 220
#define YELLOW_DISTANCE 92
#define BLUE_DISTANCE 380

// WALL PID CONSTANTS AND VARIABLES
const int stackSize = 4;
int stack[stackSize] = {0, 0, 0, 0};
int top = 3;

const float KP = 0.1;
const float KI = 0.00;
const float KD = 0.0;
const float MAX_SPEED_IN_INCHES = 34.5;
const float MAX_SPEED = 0.5;
const float DISTANCE_PER_PULSE = M_PI*WHEEL_DIAMETER/PULSE_PER_ROTATION; // La distance en pouce fait avec un pulse
float integralLineFollowing = 0;
float lastErrorLineFollowing = 0;
uint16_t lastPv = -1;


// GLOBAL VARIABLES
int actualStep = 0;
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

PIDValues rightPIDValues = {6.65, 0.8, 0.0};
PIDValues leftPIDValues = {6.65, 0.8, 0.0};
PIDValues mainPIDValues;

struct Robot {
    float orientation;
    float leftMotorSpeed;
    float rightMotorSpeed;
    float xPosition;
    float yPosition;
};

Robot bot;

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

void turn360()
{
    uint8_t gauche = 0; // Moteur2
    uint8_t droite = 1; // Moteur1
    int rotation = 7740;
    bool turnCompleted = false;
    
    while (!turnCompleted) {
        // Read current encoder positions
        float pCurrentd = ENCODER_Read(droite);
        float pCurrentg = ENCODER_Read(gauche);
        // Apply corrections to motor speeds
        MOTOR_SetSpeed(droite, -0.3);
        MOTOR_SetSpeed(gauche, 0.3);
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

void turn60(uint8_t direction) {
  uint8_t gauche = 0; // Moteur2
  uint8_t droite = 1; // Moteur1
  int rotation = 1260;
  bool turnCompleted = false;

  Serial.println("Turning right !");
  
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
          
          Serial.println("Turn completed!");
      }
  }

  delay(10);
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

void followWall(float spPourcentage, float MaxSpeed) {  //@brief sPour : setpoint en %, MaxSpeed est la vitesse voulue


    /* Filtre numerique
    stack[3] = stack[2];
    stack[2] = stack[1];
    stack[1] = stack[0];
    stack[0] = ROBUS_ReadIR(3);

    int pv = (stack[3]+stack[2]+stack[1]+stack[0])/4;
    */

   //lecture de la distance 
    int pvBrute = ROBUS_ReadIR(3); //retourne lecture analogique donc 0 a 1023
    //convertie en %
    int pvPourcentage = ((100/1023) * pvBrute) ;

    //calcul l'erreur
    float error = spPourcentage - pvPourcentage;
 
    //integral
    if (abs(error) < 50)
        integralLineFollowing += 0 + error;
    else if (abs(error) > 50)
        integralLineFollowing = 0;
   integralLineFollowing = constrain(integralLineFollowing,0, 10000);
    
    //calcul de l'ajustement
   float OUTLineFollowing = KP*error + KI*integralLineFollowing + KD*(error - lastErrorLineFollowing);

    //sauvegarde pour la derivee
    lastErrorLineFollowing = error;

    //convertion en po/s pour la boucle low level 34.5 po/s / 100%   0.345 po/s/%
    float OutLineFollowingspeedInches = (OUTLineFollowing * 0.345);
    OutLineFollowingspeedInches = constrain(OutLineFollowingspeedInches, 0, 34.5);

    //correction low-level
    leftPID(constrain(MaxSpeed + OutLineFollowingspeedInches, 0, 34.5));
    rightPID(constrain(MaxSpeed - OutLineFollowingspeedInches, 0, 34.5));
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
    turn360();
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

void move(float radius, float cruisingSpeed, float distance,float finishAngle, int direction) {
    float leftSpeedSetpoint, rightSpeedSetpoint;

    if (radius == 0) {
        // Going straight
        leftSpeedSetpoint = rightSpeedSetpoint = cruisingSpeed;
        if(bot.xPosition > distance) {
            actualStep++;
        }
    } else {
        if(direction == 1 && bot.orientation <= finishAngle) {
            actualStep++;
        } else if(direction == 0 && bot.orientation <= finishAngle) {
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
    initializeServos();
    pinMode(RIGHT_GREEN_IR_PIN, INPUT);
    pinMode(RIGHT_RED_IR_PIN, INPUT);
    pinMode(LEFT_GREEN_IR_PIN, INPUT);
    pinMode(LEFT_RED_IR_PIN, INPUT);
    /*
    if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
    }
    */
}

void loop() {

followWall(30, 15);

/*
float distance = ROBUS_ReadIR(3);
Serial.println(distance);
delay(200);
*/

/*
    switch (actualStep)
    {   
        case 0:
            actualColor = findColor();
            startDistance = ROBUS_ReadIR(3);
            bot.orientation = 0;
            actualStep++;
            break;
        case 1:
            move(29, 12, 0, -M_PI, 1);
            Serial.println(bot.orientation);
            break;
        case 2:
            followWall(GREEN_DISTANCE, 0.29);
            break;
        default:
            break;
    }
*/
}
