#define USERBRK 0
#define USERFOR 1
#define USERREV 2
#define USERLEF 3
#define USERRIG 4
#define USERINV 5
#define USERRTU 6

#define SIZEX 16
#define SIZEY 16
//mouse characteristics
/*
 * Up = 1
 * Right = 2
 * Down = 3
 * Left = 4
 *
 */
char mouseOrient = 1;
bool wallRight, wallBack, wallLeft, wallFront = false;
bool wallsOnBothSides = false;

int posX = 0;
int posY = 0;

bool settingVars = true;//used in setup when awaiting user start
bool isTurning = false;
//maze specs
int sizeX = SIZEX;//recasting
int sizeY = SIZEY;//recasting

int goalx = 7;
int goaly = 7;

int mazeDist[SIZEX][SIZEY];
bool  mazeWalls[2 * SIZEX][SIZEY];//X indices 0 to SIZEX - 1 are horizontal, SIZEX - (2 * SIZEX) - 1 are vertical. 

int checkQueue[SIZEX * SIZEY];
int checkTempValue;
int checkSize = 0;

bool goalFound = false;

int readingWallLeft = 150;//lower threshold to set wallLeft flag
int readingWallRight = 150;//lower threshold to set wallRight flag
int readingWallFront = 350;//lower threshold to set wallFront flag
//pins
int irRecievePinL = A5;
int irRecievePinFL = A4;
int irRecievePinFR = A3;
int irRecievePinR = A2;

int irEmitPinL = A9;
int irEmitPinFL = A8;
int irEmitPinFR = A7;
int irEmitPinR = A6;

int forwardPinR = 6;
int reversePinR = 5;
int forwardPinL = 4;
int reversePinL = 3;

int aPinL = 7;
int bPinL = 8;
int aPinR = 9;
int bPinR = 10;

int ledPin = 13;
//led blinker bool
bool isLedOn = false;

//calculations

int speedMax = 250;//max speed for each motor //!not in use
int speedMaxLeft = 160;
int speedMaxRight = 135;
int speedLeft = 157;//initial speed of left motor
int speedRight = 135;//inital speed of right motor
int recoverSpeedL;//speed used in recovery//!not in use
int recoverSpeedR;//speed used in recovery//!not in use
//mapped values
int mappedL = 1000;
int mappedR = 950;
//const int speedREV = 0;
//const int speedFOR = 255;
const int speedNEU = 128;//speed to halt motor (questionable)

int userCommand = USERBRK;
int sensorReadL, sensorReadFL, sensorReadFR, sensorReadR;

const int sensorReadCorrectionBoundL = 30;
const int sensorReadCorrectionBoundR = 30;

int displacementReadings;

bool switchMove = false;
bool actionFinished = true;
bool actionLeft = false;//true if left has finshed action, false otherwise
bool actionRight = false;//true if right has finshed action, false otherwise

bool recoveryMode = false;


int interL, interFL, interFR, interR;

const double kP = 0.3;
const double kI = 0.05;
const double kD = 0.005;
double oldError = 0.0;//used in D control
double sumOfErrors = 0.0;//used in I control
volatile long errorCount = 0;//used in I control

const int correctionTotal = 500;//used to flag absense of walls
int stickToWallValue;//used in 1-wall control
//L = 0 R = 1
bool stickToWallLorR = 0;
bool stickToWall = false;

//interupts
volatile long countLRA = 0;
volatile long countRRA = 0;

volatile long countLRASaved = 0;
volatile long countRRASaved = 0;

volatile long countTempTicks;
/*estimated measurements
 * 1 cell step = 500 ticks
 * 1 90 degree turn = 175 ticks (ps includes both sides)
 * 
 */
long currentLRABound = 530;
long currentRRABound = 530;
//175
long currentTurnBound = 180;
long currentFullBound = 410;
//timer values
unsigned long blinkerMillis = 0;
unsigned long irMillis = 0;
unsigned long infoMillis = 0;
unsigned long actionMillis = 0;
unsigned long correctionMillis = 0;
unsigned long currentMillis;

const unsigned long blinkerDelay = 1000;

const unsigned long irDelay = 1;
bool areIREmittersOn = true;

const unsigned long infoDelay = 100;
const unsigned long correctionDelay = 10;
const unsigned long actionDelay = 2000;
const unsigned long breakDelay = 1000;
//function declarations
  //mouse movements
void moveBreak(int pinFor, int pinRev);

void turnLeft(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void turnRight(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void moveForward(int pinFor, int pinRev, int motSpeed);

void rightMotor(int pinF,int pinR, int sp1,int sp2);
void leftMotor(int pinF,int pinR, int sp1,int sp2);

void moveWheelsFor(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

int randomInstruction();

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR);
  //ir functions

void leftEncoderEvent();
void rightEncoderEvent();

int newCommand();//global variable char instructionMethod controls which function generates instructions
int proceduralCommand();//set instructionMethod to 'p'
int randomCommand();//set instructionMethod to 'r'
int solverCommand();//set instructionMethod to 's'
const char instructionMethod = 'r';

//debug values
char keyboardInput = '0';

void setup() {
  Serial.begin(9600);
  //define pins
  //led pin
  pinMode(ledPin, OUTPUT);
  //encoder pins
  pinMode(aPinL, INPUT);
  pinMode(bPinL, INPUT);
  pinMode(aPinR, INPUT);
  pinMode(bPinR, INPUT);
  
  //motor control pins
  pinMode(forwardPinL, OUTPUT);
  pinMode(reversePinR, OUTPUT);
  pinMode(forwardPinL, OUTPUT);
  pinMode(reversePinR, OUTPUT);

  //ir receiver pins
  pinMode(irRecievePinL, INPUT);
  pinMode(irRecievePinFL, INPUT);
  pinMode(irRecievePinFR, INPUT);
  pinMode(irRecievePinR, INPUT);
  
  //ir emitter pins
  pinMode(irEmitPinL, OUTPUT);
  pinMode(irEmitPinFL, OUTPUT);
  pinMode(irEmitPinFR, OUTPUT);
  pinMode(irEmitPinR, OUTPUT);

  //find interference
  analogWrite(irEmitPinL, 0);
  analogWrite(irEmitPinFL, 0);
  analogWrite(irEmitPinFR, 0);
  analogWrite(irEmitPinR, 0);
  delay(10);
  interL = analogRead(irRecievePinL);
  interFL =  analogRead(irRecievePinFL);
  interFR =  analogRead(irRecievePinFR);
  interR =  analogRead(irRecievePinR);

  
  
  //setup maze vars
  for(int i = 0; i < sizeX/2; i++) {
	  for(int j = 0; j < sizeY/2; j++) {
      int val = sizeX - i - j - 2;
      mazeDist[i][j] = val;
      mazeDist[sizeX - i - 1][sizeY - j - 1] = val;
      mazeDist[i][sizeY - j - 1] = val;
      mazeDist[sizeX - i - 1][j] = val;
	  }
  }
  checkQueue[0] = mazeDist[0][0];
  checkSize++;
  //debug keyboard
  
  //set bounds
  //ready to go
  analogWrite(irEmitPinFR, 255);
  sensorReadFR = analogRead(irRecievePinFR) - interFR;
  digitalWrite(ledPin, HIGH);
  while(settingVars){
    analogWrite(irEmitPinFR, 255);
    analogWrite(irEmitPinL, 255);
    analogWrite(irEmitPinR, 255);
    sensorReadFR = analogRead(irRecievePinFR);
    //mappedL = analogRead(irRecievePinL);
    //mappedR = analogRead(irRecievePinR);
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    if(sensorReadFR >= 500) {
      settingVars = false;
    }
  }

  //attachInterupts
  attachInterrupt(digitalPinToInterrupt(aPinL),leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(aPinR),rightEncoderEvent, CHANGE);

  //set all walls to false:
for(int i = 0; i < (2 * sizeX); i++) {
  for(int j = 0; j < sizeY; j++) {
    mazeWalls[i][j] = false;
    }
  }
  
}

void loop() {
  //blinks led every 2 loops
  currentMillis = millis();
  if(currentMillis - blinkerMillis >= blinkerDelay) {
    if(!isLedOn) {
      digitalWrite(ledPin, HIGH);
      isLedOn = true;
    }
    else{
      digitalWrite(ledPin, LOW);
      isLedOn = false;
    }
    blinkerMillis = currentMillis;
  }
  
  if(Serial.available() > 0) {
    keyboardInput = Serial.read();
    if(keyboardInput == 'w') {
      userCommand = USERFOR;
    }
    else if(keyboardInput == 's') {
      userCommand = USERREV;
    }
    else if(keyboardInput == 'b') {
      userCommand = USERBRK;
    }
    else if(keyboardInput == 'a') {
      userCommand = USERLEF;
    }
    else if(keyboardInput == 'x') {
      userCommand = USERINV;
    }
    else if(keyboardInput == 'd') {
      userCommand = USERRIG;
    }
    countLRASaved = countLRA;
  }
  
  /*if(locationX != goalx && locationY != goaly) {
    checkQueue[0] = mazeDist[locationX][locationY];
    checkSize = 1;
    while(checkSize != 0) {
      checkTempValue = checkQueue[checkSize - 1];
      checkSize--;
      if(checkTempValue != 
    }
    //
  }
  else if(locationX == goalx && locationY == goaly) {
    
  }*/
  //finds interference and reads
  if(currentMillis - irMillis >= irDelay) {
    if(areIREmittersOn) {
      
      sensorReadL = analogRead(irRecievePinL) - interL;
      sensorReadFL = analogRead(irRecievePinFL) - interFL;
      sensorReadFR = analogRead(irRecievePinFR) - interFR;
      sensorReadR = analogRead(irRecievePinR) - interR;

      sensorReadL = map(sensorReadL, 0, mappedL, 0, 500);
      sensorReadFL = map(sensorReadFL, 0, 1000, 0, 500);
      sensorReadFR = map(sensorReadFR, 0, 1000, 0, 500);
      sensorReadR = map(sensorReadR, 0, mappedR, 0, 500);

      if(sensorReadFL >= readingWallFront || sensorReadFR >= readingWallFront) {
        wallFront = true;
      }
      else {
        wallFront = false;
      }
      if(sensorReadL >= readingWallLeft) {
        wallLeft = true;
      }
      else {
        wallLeft = false;
      }
      if(sensorReadR >= readingWallRight) {
        wallRight = true;
      }
      else {
        wallRight = false;
      }
      
      analogWrite(irEmitPinL, 0);
      analogWrite(irEmitPinFL, 0);
      analogWrite(irEmitPinFR, 0);
      analogWrite(irEmitPinR, 0);
      
      areIREmittersOn = false;
    }
    else {
      interL = analogRead(irRecievePinL);
      interFL =  analogRead(irRecievePinFL);
      interFR =  analogRead(irRecievePinFR);
      interR =  analogRead(irRecievePinR);
      
      analogWrite(irEmitPinL, 255);
      analogWrite(irEmitPinFL, 255);
      analogWrite(irEmitPinFR, 255);
      analogWrite(irEmitPinR, 255);

      areIREmittersOn = true;
    }
    irMillis = currentMillis;
  }
  
  //breaks after a cell or action is completed
  if(recoveryMode) {
    if(userCommand == USERBRK && currentMillis - actionMillis >= actionDelay) {
      userCommand = USERREV;
      switchMove = false;
      actionFinished = false;
      countLRASaved = countLRA;
      actionMillis = currentMillis;
      recoverSpeedL = speedLeft;
      recoverSpeedR = speedRight;
      speedLeft = speedMaxLeft;
      speedRight = (speedMaxRight * 3 ) /4;
    }
    else if(userCommand == USERREV) {
      if(countLRASaved - countLRA >= currentLRABound || countLRA - countLRASaved >= currentLRABound) {
        actionFinished = true;
        recoveryMode = false;
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
      }
    }
  }
  if(!actionFinished) {
    if(userCommand == USERRIG) {
      if(countLRASaved - countLRA  >= currentTurnBound || countLRA - countLRASaved >= currentTurnBound) {
        actionRight = true;
        leftMotor(forwardPinL, reversePinL, 0 ,0);
      }
      else{
        leftMotor(forwardPinL, reversePinL, speedLeft,0);
      }
      if(countRRASaved - countRRA  >= currentTurnBound || countRRA - countRRASaved >= currentTurnBound) {
        actionRight = true;
        rightMotor(forwardPinR, reversePinR, 0, 0);
      }
      else {
        rightMotor(forwardPinR, reversePinR, 0, speedRight);
      }
      if(actionRight && actionLeft) {
        actionFinished = true;
        isTurning = false;
      }
    }
    else if(userCommand == USERLEF) {
      if(countLRASaved - countLRA  >= currentTurnBound || countLRA - countLRASaved >= currentTurnBound) {
        actionRight = true;
        leftMotor(forwardPinL, reversePinL, 0 ,0);
      }
      else {
        leftMotor(forwardPinL, reversePinL, 0,speedLeft);
      }
      if(countRRASaved - countRRA  >= currentTurnBound || countRRA - countRRASaved >= currentTurnBound) {
        actionRight = true;
        rightMotor(forwardPinR, reversePinR, 0 ,0);
      }
      else {
        rightMotor(forwardPinR, reversePinR, speedRight,0);
      }
      if(actionRight && actionLeft) {
        actionFinished = true;
        isTurning = false;
      }
    }
    else if(userCommand == USERINV) {
      if((abs(countLRASaved - countLRA)>= currentFullBound)) {
        actionFinished = true;
        isTurning = false;
      }
    }
    else if(userCommand == USERFOR) {
      if(countLRASaved - countLRA >= currentLRABound || countLRA - countLRASaved >= currentLRABound) {
        actionFinished = true;
      }
      else if(wallFront) {
        actionFinished = true;
      }
      if(currentMillis - correctionMillis > correctionDelay && areIREmittersOn) {
        if(sensorReadL + sensorReadR >= correctionTotal) {
          if((sensorReadL > sensorReadR) && wallsOnBothSides && userCommand == USERFOR && !(sensorReadR <= 100 && sensorReadL >= 400)) {
            displacementReadings = sensorReadL - sensorReadR;
            sumOfErrors += displacementReadings;
            if(displacementReadings >= 0) {
              speedLeft = speedMaxLeft;
              speedRight = speedMaxRight - (displacementReadings * kP) - (abs(oldError - displacementReadings) * kD);
              //Serial.println("displacementReadings: left");
              //Serial.println(displacementReadings);
            }
            oldError = displacementReadings;
          }
          else if ((sensorReadL < sensorReadR) && wallsOnBothSides && userCommand == USERFOR && !(sensorReadL <= 100 && sensorReadR >= 400)){
            displacementReadings = sensorReadR - sensorReadL;
            sumOfErrors += displacementReadings;
            if(displacementReadings >= 0) {
              speedLeft = speedMaxLeft - (displacementReadings * kP) - (abs(oldError - displacementReadings) * kD);
              speedRight = speedMaxRight;
              //Serial.println("displacementReadings: right");
              //Serial.println(displacementReadings);
            }
            oldError = displacementReadings;
          }
          stickToWall = false;
        }
        else if(!stickToWall){
          if((sensorReadL < sensorReadR) && wallsOnBothSides && userCommand == USERFOR) {
            stickToWallValue = sensorReadR;
            stickToWallLorR = 1;
            stickToWall = true;
          }          
          else if((sensorReadL > sensorReadR) && wallsOnBothSides && userCommand == USERFOR) {
            stickToWallValue = sensorReadL;
            stickToWallLorR = 0;
            stickToWall = true;
          }
        }
        else if(stickToWall) {
          if((sensorReadL > stickToWallValue) && userCommand == USERFOR && stickToWallLorR == 0) {
            displacementReadings = sensorReadL - stickToWallValue;
            if(displacementReadings >= 0) {
              speedLeft = speedMaxLeft;
              speedRight = speedMaxRight - (displacementReadings * kP) ;
              //Serial.println("displacementReadings: left");
              //Serial.println(displacementReadings);
            }
          }
          else if ((sensorReadL < stickToWallValue) && userCommand == USERFOR && stickToWallLorR == 0){
            displacementReadings = stickToWallValue - sensorReadL;
            if(displacementReadings >= 0) {
              speedLeft = speedMaxLeft - (displacementReadings * kP);
              speedRight = speedMaxRight;
              //Serial.println("displacementReadings: right");
              //Serial.println(displacementReadings);
            }
          }
          else if((sensorReadR > stickToWallValue) && userCommand == USERFOR && stickToWallLorR == 1) {
            displacementReadings = sensorReadR - stickToWallValue;
            if(displacementReadings >= 0) {
              speedLeft = speedMaxLeft - (displacementReadings * kP) ;
              speedRight = speedMaxRight;
              //Serial.println("displacementReadings: left");
              //Serial.println(displacementReadings);
            }
          }
          else if ((sensorReadR < stickToWallValue) && userCommand == USERFOR && stickToWallLorR == 1){
            displacementReadings = stickToWallValue - sensorReadR;
            if(displacementReadings >= 0) {
              speedLeft = speedMaxLeft;
              speedRight = speedMaxRight - (displacementReadings * kP);
              //Serial.println("displacementReadings: right");
              //Serial.println(displacementReadings);
            }
          }
          correctionMillis = currentMillis;
        }
      }
    }
    else if(userCommand == USERREV) {
      if(countLRASaved - countLRA >= currentLRABound || countLRA - countLRASaved >= currentLRABound) {
        actionFinished = true;
      }
    }
    else if(userCommand == USERBRK) {
      if(currentMillis - actionMillis >= breakDelay) {
        actionFinished = true;
      }
      if(wallLeft && wallRight) {
        wallsOnBothSides = true;
      }
      else {
        wallsOnBothSides = false;
      }
    }
    if (currentMillis - actionMillis >= actionDelay) {
      if(userCommand != USERBRK) {
        recoveryMode = true;
        countLRASaved = countLRA;
        actionMillis = currentMillis;
        userCommand = USERBRK;
        actionFinished = true;
      }
    }
  }//action finshed, generate new instruction:
  else {
    userCommand = newCommand();

    switchMove = (userCommand == USERBRK);
    actionFinished = false;

    //begin turn if wallFront
    if(wallFront) {
      actionLeft = false;
      actionRight = false;
      isTurning = true;
    }

    //if no walls, max speed
    if(!wallRight || !wallLeft) {
      speedLeft = speedMaxLeft;
      speedRight = speedMaxRight;
    }

    if(userCommand != USERBRK) {
      countLRASaved = countLRA;
      countRRASaved = countRRA;
    }

    actionMillis = currentMillis;
  }
  
  //userCommand = USERBRK;
  
  if(userCommand != USERLEF && userCommand != USERRIG) {
    moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  }

  if(currentMillis - infoMillis >= infoDelay) {
    Serial.print("LeftSpeed: ");
    Serial.println(speedLeft);
    Serial.print("RightSpeed: ");
    Serial.println(speedRight);
    Serial.print("I am :");
    if(userCommand == USERFOR) {
      Serial.println("going forward.");
    }
    else if(recoveryMode) {
      Serial.println("in recoveryMode.");
    }
    else if(userCommand == USERINV) {
      Serial.println("turning around.");
    }
    else if(userCommand == USERRIG) {
      Serial.println("turning right.");
    }
    else if(userCommand == USERLEF) {
      Serial.println("turning left.");
    }
    else if(userCommand == USERBRK) {
      Serial.println("break.");
    }
    else {
      Serial.println("misc");
    }
    Serial.print("Walls: ");
    if(wallRight){
      Serial.print(" right ");
    }
    if(wallLeft){
      Serial.print(" left ");
    }
    if(wallFront){
      Serial.print(" front ");
    }
    if(wallsOnBothSides) {
      Serial.print(" both ");
    }
    Serial.println(".");
    Serial.print("TicksL: ");
    Serial.println(countLRA);
    Serial.print("TicksR: ");
    Serial.println(countRRA);
    Serial.print("Right: ");
    Serial.println(sensorReadR);
    Serial.print("Interference: ");
    Serial.println(interR);
    Serial.print("RightTop: ");
    Serial.println(sensorReadFR);
    Serial.print("Interference: ");
    Serial.println(interFR);
    Serial.print("LeftTop: ");
    Serial.println(sensorReadFL);
    Serial.print("Interference: ");
    Serial.println(interFL);
    Serial.print("Left: ");
    Serial.println(sensorReadL);
    Serial.print("Interference: ");
    Serial.println(interL);
    infoMillis = currentMillis;
  }
}

//function definitions
void moveForward(int pinFor, int pinRev, int motSpeed) {
    analogWrite(pinFor, motSpeed);
    digitalWrite(pinRev, LOW);
}

void moveBackwards(int pinFor, int pinRev, int motSpeed) {
    digitalWrite(pinFor, LOW);
    analogWrite(pinRev, motSpeed);
}

void moveBreak(int pinFor, int pinRev) {
    analogWrite(pinFor, speedNEU);
    analogWrite(pinRev, speedNEU);
}


void turnLeft(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
  analogWrite(pinForL, 0);
  analogWrite(pinRevL, spL);
  analogWrite(pinForR, 0);
  analogWrite(pinRevR, spR);
}

void turnRight(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
  analogWrite(pinForL, spL);
  analogWrite(pinRevL, 0);
  analogWrite(pinForR, spR);
  analogWrite(pinRevR, 0);
}

void turnHalfCircle(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
  analogWrite(pinForL, 0);
  analogWrite(pinRevL, spL);
  analogWrite(pinForR, 0);
  analogWrite(pinRevR, spR);
}

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR) {
  switch(userCommand) {
    case USERBRK:
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    break;
    
    case USERFOR:
    moveWheelsFor(speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
    break;
    
    case USERREV:
    moveWheelsRev(speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
    break;
    
    case USERLEF:
    turnLeft(speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
    break;
    
    case USERRIG:
    turnRight(speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
    break;

    case USERINV:
    turnHalfCircle(speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
    break;
    
    default:
	  moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    break;
  }
}

void moveWheelsFor(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
  analogWrite(pinForL, spL);
  analogWrite(pinRevL, 0);
  analogWrite(pinForR, 0);
  analogWrite(pinRevR, spR);
}

void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
  analogWrite(pinForL, 0);
  analogWrite(pinRevL, spL);
  analogWrite(pinForR, spR);
  analogWrite(pinRevR, 0);
}

void leftMotor(int pinF,int pinR, int sp1,int sp2) {
  analogWrite(pinF, sp1);
  analogWrite(pinR, sp2);
}

void rightMotor(int pinF,int pinR, int sp1,int sp2) {
  analogWrite(pinF, sp2);
  analogWrite(pinR, sp1);
}

int newCommand() {
  switch(instructionMethod){
    case 'p':
      return proceduralCommand();
    break;
    case 'r':
     return randomCommand();
    break;
    case 's':
      return solverCommand();
    break;
    default:
      return USERBRK;
  }
}

int proceduralCommand() {
  if(switchMove) {
    if(wallFront) {
      if(!wallLeft) {
        return USERLEF;
      }
      else if(!wallRight) {
        return USERRIG;
      }
      else {
        return USERINV;
      }
    }
    else {
      return USERFOR;
    }
  }
  else {
    return USERBRK;
  }
}

int randomCommand() {
  int options = -1;
  if(!wallLeft)
    options++;
  if(!wallFront)
    options++;
  if(!wallRight)
    options++;

  int decision = (int)(random(0, 6)) % options;//6 is used because it has factors 1,2,&3

  if(!wallLeft && decision == 0)
    return USERLEF;
    
  decision--;

  if(!wallFront && decision == 0)
    return USERFOR;
    
  decision--;

  if(!wallRight && decision == 0)
    return USERRIG;
  
  //if all fail, turn around:
  return USERINV;
}



void mazeSolving() {
	/*distmaze := int[16][16]
wallmaze := int[16][16]
goal := (8,8)
start := (0,0)
checks := stack of cells to verify
all cells in wallmaze := empty
for cell in distmaze
    cell := shortest dist to goal
checks.push(start)
while(start != goal)
    while(checks !empty)
        cellCheck := checks.pop
        if cellCheck.value isn't 1 greater than accessible neighbors
            minVal := minimal value of accessible neighbors of cellCheck
            cellCheck := minVal++
            for neighbor in cellCheck neighbors
                cellCheck.push(neighbor)
    advance to next ideal neighbor
return ideal path*/
}

void leftEncoderEvent() {
  if (digitalRead(aPinL) == digitalRead(bPinL)) {
    countLRA++;
    }
  else {
    countLRA--;
  }
}

void rightEncoderEvent() {
  if (digitalRead(aPinR) == digitalRead(bPinR)) {
    countRRA++;
    }
  else {
    countRRA--;
  }
}

//returns whether a given wall is not an edge:
bool testWall(int locX, int locY, int orient) {
  //detect ajacent maze edges
  bool edgeU = locX == SIZEX - 1;
  bool edgeR = locY == SIZEX - 1;
  bool edgeD = locX == 0;
  bool edgeL = locY == 0;

  //compare to orientation:
  switch(orient) {
    case 0:
      return edgeU;
    break;
    case 1:
      return edgeR;
    break;
    case 2:
      return edgeD;
    break;
    case 3:
      return edgeL;
    break;
    default://this default is fail dangerous, may lead to endless loop if default set true or let mouse crash into wall if false. It's better to let it crash so we can correct it sooner.
      return false;
  }
}

//returns true if a given cell exists:
bool testCell(int locX, int locY) {
  return locX >= 0 && locX < SIZEX && locY >= 0 &&locY < SIZEY;
}

//sets correct wall to state passed to it:
void setMazeWall(int mouseOrient, bool wallState) {
  int indexX = posX;
  int indexY = posY;
  
  if(mouseOrient % 2 == 1)//points to vertical directions (horizontal needs no addition)
    indexX += SIZEX;//shift to correct region of array

  if(mouseOrient == 2)
    indexY--;

  if(mouseOrient == 3)
    indexX--;

    mazeWalls[indexX][indexY] = wallState;
}

//returns the requested int from a scan of neighbors
int findNeighborLow(int locX, int locY, char infoReq) {
  
//infoReq: 'o' = orientation [0 to 3]; 'v' = value [0 to 255]
int dirNeighborLow = -1;//null
int valNeighborLow = 256;//null


//for each direction, compare
//tests if each neighbor exists and if there isn't a wall between them
//!make more legible

//up
if(testCell(locX + 1, locY) && valNeighborLow > mazeDist[locX + 1][locY] && testWall(locX, locY, 0) && !mazeWalls[locX][locY + 1]) {
  dirNeighborLow = 0;
  valNeighborLow = mazeDist[locX + 1][locY];
}
//right
if(testCell(locX, locY + 1) && valNeighborLow > mazeDist[locX][locY + 1] && testWall(locX, locY, 1) && !mazeWalls[locX + SIZEX][locY]) {
  dirNeighborLow = 1;
  valNeighborLow = mazeDist[locX][locY + 1];
}
//down
if(testCell(locX - 1, locY) && valNeighborLow > mazeDist[locX - 1][locY] && testWall(locX, locY, 2) && !mazeWalls[locX][locY]) {
  dirNeighborLow = 2;
  valNeighborLow = mazeDist[locX - 1][locY];
}
//left
if(testCell(locX, locY - 1) && valNeighborLow > mazeDist[locX][locY - 1] && testWall(locX, locY, 3) && !mazeWalls[locX + SIZEX - 1][locY]) {
  dirNeighborLow = 3;
  valNeighborLow = mazeDist[locX][locY - 1];
}

switch(infoReq) {
  case 'o':
    return dirNeighborLow;
    break;
  case 'v':
    return valNeighborLow;
    break;
  default:
    return -1;
  }
}

void pushQueue(int item) {
  checkQueue[checkSize] = item;
  checkSize++;
}

int popQueue() {
  //error case:
  if(checkSize < 1)
    return -1;
    
  checkSize--;
  return checkQueue[checkSize];
}

void cellUpdate(int cell) {
  //decompose into x and y
  int locX = cell / SIZEX;
  int locY = cell % SIZEX;

    //if path is broken(all neighbors have greater distance to goal):
  if(mazeDist[locX][locY] != findNeighborLow(locX, locY, 'v') + 1) {
    //update this cell's value:
    mazeDist[locX][locY] = findNeighborLow(locX, locY, 'v') + 1;
    //then push neighbors onto checkQueue:
    //up
    if(testCell(locX, locY + 1) && !mazeWalls[locX + 1][locY]) {
      pushQueue(cell + SIZEX);
    }
    //right
    if(testCell(locX + 1, locY) && !mazeWalls[locX + SIZEX][locY]) {
      pushQueue(cell + 1);
    }
    //down
    if(testCell(locX, locY - 1) && !mazeWalls[locX][locY]) {
      pushQueue(cell - SIZEX);
    }
    //left
    if(testCell(locX - 1, locY) && !mazeWalls[locX + SIZEX - 1][locY]) {
     pushQueue(cell - 1);
    }
    
  }
}

//5/16/18
