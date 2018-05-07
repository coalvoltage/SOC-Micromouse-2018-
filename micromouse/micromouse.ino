/*  
 *  Micromouse 2018
 *  UCR Robotics, Team: Straight 'Outta Cheddar
 */

//Pins:
  //sensors: F = front
     //inputs:
int irRecievePinL = A5;
int irRecievePinFL = A4;
int irRecievePinFR = A3;
int irRecievePinR = A2;
      //outputs:
int irEmitPinL = A9;
int irEmitPinFL = A8;
int irEmitPinFR = A7;
int irEmitPinR = A6;
  //motors:
    //controls:
int forwardPinR = 6;
int reversePinR = 5;
int forwardPinL = 4;
int reversePinL = 3;
    //timing data:
int aPinL = 7;
int bPinL = 8;
int aPinR = 9;
int bPinR = 10;
  //led:
int ledPin = 13;
bool isLedOn = false;


//Maze parameters:
#define SIZEX 16    //counting from 1
#define SIZEY 16
const int GOALX = 7;//counting from 0
const int GOALY = 7;


//Sensor vars:
int sensorReadL, sensorReadFL, sensorReadFR, sensorReadR;
int readingWallLeft, readingWallRight, readingWallFront = 0;//threshold sensor readings to confirm a wall


//Locomotion vars:
  //directional commands, used in int userCommand:
#define USERBRK 0
#define USERFOR 1
#define USERREV 2
#define USERLEF 3
#define USERRIG 4
#define USERINV 5
#define USERRTU 6

int userCommand = USERBRK;

bool isTurning = false;//not in control structure


//Calibration and speed controls:
int speedMaxLeft = 255;
int speedMaxRight = 255;
int speedLeft = 0;
int speedRight = 0;
int recoverSpeedL;
int recoverSpeedR;
  //mapped values
int mappedL = 1000;//!should not be hardcoded
int mappedR = 1000;//!should not be hardcoded
//const int speedREV = 0;
//const int speedFOR = 255;
const int speedNEU = 127;

  //steady-state interference:
int interL, interFL, interFR, interR;

  //PID and corrections:
const double kP = 0.15;
//const double kI = 0.0;
//const double kD = 0.0;

//const int sensorReadCorrectionBoundL = 30;
//const int sensorReadCorrectionBoundR = 30;
int displacementReadings;

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
long currentLRABound = 450;//not used
long currentRRABound = 450;//not used
//175
long currentTurnBound = 170;
long currentFullBound = 350;
//timer values
unsigned long blinkerMillis = 0;
unsigned long irMillis = 0;
unsigned long infoMillis = 0;
unsigned long actionMillis = 0;
unsigned long correctionMillis = 5;
unsigned long currentMillis;

    //delays:
const unsigned long blinkerDelay = 1000;

const unsigned long irDelay = 10;
bool areIREmittersOn = true;

const unsigned long infoDelay = 1000;//delay between debug updates
const unsigned long correctionDelay = 10;//delay between speed updates
const unsigned long actionDelay = 2000;
const unsigned long breakDelay = 1000;


//Solver vars
int mazeDist[SIZEX][SIZEY];
int mazeWalls[SIZEX][SIZEY];

int checkQueue[SIZEX * SIZEY];
int checkTempValue;
int checkSize = 0;

bool wallRight, wallBack, wallLeft, wallFront = false;

bool routeFound = false;

int posX = 0;
int posY = 0;

/*
 * Up = 1
 * Right = 2
 * Down = 3
 * Left = 4
 */
char mouseOrient = 1;

bool switchMove = false;
bool actionFinished = true;
bool actionLeft = false;
bool actionRight = false;

bool recoveryMode = false;



//function declarations
  //mouse movements
void moveBreak(int pinFor, int pinRev);

void turnLeft(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void turnRight(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void rightMotor(int pinF,int pinR, int sp1,int sp2);
void leftMotor(int pinF,int pinR, int sp1,int sp2);

void moveWheelsFor(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR);
  //ir functions

void leftEncoderEvent();
void rightEncoderEvent();

//Debug vars:
char keyboardInput = '\0';



void setup() {
  Serial1.begin(9600);//Serial is used for USB, Serial1 is used for HC059(Bluetooth)
  //Define pins:
    //encoder pins
  pinMode(aPinL, INPUT);
  pinMode(bPinL, INPUT);
  pinMode(aPinR, INPUT);
  pinMode(bPinR, INPUT);
  
    //motor control pins:
  pinMode(forwardPinL, OUTPUT);
  pinMode(reversePinR, OUTPUT);
  pinMode(forwardPinL, OUTPUT);
  pinMode(reversePinR, OUTPUT);

  //ir receiver pins:
  pinMode(irRecievePinL, INPUT);
  pinMode(irRecievePinFL, INPUT);
  pinMode(irRecievePinFR, INPUT);
  pinMode(irRecievePinR, INPUT);
  //ir emitter pins:
  pinMode(irEmitPinL, OUTPUT);
  pinMode(irEmitPinFL, OUTPUT);
  pinMode(irEmitPinFR, OUTPUT);
  pinMode(irEmitPinR, OUTPUT);
  //led pin:
  pinMode(ledPin, OUTPUT);

  //find interference:
  analogWrite(irEmitPinL, 0);
  analogWrite(irEmitPinFL, 0);
  analogWrite(irEmitPinFR, 0);
  analogWrite(irEmitPinR, 0);
  delay(10);//calibrations
  interL = analogRead(irRecievePinL);
  interFL =  analogRead(irRecievePinFL);
  interFR =  analogRead(irRecievePinFR);
  interR =  analogRead(irRecievePinR);


  //setup maze vars:
  //creates a pattern in which the corners are 0 and each orthogonally adjacent index iteratively increments by 1
  for(int i = 0; i < (SIZEX + 1)/2; i++) {
    for(int j = 0; j < (SIZEY + 1)/2; j++) {
      int val = i + j;
      mazeDist[i][j] = val;
      mazeDist[i][SIZEY - j - 1] = val;
      mazeDist[SIZEX - i - 1][j] = val;
      mazeDist[SIZEX - i - 1][SIZEY - j - 1] = val;
    }
  }
  
  checkQueue[0] = mazeDist[0][0];
  checkSize++;
  //debug keyboard
  
  //set bounds
  //ready to go
  //calibrations
  analogWrite(irEmitPinFR, 255);
  sensorReadFR = analogRead(irRecievePinFR) - interFR;
  digitalWrite(ledPin, HIGH);

  //! rework
  bool settingVars = true;//not in control structure(used in setup only)
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

  //attachInterupts for motor edge counts:
  attachInterrupt(digitalPinToInterrupt(aPinL), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(aPinR), rightEncoderEvent, CHANGE);
}



void loop() {
  //blinks led every 2 loops
  currentMillis = millis();
  if(currentMillis - blinkerMillis >= blinkerDelay) {
    if (!isLedOn) {
      digitalWrite(ledPin, HIGH);
      isLedOn = true;
    }
    else {
      digitalWrite(ledPin, LOW);
      isLedOn = false;
    }
    blinkerMillis = currentMillis;
  }


  //Debug keyboard input:
  if(Serial1.available() > 0) {
    keyboardInput = Serial1.read();
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
  
  /*if(posX != goalx && posY != goaly) {
    checkQueue[0] = mazeDist[posX][posY];
    checkSize = 1;
    while(checkSize != 0) {
      checkTempValue = checkQueue[checkSize - 1];
      checkSize--;
      if(checkTempValue != 
    }
    //
  }
  else if(posX == goalx && posY == goaly) {
    
  }*/
  //finds interference and reads

 //Calibrations
  if(currentMillis - irMillis >= irDelay) {
    if(areIREmittersOn) {
      sensorReadL = analogRead(irRecievePinL) - interL;
      sensorReadFL = analogRead(irRecievePinFL) - interFL;
      sensorReadFR = analogRead(irRecievePinFR) - interFR;
      sensorReadR = analogRead(irRecievePinR) - interR;

      sensorReadL = map(sensorReadL, 0, mappedL, 0, 500) + 20;
      sensorReadFL = map(sensorReadFL, 0, 1000, 0, 500);
      sensorReadFR = map(sensorReadFR, 0, 1000, 0, 500);
      sensorReadR = pow(map(sensorReadR, 0, mappedR, 0, 500),1.1);

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

  if(currentMillis - correctionMillis > correctionDelay) {
    if(sensorReadL > sensorReadR && areIREmittersOn && wallLeft && wallRight && userCommand == USERFOR) {
      displacementReadings = sensorReadL - sensorReadR;
      if(displacementReadings >= 0) {
        speedLeft = speedMaxLeft + (displacementReadings * kP);
        speedRight = speedMaxRight - (displacementReadings * kP);
        //Serial1.println("displacementReadings: left");
        //Serial1.println(displacementReadings);
      }
    }
    else if (sensorReadL < sensorReadR && areIREmittersOn && wallLeft && wallRight && userCommand == USERFOR){
      displacementReadings = sensorReadR - sensorReadL;
      if(displacementReadings >= 0) {
        speedLeft = speedMaxLeft - (displacementReadings * kP);
        speedRight = speedMaxRight + (displacementReadings * kP);
        //Serial1.println("displacementReadings: right");
        //Serial1.println(displacementReadings);
      }
    }
    correctionMillis = currentMillis;
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
  else if(!actionFinished) {
    if(userCommand == USERRIG) {
      if(countLRASaved - countLRA  >= currentTurnBound || countLRA - countLRASaved >= currentTurnBound) {
        actionLeft = true;
        leftMotor(forwardPinL, reversePinL, 0, 0);
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
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
      }
    }
    else if(userCommand == USERLEF) {
      if(countLRASaved - countLRA  >= currentTurnBound || countLRA - countLRASaved >= currentTurnBound) {
        actionLeft = true;
        leftMotor(forwardPinL, reversePinL, 0, 0);
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
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
      }
    }
    else if(userCommand == USERINV) {
      if((abs(countLRASaved - countLRA)>= currentFullBound)) {
        actionFinished = true;
        isTurning = false;
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
      }
    }
    else if(userCommand == USERFOR) {
      if(countLRASaved - countLRA >= currentLRABound || countLRA- countLRASaved >= currentLRABound) {
        actionFinished = true;
      }
      else if(wallFront) {
        actionFinished = true;
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
  }
  else if(switchMove) {
    if(!wallLeft && wallFront) {
      userCommand = USERLEF;
      switchMove = false;
      actionFinished = false;
      actionLeft = false;
      actionRight = false;
      countLRASaved = countLRA;
      countRRASaved = countRRA;
      actionMillis = currentMillis;
      isTurning = true;
    }
    else if(!wallRight && wallFront) {
      userCommand = USERRIG;
      switchMove = false;
      actionFinished = false;
      actionLeft = false;
      actionRight = false;
      countLRASaved = countLRA;
      countRRASaved = countRRA;
      actionMillis = currentMillis;
      isTurning = true;
    }
    else if(wallFront) {
      userCommand = USERINV;
      switchMove = false;
      actionFinished = false;
      actionLeft = false;
      actionRight = false;
      countLRASaved = countLRA;
      actionMillis = currentMillis;
      isTurning = true;
    }
    else {
      userCommand = USERFOR;
      switchMove = false;
      actionFinished = false;
      countLRASaved = countLRA;
      actionMillis = currentMillis;
    }
  }
  else {
      userCommand = USERBRK;
    switchMove = true;
    actionFinished = false;
    actionMillis = currentMillis;
  }
  //userCommand = USERBRK;
  if(userCommand != USERLEF && userCommand != USERRIG) {
    moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  }
  
  if(currentMillis - infoMillis >= infoDelay) {
    Serial1.print("LeftSpeed: ");
    Serial1.println(speedLeft);
    Serial1.print("RightSpeed: ");
    Serial1.println(speedRight);
    Serial1.print("I am :");
    if(userCommand == USERFOR) {
      Serial1.println("going forward.");
    }
    else if(recoveryMode) {
      Serial1.println("in recoveryMode.");
    }
    else if(userCommand == USERINV) {
      Serial1.println("turning around.");
    }
    else if(userCommand == USERRIG) {
      Serial1.println("turning right.");
    }
    else if(userCommand == USERLEF) {
      Serial1.println("turning left.");
    }
    /*Serial1.print("TicksL: ");
    Serial1.println(countLRA);
    Serial1.print("TicksR: ");
    Serial1.println(countRRA);*/
    Serial1.print("Right: ");
    Serial1.println(sensorReadR);
    Serial1.print("Interference: ");
    Serial1.println(interR);
    Serial1.print("RightTop: ");
    Serial1.println(sensorReadFR);
    Serial1.print("Interference: ");
    Serial1.println(interFR);
    Serial1.print("LeftTop: ");
    Serial1.println(sensorReadFL);
    Serial1.print("Interference: ");
    Serial1.println(interFL);
    Serial1.print("Left: ");
    Serial1.println(sensorReadL);
    Serial1.print("Interference: ");
    Serial1.println(interL);
    infoMillis = currentMillis;
  }
}



//Function definitions:

  //simple locomotion:
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
  if (digitalRead(aPinL) == HIGH) {
    if (digitalRead(bPinL) == LOW) {
      countLRA--;
    }
    else {
      countLRA++;
    }
  }
  else {
    if(digitalRead(bPinL) == LOW) {
      countLRA++;
    }
    else {
      countLRA--;
    }
  }
}

void rightEncoderEvent() {
  if (digitalRead(aPinR) == HIGH) {
    if (digitalRead(bPinR) == LOW) {
      countRRA--;
    }
    else {
      countRRA++;
    }
  }
  else {
    if(digitalRead(bPinR) == LOW) {
      countRRA++;
    }
    else {
      countRRA--;
    }
  }
}
