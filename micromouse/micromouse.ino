/*  
 *  Micromouse 2018
 *  UCR Robotics, Team: Straight 'Outta Cheddar
 */

//Pins:
  //sensors: F = front
     //inputs:
int irReceivePinL = A5;
int irReceivePinFL = A4;
int irReceivePinFR = A3;
int irReceivePinR = A2;
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
int readingWallF, readingWallL, readingWallR = 0;//threshold sensor readings to confirm a wall
double wallToleranceLow = .05;//lower bound for IR wall detection as compared to calibrations;

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

int initSamples = 10;//number of interference samples on startup

int speedMaxLeft = 255;//ceiling of motor output
int speedMaxRight = 255;
int speedLeft = 0;//actual speed
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
long currentLRABound = 450;
long currentRRABound = 450;
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

int posX = 0;//corresponds to BOTTOM LEFT
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



//Function declarations

  //mouse movements
void moveBreak(int pinFor, int pinRev);

void turnLeft(int spL, int spR);
void turnRight(int spL, int sp);

void rightMotor(int pinF,int pinR, int sp1,int sp2);
void leftMotor(int pinF,int pinR, int sp1,int sp2);

void moveWheelsFor(int spL, int spR);
void moveWheelsRev(int spL, int spR);

void moveMouse(int userCommand,int speedLeft,int speedRight);

//sets emitters to 'val' analog output and delays 'del' ms to allow adjustment time:
void setEmitterState(int val, int del);

//sets interference values using average of 'samples' number of tests and 'del' ms delay:
void setInter(int samples, int del);

//resets readingWall based on the position of the initial cell:
void setWallThreshold(char orient);

  //IR interrupts:
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
  
  pinMode(irReceivePinL, INPUT);
  pinMode(irReceivePinFL, INPUT);
  pinMode(irReceivePinFR, INPUT);
  pinMode(irReceivePinR, INPUT);
  
  pinMode(irEmitPinL, OUTPUT);
  pinMode(irEmitPinFL, OUTPUT);
  pinMode(irEmitPinFR, OUTPUT);
  pinMode(irEmitPinR, OUTPUT);
  
  pinMode(ledPin, OUTPUT);


  //attachInterupts for motor edge counts:
  attachInterrupt(digitalPinToInterrupt(aPinL), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(aPinR), rightEncoderEvent, CHANGE);

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



  //! rework
  /*
  
  //calibrations
  analogWrite(irEmitPinFR, 255);
  sensorReadFR = analogRead(irReceivePinFR) - interFR;
  digitalWrite(ledPin, HIGH);


  bool settingVars = true;//not in control structure(used in setup only)
  while(settingVars){
    analogWrite(irEmitPinFR, 255);
    analogWrite(irEmitPinL, 255);
    analogWrite(irEmitPinR, 255);
    sensorReadFR = analogRead(irReceivePinFR);
    //mappedL = analogRead(irReceivePinL);
    //mappedR = analogRead(irReceivePinR);
        (forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    if(sensorReadFR >= 500) {
      settingVars = false;
    }
  }
*/



//MOUSE IS MOVING STARTING NOW

  //perform inital calibrations, assuming perfect inital position:
  setInter(initSamples, 50);

  //initial orientation = DOWN
  
  setWallThreshold('f');
  setWallThreshold('r');

  //wait for button || auto orient to LEFT


  
  //orientation = LEFT

  setWallThreshold('l');


  //Serial1.println("Walls:");
  //Serial1.println("Front: " + readingWallF);
  //Serial1.println("Right: " + readingWallR);
  //Serial1.println("Left: " + readingWallL);

  
  //wait for button || auto orient to UP

  

  //TEST ONLY: scan all 4 directions, then report what directions mouse thinks are walls:
  //testWallDetection();

  //TEST ONLY: turn from facing a wall to another wall, report difference between original and now
  //testTurnFidelity();

  }





void loop() {
 
  currentMillis = millis();//time set
  
  //blinks led every 2 loops if enough time passes
  if(currentMillis - blinkerMillis >= blinkerDelay) {
    if (!isLedOn) {
      digitalWrite(ledPin, HIGH);
    }
    else {
      digitalWrite(ledPin, LOW);
    }
    
    isLedOn = !isLedOn;
    blinkerMillis = currentMillis;//reset timer
  }


  //Debug keyboard input:
  if(Serial1.available() > 0) {
    keyboardInput = Serial1.read();
    
    switch(keyboardInput) {
    case 'b':
      userCommand = USERBRK;
    break;
    case 'w':
      userCommand = USERFOR;
    break;
    case 's':
      userCommand = USERREV;
    break;
    case 'a':
      userCommand = USERLEF;
    break;
    case 'd':
      userCommand = USERRIG;
    break;
    case 'x':
      userCommand = USERINV;
    break;
    }
    
    countLRASaved = countLRA;
  }
  
  /*if(posX != goalx && posY != goaly) {
  if(posX != goalx && posY != goaly) {
    checkQueue[0] = mazeDist[posX][posY];
    checkSize = 1;
    while(checkSize != 0) {
      checkTempValue = checkQueue[checkSize - 1];
      checkSize--;
      if(wallFront) {
        
      }
      if(wallLeft) {
        
      }
      if(wallRight) {
        
      }
      if(wallBack) {
        
      }
    }
    //
  }
  else if(posX == goalx && posY == goaly) {
    
  }*/
  
  //finds interference and reads
  if(currentMillis - irMillis >= irDelay) {
    if(areIREmittersOn) {
      sensorReadL = analogRead(irReceivePinL) - interL;
      sensorReadFL = analogRead(irReceivePinFL) - interFL;
      sensorReadFR = analogRead(irReceivePinFR) - interFR;
      sensorReadR = analogRead(irReceivePinR) - interR;

      sensorReadL = map(sensorReadL, 0, mappedL, 0, 500) + 20;
      sensorReadFL = map(sensorReadFL, 0, 1000, 0, 500);
      sensorReadFR = map(sensorReadFR, 0, 1000, 0, 500);
      sensorReadR = pow(map(sensorReadR, 0, mappedR, 0, 500),1.1);

      wallFront = (sensorReadFL + sensorReadFR >= 2 * readingWallF);
      wallLeft = (sensorReadL >= readingWallL);
      wallRight = (sensorReadR >= readingWallR);

      setEmitterState(0,0);
    }
    else {
      interL = analogRead(irReceivePinL);
      interFL =  analogRead(irReceivePinFL);
      interFR =  analogRead(irReceivePinFR);
      interR =  analogRead(irReceivePinR);

      setEmitterState(255, 0);
    }
    
    areIREmittersOn = !areIREmittersOn;
    irMillis = currentMillis;//reset timer
  }



//Proportional Error Correction
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
    correctionMillis = currentMillis;//reset timer
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
      if(abs(countLRASaved - countLRA) >= currentLRABound) {
        actionFinished = true;
        recoveryMode = false;
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
      }
    }
  }
  else if(!actionFinished) {
    if(userCommand == USERRIG) {
      if(abs(countLRASaved - countLRA)  >= currentTurnBound) {
        actionLeft = true;
        leftMotor(forwardPinL, reversePinL, 0, 0);
      }
      else{
        leftMotor(forwardPinL, reversePinL, speedLeft,0);
      }
      if(abs(countRRASaved - countRRA)  >= currentTurnBound) {
        actionRight = true;
        rightMotor(forwardPinR, reversePinR, 0, 0);
      }
      else {
        rightMotor(forwardPinR, reversePinR, 0, speedRight);
      }
      if(actionRight && actionLeft) {
        if(mouseOrient == 4) {
          mouseOrient = 1;
        }
        else {
          mouseOrient++;
        }
        actionFinished = true;
        isTurning = false;
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
      }
    }
    else if(userCommand == USERLEF) {
      if(abs(countLRASaved - countLRA)  >= currentTurnBound) {
        actionLeft = true;
        leftMotor(forwardPinL, reversePinL, 0, 0);
      }
      else {
        leftMotor(forwardPinL, reversePinL, 0,speedLeft);
      }
      if(abs(countRRASaved - countRRA)  >= currentTurnBound) {
        actionRight = true;
        rightMotor(forwardPinR, reversePinR, 0 ,0);
      }
      else {
        rightMotor(forwardPinR, reversePinR, speedRight,0);
      }
      if(actionRight && actionLeft) {
        if(mouseOrient == 1) {
          mouseOrient = 4;
        }
        else {
          mouseOrient--;
        }
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
    moveMouse(userCommand, speedLeft, speedRight);
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


void turnLeft(int spL, int spR) {
  analogWrite(forwardPinL, 0);
  analogWrite(reversePinL, spL);
  analogWrite(forwardPinR, 0);
  analogWrite(reversePinR, spR);
}

void turnRight(int spL, int spR) {
  analogWrite(forwardPinL, spL);
  analogWrite(reversePinL, 0);
  analogWrite(forwardPinR, spR);
  analogWrite(reversePinR, 0);
}

void turnHalfCircle(int spL, int spR) {
  analogWrite(forwardPinL, 0);
  analogWrite(reversePinL, spL);
  analogWrite(forwardPinR, 0);
  analogWrite(reversePinR, spR);
}

void moveMouse(int userCommand,int speedLeft,int speedRight) {
  switch(userCommand) {
    case USERBRK:
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    break;
    
    case USERFOR:
    moveWheelsFor(speedLeft, speedRight);
    break;
    
    case USERREV:
    moveWheelsRev(speedLeft, speedRight);
    break;
    
    case USERLEF:
    turnLeft(speedLeft, speedRight);
    break;
    
    case USERRIG:
    turnRight(speedLeft, speedRight);
    break;

    case USERINV:
    turnHalfCircle(speedLeft, speedRight);
    break;
    
    default:
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    break;
  }
}

void moveWheelsFor(int spL, int spR) {
  analogWrite(forwardPinL, spL);
  analogWrite(reversePinL, 0);
  analogWrite(forwardPinR, 0);
  analogWrite(reversePinR, spR);
}

void moveWheelsRev(int spL, int spR) {
  analogWrite(forwardPinL, 0);
  analogWrite(reversePinL, spL);
  analogWrite(forwardPinR, spR);
  analogWrite(reversePinR, 0);
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


void setInter(int samp, int del) {
  int sumL = 0;
  int sumFL = 0;
  int sumFR = 0;
  int sumR = 0;
    
  setEmitterState(0, 10);

  for(int i = 1; i >= samp; i++) {
    sumL += analogRead(irReceivePinL);
    sumFL += analogRead(irReceivePinFL);
    sumFR += analogRead(irReceivePinFR);
    sumR += analogRead(irReceivePinR);

    delay(del);
  }
  
  interL = sumL / samp;
  interFL = sumFL / samp;
  interFR = sumFR / samp;
  interR = sumR / samp;
}



void setEmitterState(int val, int del){
  analogWrite(irEmitPinL, val);
  analogWrite(irEmitPinFL, val);
  analogWrite(irEmitPinFR, val);
  analogWrite(irEmitPinR, val);

  delay(del);
  }


void setWallThreshold(char orient) {
  setEmitterState(255, 10);
  switch(orient){
    case 'u':
      //left wall only
      readingWallL = analogRead(irReceivePinL) * (1 - wallToleranceLow);
      break;
    case 'r':
      //right wall only
      readingWallR = analogRead(irReceivePinR) * (1 - wallToleranceLow);
      break;
    case 'd':
      //front & right wall
      readingWallF = (analogRead(irReceivePinFL) + analogRead(irReceivePinFR) / 2 * (1 - wallToleranceLow));
      readingWallR = analogRead(irReceivePinR) * (1 - wallToleranceLow);        
      break;
    case 'l':
      //front & left wall
      readingWallF = (analogRead(irReceivePinFL) + analogRead(irReceivePinFR) / 2 * (1 - wallToleranceLow));
      readingWallL = analogRead(irReceivePinL) * (1 - wallToleranceLow);
      break;
  }
}

void blindTurn(char dir, int deg) {
  int savedTicksR = 
  switch(dir) {
    case 'r':
      while(



    
  }
}
