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
 * Front = 1
 * Right = 2
 * Down = 3
 * Left = 4
 *
 */
char mouseOrient = 1;
bool wallRight, wallBack, wallLeft, wallFront = false;

int locationX = 0;
int locationY = 0;

bool settingVars = true;
bool isTurning = false;
//maze specs
int sizeX = SIZEX;
int sizeY = SIZEY;

int goalx = sizeX/2;
int goaly = sizeY/2;

int mazeDist[SIZEX][SIZEY];
int mazeWalls[SIZEX][SIZEY];

int checkQueue[SIZEX * SIZEY];
int checkTempValue;
int checkSize = 0;

bool goalFound = false;

int readingWallLeft = 140;
int readingWallRight = 140;
int readingWallFront = 300;
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

int speedMax = 250;
int speedMaxLeft = 165 ;
int speedMaxRight = 180;
int speedLeft = 200;
int speedRight = 200;
int recoverSpeedL;
int recoverSpeedR;
//mapped values
int mappedL = 1000;
int mappedR = 1000;
//const int speedREV = 0;
//const int speedFOR = 255;
const int speedNEU = 128;

int userCommand = USERBRK;
int sensorReadL, sensorReadFL, sensorReadFR, sensorReadR;

const int sensorReadCorrectionBoundL = 30;
const int sensorReadCorrectionBoundR = 30;

int displacementReadings;

bool switchMove = false;
bool actionFinished = true;
bool actionLeft = false;
bool actionRight = false;

bool recoveryMode = false;


int interL, interFL, interFR, interR;

const double kP = 0.2;


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
unsigned long correctionMillis = 10;
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

void rightMotor(int pinF,int pinR, int sp1,int sp2);
void leftMotor(int pinF,int pinR, int sp1,int sp2);

void moveWheelsFor(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR);
  //ir functions

void leftEncoderEvent();
void rightEncoderEvent();

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
      mazeDist[i][j] = sizeX - i - j - 2;
      mazeDist[sizeX - i - 1][sizeY - j - 1] = sizeX - i - j - 2;
      mazeDist[i][sizeY - j - 1] = sizeX - i - j - 2;
      mazeDist[sizeX - i - 1][j] = sizeX - i - j - 2;
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
      analogWrite(irEmitPinL, 0);
      analogWrite(irEmitPinFL, 0);
      analogWrite(irEmitPinFR, 0);
      analogWrite(irEmitPinR, 0);
      
      sensorReadL = analogRead(irRecievePinL) - interL;
      sensorReadFL = analogRead(irRecievePinFL) - interFL;
      sensorReadFR = analogRead(irRecievePinFR) - interFR;
      sensorReadR = analogRead(irRecievePinR) - interR;

      sensorReadL = map(sensorReadL, 0, mappedL, 0, 500) + 20;
      sensorReadFL = map(sensorReadFL, 0, 1000, 0, 500);
      sensorReadFR = map(pow(sensorReadFR,1.075) * 0.80, 0, 1000, 0, 500);
      sensorReadR = pow(map(sensorReadR, 0, mappedR, 0, 500),1.1);

      if(sensorReadFL >= readingWallFront && sensorReadFR >= readingWallFront) {
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

  if(currentMillis - correctionMillis > correctionDelay && areIREmittersOn ) {
    if(sensorReadL > sensorReadR && areIREmittersOn && wallLeft && wallRight && userCommand == USERFOR) {
      displacementReadings = sensorReadL - sensorReadR;
      if(displacementReadings >= 0) {
        speedLeft = speedMaxLeft + (displacementReadings * kP);
        speedRight = speedMaxRight - (displacementReadings * kP);
        //Serial.println("displacementReadings: left");
        //Serial.println(displacementReadings);
      }
    }
    else if (sensorReadL < sensorReadR && areIREmittersOn && wallLeft && wallRight && userCommand == USERFOR){
      displacementReadings = sensorReadR - sensorReadL;
      if(displacementReadings >= 0) {
        speedLeft = speedMaxLeft - (displacementReadings * kP);
        speedRight = speedMaxRight + (displacementReadings * kP);
        //Serial.println("displacementReadings: right");
        //Serial.println(displacementReadings);
      }
    }
    correctionMillis = currentMillis;
  }
  
  //breaks after a cell or action is completed
  /*if(recoveryMode) {
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
  }*/
  if(!actionFinished) {
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
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
        actionFinished = true;
        isTurning = false;
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
        speedLeft = speedMaxLeft;
        speedRight = speedMaxRight;
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
