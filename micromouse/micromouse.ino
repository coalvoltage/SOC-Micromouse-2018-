#define USERBRK 0
#define USERFOR 1
#define USERREV 2
#define USERLEF 3
#define USERRIG 4
#define SIZEX 16
#define SIZEY 16
//maze specs
int sizeX = SIZEX;
int sizeY = SIZEY;

int goalx = sizeX/2;
int goaly = sizeY/2;

int mazeDist[SIZEX][SIZEY];
int mazeWalls[SIZEX][SIZEY];

int checkQueue[SIZEX * SIZEY];


bool goalFound = false;


//pins
int irRecievePinL = A5;
int irRecievePinFL = A4;
int irRecievePinFR = A3;
int irRecievePinR = A2;

int irEmitPinL = A9;
int irEmitPinFL = A8;
int irEmitPinFR = A7;
int irEmitPinR = A6;

int forwardPinL = 6;
int reversePinL = 5;
int forwardPinR = 4;
int reversePinR = 3;

int aPinL = 7;
int bPinL = 8;
int aPinR = 9;
int bPinR = 10;

//calculations
int speedCounter = 0;

int speedMax = 0;
int speedMaxLeft = 0;
int speedMaxRight = 0;
int speedLeft = 100;
int speedRight = 100;

int userCommand = 0;
int sensorReadL, sensorReadFL, sensorReadFR, sensorReadR;

int inter = 0;

//function declarations
  //mouse movements
void moveForward(int pinFor, int pinRev, int motSpeed);
void moveBackwards(int pinFor, int pinRev, int motSpeed);
void moveBreak(int pinFor, int pinRev);

void turnLeft(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);
void turnRight(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);

void moveWheels(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR);
  //ir functions
int findLightInterference(int rL, int rFL, int rFR, int rR, int eL, int eFL, int eFR, int eR);

void setup() {
  //define pins
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

  //setup maze vars
  for(int i = 0; i < sizeX/2; i++) {
	  for(int j = 0; j < sizeY/2; j++) {
      mazeDist[i][j] = sizeX - i - j - 2;
      mazeDist[sizeX - i - 1][sizeY - j - 1] = sizeX - i - j - 2;
      mazeDist[i][sizeY - j - 1] = sizeX - i - j - 2;
      mazeDist[sizeX - i - 1][j] = sizeX - i - j - 2;
	  }
  }
  //set bounds
  
}

void loop() {
  sensorReadL = analogRead(irRecievePinL) - inter;
  sensorReadFL = analogRead(irRecievePinFL) - inter;
  sensorReadFR = analogRead(irRecievePinFR) - inter;
  sensorReadR = analogRead(irRecievePinR) - inter;
  
  if(!goalFound) {
	  
  }

  /*if(sensorReadFL > sensorReadFR * 1.10) {
    speedMaxLeft = speedMax * 0.9;
    speedMaxRight = speedMax;
  }
  else if(sensorReadFL * 1.10 < sensorReadFR) {
    speedMaxLeft = speedMax;
    speedMaxRight = speedMax * 0.9;
  }
  else {
    speedMaxRight = speedMax;
    speedMaxLeft = speedMax;
  }*/
  
  
  //if (sensorReadFR <= 200) {
  //  userCommand = USERFOR;
  /*}
  else {
    userCommand = USERBRK;
  }*/
  /*Serial.println(userCommand);
  moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  delay(1000);*/
  userCommand = USERBRK;
  Serial.println(userCommand);
  moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  delay(1000);
  
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
    digitalWrite(pinFor, LOW);
    digitalWrite(pinRev, LOW);
}


void turnLeft(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed) {
  moveWheels(0, motSpeed, pinForL, pinRevL, pinForR, pinRevR);
}

void turnRight(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed) {
  moveWheels(motSpeed, 0, pinForL, pinRevL, pinForR, pinRevR);
}

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR) {
  switch(userCommand) {
    case USERBRK:
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    break;
    
    case USERFOR:
    moveWheels(speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
    break;
    
    case USERREV:
    moveWheels(speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
    break;
    
    case USERLEF:
    turnLeft(forwardPinL, reversePinL, forwardPinR, reversePinR, speedRight);
    break;
    
    case USERRIG:
    turnRight(forwardPinL, reversePinL, forwardPinR, reversePinR, speedLeft);
    break;
    
    default:
	  moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    break;
  }
}

void moveWheels(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
    if(spL > 0) {
      moveForward(pinForL, pinRevL, spL);
    }
    else {
      moveBackwards(pinForL, pinRevL, -1 * spL);
    }

    if(spR > 0) {
      moveForward(pinForR, pinRevR, spR);
    }
    else {
      moveBackwards(pinForR, pinRevR, -1 * spR);
    }
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

int findLightInterference(int pinEmit, int pinRecieve) {
	int minInter;
	analogWrite(pinEmit, 0);
	
	delay(10);

  minInter = analogRead(pinRecieve);
  
	return minInter;
}
