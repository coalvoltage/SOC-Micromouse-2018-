#define USERBRK 0
#define USERFOR 1
#define USERREV 2
#define USERLEF 3
#define USERRIG 4

//maze specs
int sizeX = 16;
int sizeY = 16;

int goalx = sizeX/2;
int goaly = sizeY/2;

int mazeDist[sizeX][sizeY];
int mazeWalls[sizeX][sizeY];

int checkQueue[sizeX * sizeY];


bool goalFound = false;


//pins
int irRecievePinL = A7;
int irRecievePinFL = A6;
int irRecievePinFR = A5;
int irRecievePinR = A4;

int irEmitPinL = 23;
int irEmitPinFL = 22;
int irEmitPinFR = 17;
int irEmitPinR = 16;

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

void moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  //ir functions
int findLightInterference(int rL, int rFL, int rFR, int rR, int eL, int eFL, int eFR, int eR);

void setup() {
  //define pins
  //encoder pins
  pinMode(aPinL, INPUT);
  pinMode(bPinL, INPUT);
  pinMode(aPinR, INPUT);
  pinMode(bPinR, INPUT);
  
  attachInterupt(
  
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
  inter = findLightInterference(irRecievePinL, irRecievePinFL, irRecievePinFR, irRecievePinR, irEmitPinL, irEmitPinFL, irEmitPinFR, irEmitPinR);

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
  
  
  
  userCommand = USERFOR;
  moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  
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

void moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR) {
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

int findLightInterference(int rL, int rFL, int rFR, int rR, int eL, int eFL, int eFR, int eR) {
	int interArray[4];
	int minInter;
	analogWrite(eL, LOW);
	analogWrite(eFL, LOW);
	analogWrite(eFR, LOW);
	analogWrite(eR, LOW);
	
	delay(100);
	
	interArray[0] = analogRead(rL);
	interArray[1] = analogRead(rFL);
	interArray[2] = analogRead(rFR);
	interArray[3] = analogRead(rR);
	
	minInter = interArray[0]
	for(int i = 1; i < 4; i++) {
		if(minVal > interArray[i]) {
			minInter = interArray[i];
		}
	}
	return minInter;
}