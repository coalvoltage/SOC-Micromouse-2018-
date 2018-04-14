#define USERBRK 0
#define USERFOR 1
#define USERREV 2
#define USERLEF 3
#define USERRIG 4
#define USERINV 5

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
int speedMultiplier = 10;

int speedMax = 250;
int speedMaxLeft = speedMax;
int speedMaxRight = speedMax;
double speedLeft = 16;
double speedRight = 14;

//const int speedREV = 0;
//const int speedFOR = 255;
const int speedNEU = 128;

int userCommand = 0;
int sensorReadL, sensorReadFL, sensorReadFR, sensorReadR;

bool switchMove = false;

int timerDelay = 100;
int delayNormal = 100;
int delayHalfTurn = 400;
int inter = 0;

double Kp = 0.5;

//function declarations
  //mouse movements
void moveBreak(int pinFor, int pinRev);

void turnLeft(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);
void turnRight(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);

void moveWheelsFor(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR);
  //ir functions
int findLightInterference(int rL, int rFL, int rFR, int rR, int eL, int eFL, int eFR, int eR);

void setup() {
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
  //ready to go
  analogWrite(irEmitPinFR, 255);
  sensorReadFR = analogRead(irRecievePinFR);
  digitalWrite(ledPin, HIGH);
  while(sensorReadFR <= 400){
    analogWrite(irEmitPinFR, 255);
    sensorReadFR = analogRead(irRecievePinFR);
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
  }
}

void loop() {
  analogWrite(irEmitPinL, 255);
  analogWrite(irEmitPinFL, 255);
  analogWrite(irEmitPinFR, 255);
  analogWrite(irEmitPinR, 255);
  if (!isLedOn) {
    digitalWrite(ledPin, HIGH);
    isLedOn = true;
  }
  else {
    digitalWrite(ledPin, LOW);
    isLedOn = false;
  }
  sensorReadL = analogRead(irRecievePinL) - inter;
  sensorReadFL = analogRead(irRecievePinFL) - inter;
  sensorReadFR = analogRead(irRecievePinFR);
  sensorReadR = analogRead(irRecievePinR) - inter;
  
  if(!goalFound) {
	  
  }

  if(sensorReadFL > sensorReadFR) {
    speedMaxLeft = speedMax - (sensorReadL > sensorReadR) * Kp;
    speedMaxRight = speedMax;
  }
  else if(sensorReadFL < sensorReadFR) {
    speedMaxLeft = speedMax;
    speedMaxRight = speedMax - (sensorReadL > sensorReadR) * Kp;
  }
  else {
    speedMaxRight = speedMax;
    speedMaxLeft = speedMax;
  }

  if(sensorReadFR <= 400) {
    userCommand = USERFOR;
    timerDelay = delayNormal;
    switchMove = false;
  }
  else if (sensorReadFR > 400){
    if(switchMove) {
      userCommand = USERINV;
      switchMove = false;
      timerDelay = delayHalfTurn;
    }
    else {
      userCommand = USERBRK;
      switchMove = true;
      timerDelay = delayHalfTurn;
    }
  }
  else {
    userCommand = USERBRK;
    switchMove = true;
    timerDelay = delayHalfTurn;
  }
  Serial.println(userCommand);
  moveMouse(userCommand, speedLeft, speedRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  delay(timerDelay);
  
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


void turnLeft(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed) {
 // moveWheels(0, motSpeed, pinForL, pinRevL, pinForR, pinRevR);
}

void turnRight(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed) {
  //moveWheels(motSpeed, 0, pinForL, pinRevL, pinForR, pinRevR);
}

void turnHalfCircle(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
  analogWrite(pinForL, 0);
  analogWrite(pinRevL, speedMultiplier * spL);
  analogWrite(pinForR, 0);
  analogWrite(pinRevR, speedMultiplier * spR);
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
    turnLeft(forwardPinL, reversePinL, forwardPinR, reversePinR, speedRight);
    break;
    
    case USERRIG:
    turnRight(forwardPinL, reversePinL, forwardPinR, reversePinR, speedLeft);
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
  analogWrite(pinForL, speedMultiplier * spL);
  analogWrite(pinRevL, 0);
  analogWrite(pinForR, 0);
  analogWrite(pinRevR, speedMultiplier * spR);
}

void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR) {
  analogWrite(pinForL, 0);
  analogWrite(pinRevL, speedMultiplier * spL);
  analogWrite(pinForR, speedMultiplier * spR);
  analogWrite(pinRevR, 0);
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
