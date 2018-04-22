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
int speedMaxLeft = 200;
int speedMaxRight = 200;
double speedLeft = 16;
double speedRight = 14;

//const int speedREV = 0;
//const int speedFOR = 255;
const int speedNEU = 128;

int userCommand = USERBRK;
int sensorReadL, sensorReadFL, sensorReadFR, sensorReadR;

bool switchMove = false;

int timerDelay = 100;
int delayNormal = 100;
int delayHalfTurn = 400;
int interL, interFL, interFR, interR;

double Kp = 0.5;

const long countStepBound = 500;
const long countTurnBound = 500;
//interupts
volatile long countLRA = 0;
volatile long countRRA = 0;

volatile long countLRASaved = 0;
volatile long countRRASaved = 0;
long currentLRABound = 500;
long currentRRABound = 500;

//timer values
unsigned long blinkerMillis = 0;
unsigned long irMillis = 0;
unsigned long infoMillis = 0;
unsigned long actionMillis = 0;
unsigned long currentMillis;

const unsigned long blinkerDelay = 1000;

const unsigned long irDelay = 10;
bool areIREmittersOn = true;

const unsigned long infoDelay = 1000;

const unsigned long actionDelay = 10;

//function declarations
  //mouse movements
void moveBreak(int pinFor, int pinRev);

void turnLeft(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void turnRight(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void moveWheelsFor(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR);
  //ir functions
int findLightInterference(int pinEmit, int pinRecieve);

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
  interL = findLightInterference(irEmitPinL, irRecievePinL);
  interFL = findLightInterference(irEmitPinFL, irRecievePinFL);
  interFR = findLightInterference(irEmitPinFR, irRecievePinFR);
  interR = findLightInterference(irEmitPinR, irRecievePinR);
  //setup maze vars
  for(int i = 0; i < sizeX/2; i++) {
	  for(int j = 0; j < sizeY/2; j++) {
      mazeDist[i][j] = sizeX - i - j - 2;
      mazeDist[sizeX - i - 1][sizeY - j - 1] = sizeX - i - j - 2;
      mazeDist[i][sizeY - j - 1] = sizeX - i - j - 2;
      mazeDist[sizeX - i - 1][j] = sizeX - i - j - 2;
	  }
  }
  //debug keyboard
  
  //set bounds
  //ready to go
  analogWrite(irEmitPinFR, 255);
  sensorReadFR = analogRead(irRecievePinFR) - interFR;
  digitalWrite(ledPin, HIGH);
  while(sensorReadFR <= 400){
    analogWrite(irEmitPinFR, 255);
    sensorReadFR = analogRead(irRecievePinFR);
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
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
    else if(keyboardInput == 'd') {
      userCommand = USERRIG;
    }
    countLRASaved = countLRA;
  }
  
  //finds interference and reads
  if(currentMillis - irMillis >= irDelay) {
    if(areIREmittersOn) {
      analogWrite(irEmitPinL, 0);
      analogWrite(irEmitPinFL, 0);
      analogWrite(irEmitPinFR, 0);
      analogWrite(irEmitPinR, 0);
      
      interL = findLightInterference(irEmitPinL, irRecievePinL);
      interFL = findLightInterference(irEmitPinFL, irRecievePinFL);
      interFR = findLightInterference(irEmitPinFR, irRecievePinFR);
      interR = findLightInterference(irEmitPinR, irRecievePinR);
      areIREmittersOn = false;
    }
    else {
      analogWrite(irEmitPinL, 255);
      analogWrite(irEmitPinFL, 255);
      analogWrite(irEmitPinFR, 255);
      analogWrite(irEmitPinR, 255);
      
      sensorReadL = analogRead(irRecievePinL) - interL;
      sensorReadFL = analogRead(irRecievePinFL) - interFL;
      sensorReadFR = analogRead(irRecievePinFR) - interFR;
      sensorReadR = analogRead(irRecievePinR) - interR;

      areIREmittersOn = true;
    }
    irMillis = currentMillis;
  }
  if(countLRA - countLRASaved >= currentLRABound || countLRASaved - countLRA >= currentLRABound) {
    userCommand = USERBRK;
  }
  moveMouse(userCommand, speedMaxLeft, speedMaxRight, forwardPinL, reversePinL, forwardPinR, reversePinR);
  actionMillis = currentMillis;
  if(currentMillis - infoMillis >= infoDelay) {
    Serial.print("LeftSpeed: ");
    Serial.println(speedMaxLeft);
    Serial.print("RightSpeed: ");
    Serial.println(speedMaxRight);
    Serial.print("TicksL: ");
    Serial.println(countLRA);
    Serial.print("TicksR: ");
    Serial.println(countRRA);
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
