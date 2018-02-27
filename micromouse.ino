#define USERBRK 0;
#define USERFOR 1;
#define USERREV 2;
#define USERLEF 3;
#define USERRIG 4;

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
int speedLeft = 0;
int speedRight = 0;

int userCommand = 0;
int sensorReadL, sensorReadFL, sensorReadFR, sensorReadR;

//function declarations
void moveForward(int pinFor, int pinRev, int motSpeed);
void moveBackwards(int pinFor, int pinRev, int motSpeed);
void moveBreak(int pinFor, int pinRev);

void turnLeft(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);
void turnRight(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);

void moveWheels(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void setup() {
  pinMode(forwardPinL, OUTPUT);
  pinMode(reversePinR, OUTPUT);
  pinMode(forwardPinL, OUTPUT);
  pinMode(reversePinR, OUTPUT);

  pinMode(irRecievePinL, INPUT);
  pinMode(irRecievePinFL, INPUT);
  pinMode(irRecievePinFR, INPUT);
  pinMode(irRecievePinR, INPUT);

  //turn off emitters

  //set bounds
}

void loop() {
  sensorReadL = analogRead(irRecievePinL);
  sensorReadFL = analogRead(irRecievePinFL);
  sensorReadFR = analogRead(irRecievePinFR);
  sensorReadR = analogRead(irRecievePinR);

  if(sensorReadFL > sensorReadFR * 1.10) {
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
  }
  
  //input statement
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
    
    case default:
    break;
  }
}

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

