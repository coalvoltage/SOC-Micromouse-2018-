<<<<<<< HEAD
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
int readingWallLeft, readingWallRight, readingWallFront = 0;//threshold sensor readings to confirm a wall


//Locomotion vars:
  //directional commands, used in int userCommand:
=======
>>>>>>> 42c8e06bfa65a296a4ccf23e99228cb60d8e4912
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

<<<<<<< HEAD
void turnLeft(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void turnRight(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void rightMotor(int pinF,int pinR, int sp1,int sp2);
void leftMotor(int pinF,int pinR, int sp1,int sp2);

void moveWheelsFor(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);
void moveWheelsRev(int spL, int spR, int pinForL, int pinRevL, int pinForR, int pinRevR);

void moveMouse(int userCommand,int speedLeft,int speedRight,int forwardPinL,int reversePinL,int forwardPinR,int reversePinR);

//sets emitters to 'val' analog output and delays 'del' ms to allow adjustment time:
void setEmitterState(int val, int del);

//sets interference values using average of 'samples' number of tests and 'del' ms delay:
void setInter(int samples, int del);

  //IR interrupts:
void leftEncoderEvent();
void rightEncoderEvent();


//Debug vars:
char keyboardInput = '\0';
=======
void turnLeft(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);
void turnRight(int pinForL, int pinRevL, int pinForR, int pinRevR, int motSpeed);
>>>>>>> 42c8e06bfa65a296a4ccf23e99228cb60d8e4912

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

<<<<<<< HEAD

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




  
  //calibrations
  analogWrite(irEmitPinFR, 255);
  sensorReadFR = analogRead(irReceivePinFR) - interFR;
  digitalWrite(ledPin, HIGH);

  //! rework
  /*
  bool settingVars = true;//not in control structure(used in setup only)
  while(settingVars){
    analogWrite(irEmitPinFR, 255);
    analogWrite(irEmitPinL, 255);
    analogWrite(irEmitPinR, 255);
    sensorReadFR = analogRead(irReceivePinFR);
    //mappedL = analogRead(irReceivePinL);
    //mappedR = analogRead(irReceivePinR);
    moveBreak(forwardPinL, reversePinL);
    moveBreak(forwardPinR, reversePinR);
    if(sensorReadFR >= 500) {
      settingVars = false;
    }
  }
*/



  //perform inital calibrations, assuming perfect inital position:
  setInter(initSamples, 50);
  //ir on
  //sample
  //
  }


=======
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
>>>>>>> 42c8e06bfa65a296a4ccf23e99228cb60d8e4912

void loop() {
<<<<<<< HEAD
 
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

 //Wall Check:
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


      wallFront = (sensorReadFL >= readingWallFront || sensorReadFR >= readingWallFront);// could be changed to '(sensorReadFL + sensorRead FR >= 2 * readingWallFront)'
      wallLeft = (sensorReadL >= readingWallLeft);
      wallRight = (sensorReadR >= readingWallRight);
      
      analogWrite(irEmitPinL, 0);
      analogWrite(irEmitPinFL, 0);
      analogWrite(irEmitPinFR, 0);
      analogWrite(irEmitPinR, 0);
    }
    else {
      interL = analogRead(irReceivePinL);
      interFL =  analogRead(irReceivePinFL);
      interFR =  analogRead(irReceivePinFR);
      interR =  analogRead(irReceivePinR);
      
      analogWrite(irEmitPinL, 255);
      analogWrite(irEmitPinFL, 255);
      analogWrite(irEmitPinFR, 255);
      analogWrite(irEmitPinR, 255);
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
=======
  sensorReadL = analogRead(irRecievePinL) - inter;
  sensorReadFL = analogRead(irRecievePinFL) - inter;
  sensorReadFR = analogRead(irRecievePinFR) - inter;
  sensorReadR = analogRead(irRecievePinR) - inter;
  
  if(!goalFound) {
	  
>>>>>>> 42c8e06bfa65a296a4ccf23e99228cb60d8e4912
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

<<<<<<< HEAD


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
    
  setEmitterState(0);

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
  
=======
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
>>>>>>> 42c8e06bfa65a296a4ccf23e99228cb60d8e4912
