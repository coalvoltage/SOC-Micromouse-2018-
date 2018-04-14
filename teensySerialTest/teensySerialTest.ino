int number = 0;
int writeVolt = 255;
int irRecievePinL = A5;
int irRecievePinFL = A4;
int irRecievePinFR = A3;
int irRecievePinR = A2;

int readL = 0;
int readFL = 0;
int readFR = 0;
int readR = 0;

int irEmitPinL = A9;
int irEmitPinFL = A8;
int irEmitPinFR = A7;
int irEmitPinR = A6;

int interL, interFL, interFR, interR;
int interLightL, interLightFL, interLightFR, interLightR;

void setup() {
  Serial.begin(9600);
  pinMode(irRecievePinL, INPUT);
  pinMode(irRecievePinFL, INPUT);
  pinMode(irRecievePinFR, INPUT);
  pinMode(irRecievePinR, INPUT);
  pinMode(irEmitPinL, OUTPUT);
  pinMode(irEmitPinFL, OUTPUT);
  pinMode(irEmitPinFR, OUTPUT);
  pinMode(irEmitPinR, OUTPUT);

  interL = analogRead(irRecievePinL);
  interFL = analogRead(irRecievePinFL);
  interFR = analogRead(irRecievePinFR);
  interR = analogRead(irRecievePinR);

  analogWrite(irEmitPinL, 255);
  analogWrite(irEmitPinFL, 255);
  analogWrite(irEmitPinFR, 255);
  analogWrite(irEmitPinR, 255);

  delay(200);
  
  interLightL = analogRead(irRecievePinL);
  interLightFL = analogRead(irRecievePinFL);
  interLightFR = analogRead(irRecievePinFR);
  interLightR = analogRead(irRecievePinR);
}

void loop() {
  if(number > 300) {
    number = 0;
  }
  
  analogWrite(irEmitPinL, 255);
  analogWrite(irEmitPinFL, 255);
  analogWrite(irEmitPinFR, 255);
  analogWrite(irEmitPinR, 255);
  
  readL = analogRead(irRecievePinL) - interL /*- interLightL*/;
  readFL = analogRead(irRecievePinFL) - interFL /*- interLightFL*/;
  readFR = analogRead(irRecievePinFR) - interFR /*- interLightFR*/;
  readR = analogRead(irRecievePinR) - interR /*- interLightR*/;
  
  
  Serial.print("Right: ");
  Serial.println(readR);
  Serial.print("Interference: ");
  Serial.println(interR);
  Serial.print("RightTop: ");
  Serial.println(readFR);
  Serial.print("Interference: ");
  Serial.println(interLightFR);
  Serial.print("LeftTop: ");
  Serial.println(readFL);
  Serial.print("Interference: ");
  Serial.println(interLightFL);
  Serial.print("Left: ");
  Serial.println(readL);
  Serial.print("Interference: ");
  Serial.println(interL);
  Serial.println(number);
  number++;
  delay(200);
}
