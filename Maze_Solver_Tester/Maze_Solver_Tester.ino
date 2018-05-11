//Maze Solver Tester
//QA for solving algorithms.
//To setup, place iterator in void loop at label //ADD ALGOITHM
//to interface, put instructions in nextInstruction and call advance();
//default cell array is mazeDist[SizeX][SizeY]
//default wall array is mazeWalls[2 * SizeX][SizeY]
//use printState to print maze status to Serial.
#define USERBRK 0
#define USERFOR 1
#define USERREV 2
#define USERLEF 3
#define USERRIG 4
#define USERINV 5
int userCommand = USERBRK;

const int sizeX = 16;
const int sizeY = 16;
const int goalx = 7;
const int goaly = 7;

/*
 * Up = 1
 * Right = 2
 * Down = 3
 * Left = 4
 *
 */
char mouseOrient = 1;
bool wallRight, wallBack, wallLeft, wallFront = false;

int posX = 0;
int posY = 0;

int mazeDist[sizeX][sizeY];
bool mazeWalls[2 * sizeX][sizeY];//X indices 0 to SIZEX - 1 are horizontal, SIZEX - (2 * SIZEX) - 1 are vertical. 

int checkQueue[sizeX * sizeY];
int checkTempValue;
int checkSize = 0;

//MODES:
bool manualControl = true;

//ADD GLOBAL VARIABLES:


void setup() {
  //initialize MazeDist
  Serial.begin(9600);
  Serial.println("Serial initialized!");

  //populate mazeDist with inital pattern:
  for(int i = 0; i < sizeX/2; i++) {
    for(int j = 0; j < sizeY/2; j++) {
      int val = sizeX - i - j - 2;
      mazeDist[i][j] = val;
      mazeDist[sizeX - i - 1][sizeY - j - 1] = val;
      mazeDist[i][sizeY - j - 1] = val;
      mazeDist[sizeX - i - 1][j] = val;
    }
  }
  
  //populate mazeWalls with inital pattern:
  for(int i = 0; i < sizeX * 2; i++) {
    for(int j = 0; j < sizeY; j++) {
      mazeWalls[i][j] = false;
    }
  }

  //bottom
  for(int i = 0; i < sizeX; i++)
    mazeWalls[i][0] = true;
  //right
  for(int j = 0; j < sizeY; j++)
    mazeWalls[(2 * sizeX) - 1][j] = true;

}

void loop() {
  printWalls(true);
  while(true){}
}



void printDist() {
  Serial.println();
  Serial.print("\t");
  
  for(int i = 0; i < sizeX; i++){
    Serial.print("c:");
    Serial.print(i);
    Serial.print("\t");
  }
  Serial.println();
  
  for(int i = 0; i < sizeX; i++) {
      Serial.print("r:");
      Serial.print(i);
      Serial.print("\t");
    for(int j = sizeY - 1; j >= 0; j--) {
      Serial.print(mazeDist[i][j]);
      Serial.print("\t");
    }
    
    Serial.println();
    Serial.println();
  }
  
  Serial.print("PrintDist complete.");
}

//prints state of mazeWalls
//parameter addDist includes mazeDist values
void printWalls(bool addDist) {
  Serial.println();
  Serial.print("\t");


  //column titles:
  for(int i = 0; i < sizeX; i++) {
    Serial.print("c:");
    Serial.print(i);
    Serial.print("\t");
  }
  //Top edge
  Serial.println();
  Serial.print("\t");
  for(int i = 0; i < sizeX; i++) {
    Serial.print("+-------");
  }
  Serial.print("+");

  for(int i = 0; i < sizeX; i++) {
    //column titles
    Serial.println();
    Serial.print("r:");
    Serial.print(i);
    
    //vertical lines
    //p = number of lines rendered per wall
    int p = 3;
    for(int l = 0; l < p; l++) {
      Serial.print("\t|");
      for(int q = 0; q < sizeY; q++) {
        
        //add Dist if prompted
        if((l == p/2) && addDist) {
          Serial.print("   ");
          Serial.print(mazeDist[i][q]);
          }

         Serial.print("\t");
         
         if(mazeWalls[sizeX + q][(sizeX - 1) - i]) 
         Serial.print("|");

         //debug: prints cordinates referenced (comment out add Dist conditional above
         //Serial.print(sizeX + q);
         //Serial.print(" ");
         //Serial.print((sizeX - 1) - i);
      }
      
=======
      for(int q = 0; q < sizeX; q++) {
      if(mazeWalls[sizeX +i][q]) 
        Serial.print("|");

      //add Dist if prompted
      if((l == p/2) && addDist) {
        Serial.print("   ");
        Serial.print(mazeDist[i][q]);
        }
      
      Serial.print("\t");
      }
>>>>>>> 0b34201848d241216f4d7f7f71835845db4a8825
    Serial.println();
    }
    
    //horizontal lines
    Serial.print("\t");
    for(int j = 0; j < sizeY; j++) {
  
     Serial.print("+");
      if(mazeWalls[(sizeX - 1) - i][j])
        Serial.print("-------");
      else
        Serial.print("       ");
    }
    Serial.print("+");
  }
  Serial.print("\nPrintWalls Complete.");
}


//returns the length of an int string
//cond: -32,768 < num < 32,767 on Arduino UNO
int getDispLength(int num) {
  int len = 1;
  num = abs(num);
  while(num > 1) {
    num = (int)(num / 10);
    len++;
  }
  return len;
}

