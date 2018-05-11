//Maze Solver Tester
//QA for solving algorithms.
//To setup, place iterator in void loop at label //ADD ALGOITHM
//to interface, put instructions in nextInstruction and call advance();
//default cell array is mazeDist[SizeX][SizeY]
//default wall array is mazeWalls[2 * SizeX][SizeY]
//use printWalls() to print maze status to Serial.
#define USERBRK 0
#define USERFOR 1
#define USERREV 2
#define USERLEF 3
#define USERRIG 4
#define USERINV 5
int nextInstruction = USERBRK;

bool actionFinished = true;

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
  printWalls(true, false, false);
  
  //INSERT ALGORITHM:
  //Maze Solver //!Edited to use MazeWalls to read maze, and 
  if(actionFinished) {
  //scan walls:
    wallLeft = 
    
    wallFront = wallCheck(mouseOrient);
    wallRight = wallCheck((mouseOrient + 1) % 4);

    
  
  //set mazeWalls[][] using info from wall scan:
  int wallOrient = 0;//states what wall is being passed to a function  
    //left:
  wallOrient = (mouseOrient + 3) % 4;//gives dir of wall facing left sensor
  if(testWall(posX, posY, wallOrient))
    setMazeWall(wallOrient, wallLeft);

  //front:
  wallOrient = mouseOrient;//no addition for front
  if(testWall(posX, posY, wallOrient))
    setMazeWall(wallOrient, wallFront);

  //right:
  wallOrient = (mouseOrient + 1) % 4;//gives dir of wall facing right sensor
  if(testWall(posX, posY, wallOrient))
    setMazeWall(wallOrient, wallRight);

  //now, update mazeDist using checkQueue
  if(mazeDist[posX][posY] != findNeighborLow(posX, posY, 'v') + 1) {
    pushQueue(posX * SIZEX + posY);
    while(checkSize != 0) {
      cellUpdate(popQueue());
    }
  }

  //finally, choose new direction:
  //(choose lowest valued neighbor with tiebreaking priority: left, down, right, then up)
  int newOrient = findNeighborLow(posX, posY, 'o');
  int toTurn = (newOrient - mouseOrient + 4) % 4;

  switch(toTurn) {
    case 0:
      userCommand = USERFOR;
      break;
    case 1:
      userCommand = USERRIG;
      break;
    case 2:
      userCommand = USERINV;
      break;
    case 3:
      userCommand = USERLEF;
      break;
    default:
    //it should always be safe to go back a cell if there are errors
      userCommand = USERINV;
  }

  //update orientation:
  mouseOrient = newOrient;
  }
}

//FUNCTIONS

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
//debug parameter printHCoords subs horizontal lines with referenced indices of mazeWalls
//debug parameter printVCoords subs vertical lines with referenced indices of mazeWalls
void printWalls(bool addDist, bool printHCoords, bool printVCoords) {
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
    //row titles
    Serial.println();
    Serial.print("r:");
    Serial.print(i);
    
    //vertical walls
    //p = number of lines rendered per wall
    int p = 3;
    for(int l = 0; l < p; l++) {
      Serial.print("\t|");
      for(int q = 0; q < sizeY; q++) {
        
        //add Dist if prompted
        if((l == p/2) && addDist && !printVCoords) {
          Serial.print("   ");
          Serial.print(mazeDist[i][q]);
          }
          
         Serial.print("\t");
         
         if(mazeWalls[sizeX + q][(sizeX - 1) - i]) 
         Serial.print("|");

         //debug: prints cordinates referenced
         if(printVCoords){
         Serial.print(sizeX + q);
         Serial.print(" ");
         Serial.print((sizeX - 1) - i);
         }
         
      }
      
    Serial.println();
    }
    
    //horizontal lines
    Serial.print("\t");
    for(int h = 0; h < sizeY; h++) {
     Serial.print("+");

     //print segments:
     if(!printHCoords) {
      if(mazeWalls[h][(sizeX - 1) - i])
        Serial.print("-------");
      else
        Serial.print("       ");
     }
     else{
        //debug: prints cordinates referenced
        Serial.print(h);
        Serial.print(" ");
        Serial.print((sizeX - 1) - i);
     }
    }
    //Serial.print("\t");   
    Serial.print("+");
  }
  Serial.print("\nPrintWalls Complete.");
}


//Algorithm functions:

//returns whether a given wall is not an edge:
bool testWall(int locX, int locY, int orient) {
  //detect ajacent maze edges
  bool edgeU = locX == SIZEX - 1;
  bool edgeR = locY == SIZEX - 1;
  bool edgeD = locX == 0;
  bool edgeL = locY == 0;

  //compare to orientation:
  switch(orient) {
    case 0:
      return edgeU;
    break;
    case 1:
      return edgeR;
    break;
    case 2:
      return edgeD;
    break;
    case 3:
      return edgeL;
    break;
    default://this default is fail dangerous, may lead to endless loop if default set true or let mouse crash into wall if false. It's better to let it crash so we can correct it sooner.
      return false;
  }
}

//returns true if a given cell exists:
bool testCell(int locX, int locY) {
  return locX >= 0 && locX < SIZEX && locY >= 0 &&locY < SIZEY;
}

//sets correct wall to state passed to it:
void setMazeWall(int mouseOrient, bool wallState) {
  int indexX = posX;
  int indexY = posY;
  
  if(mouseOrient % 2 == 1)//points to vertical directions (horizontal needs no addition)
    indexX += SIZEX;//shift to correct region of array

  if(mouseOrient == 2)
    indexY--;

  if(mouseOrient == 3)
    indexX--;

    mazeWalls[indexX][indexY] = wallState;
}

//returns the requested int from a scan of neighbors
int findNeighborLow(int locX, int locY, char infoReq) {
  
//infoReq: 'o' = orientation [0 to 3]; 'v' = value [0 to 255]
int dirNeighborLow = -1;//null
int valNeighborLow = 256;//null


//for each direction, compare
//tests if each neighbor exists and if there isn't a wall between them
//!make more legible

//up
if(testCell(locX + 1, locY) && valNeighborLow > mazeDist[locX + 1][locY] && testWall(locX, locY, 0) && !mazeWalls[locX][locY + 1]) {
  dirNeighborLow = 0;
  valNeighborLow = mazeDist[locX + 1][locY];
}
//right
if(testCell(locX, locY + 1) && valNeighborLow > mazeDist[locX][locY + 1] && testWall(locX, locY, 1) && !mazeWalls[locX + SIZEX][locY]) {
  dirNeighborLow = 1;
  valNeighborLow = mazeDist[locX][locY + 1];
}
//down
if(testCell(locX - 1, locY) && valNeighborLow > mazeDist[locX - 1][locY] && testWall(locX, locY, 2) && !mazeWalls[locX][locY]) {
  dirNeighborLow = 2;
  valNeighborLow = mazeDist[locX - 1][locY];
}
//left
if(testCell(locX, locY - 1) && valNeighborLow > mazeDist[locX][locY - 1] && testWall(locX, locY, 3) && !mazeWalls[locX + SIZEX - 1][locY]) {
  dirNeighborLow = 3;
  valNeighborLow = mazeDist[locX][locY - 1];
}

switch(infoReq) {
  case 'o':
    return dirNeighborLow;
    break;
  case 'v':
    return valNeighborLow;
    break;
  default:
    return -1;
  }
}

void pushQueue(int item) {
  checkQueue[checkSize] = item;
  checkSize++;
}

int popQueue() {
  //error case:
  if(checkSize < 1)
    return -1;
    
  checkSize--;
  return checkQueue[checkSize];
}

void cellUpdate(int cell) {
  //decompose into x and y
  int locX = cell / SIZEX;
  int locY = cell % SIZEX;

    //if path is broken(all neighbors have greater distance to goal):
  if(mazeDist[locX][locY] != findNeighborLow(locX, locY, 'v') + 1) {
    //update this cell's value:
    mazeDist[locX][locY] = findNeighborLow(locX, locY, 'v') + 1;
    //then push neighbors onto checkQueue:
    //up
    if(testCell(locX, locY + 1) && !mazeWalls[locX + 1][locY]) {
      pushQueue(cell + SIZEX);
    }
    //right
    if(testCell(locX + 1, locY) && !mazeWalls[locX + SIZEX][locY]) {
      pushQueue(cell + 1);
    }
    //down
    if(testCell(locX, locY - 1) && !mazeWalls[locX][locY]) {
      pushQueue(cell - SIZEX);
    }
    //left
    if(testCell(locX - 1, locY) && !mazeWalls[locX + SIZEX - 1][locY]) {
     pushQueue(cell - 1);
    }
    
  }
}
//5/11/18

