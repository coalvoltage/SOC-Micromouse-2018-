int randomDecision() {
  int options = -1;
if(wallLeft)
  options++;
if(wallFront)
  options++;
if(wallRight)
  options++;

int decision = random(0, 100) % options;
  
if(wallLeft && decision == 0)
 decision--; 
else
  return USERLEF;

if(wallFront && decision == 0)
 decision--; 
else
  return USERFOR;
  
if(wallRight && decision == 0)
 decision--; 
else
  return USERLEF;

return USERINV;
}
