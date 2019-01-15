#include <Ultrasonic.h>
#define MAXD 230
#define SPOOKD 150
#define SSPOOKD 80
Ultrasonic sensor1(12,13);//front
Ultrasonic sensor2(2,3);//left
Ultrasonic sensor3(6,7);//right

int sonicControl(){
  //legend for int out
  //0=front/nothing
  //1=left
  //-1=right
  //4=back
  int m1,m2,m3;
  bool front=true;
  bool left=true;
  bool right=true;
  
  m1=sensor1.read();
  if(m1>MAXD){m1=0;}
  if((m1<SPOOKD)&&(m1!=0)){front = false;}  //If less then spook then don't go forward
  
  m2=sensor2.read();
  if(m2>MAXD){m2=0;}
  if((m2<SPOOKD)&&(m2!=0)){left = false;}   //Same
  
  m3=sensor3.read();
  if(m3>MAXD){m3=0;}
  if((m3<SPOOKD)&&(m3!=0)){right = false;}  //Same

  //drifting/object appearance conditions
  if((m2<SSPOOKD)&&(m2!=0)&&(right == true)){ return -1; }  //for drifting left/object appearing on left, go right
  if((m3<SSPOOKD)&&(m3!=0)&&(left  == true)){ return 1; }   //for drifting right/object appearing on right, go left

  //if passes drifting/object apperance take these conditionals
  if (front == true){return 0;} //go front if available
  if ((front == false)&&(left == true)){return 1;}  //go left of front not availible
  if ((front == false)&&(left == false)&&(right == true)){return -1;} //go right
  if ((front == false)&&(left==false)&&(right==false)){return 4;} //go backwards
  else return 4;  //all else fails, go backwards until out of mess.
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  Serial.println(sonicControl()); 
}
