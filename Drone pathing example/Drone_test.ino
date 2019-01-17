#include <FastLED.h>
#define DATA_PIN 2
#define Size 70
#define NUM_LEDS Size

CRGB leds[NUM_LEDS];

//struct for drone position
typedef struct drone{
  int vert;
  int hori;
}drone_t;

drone_t drone_location;
int rows = 10;
int col = 7;

//1d array for LEDs
int flatarray[70];

//2d Array for mapping
int room[10][7] = {
{1, 1, 1, 1, 1, 1, 1},
{1, 2, 0, 0, 0, 0, 1},
{1, 0, 0, 1, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 1},
{1, 0, 1, 0, 1, 0, 1},
{1, 0, 0, 0, 1, 0, 1},
{1, 0, 0, 0, 0, 1, 1},
{1, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 1},
{1, 1, 1, 1, 1, 1, 1}
};


int sonicControl(drone_t drone)
{
  //legend for int out
  //0=front/nothing
  //1=left
  //-1=right
  //4=back
  bool front=true;
  bool left=true;
  bool right=true;
  int path[10][7] = {
{1, 1, 1, 1, 1, 1, 1},
{1, 2, 0, 0, 0, 0, 1},
{1, 0, 0, 1, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 1},
{1, 0, 1, 0, 1, 0, 1},
{1, 0, 0, 0, 1, 0, 1},
{1, 0, 0, 0, 0, 1, 1},
{1, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 1},
{1, 1, 1, 1, 1, 1, 1}
};

  if(path[(drone.vert)-1][(drone.hori)]==0){front = true;}  //If less then spook then don't go forward
  else {front = false;}

  if(path[(drone.vert)][(drone.hori-1)]==0){left = true;}   //Same
  else {left = false;}
  
  if(path[(drone.vert)][(drone.hori+1)]==0){right = true;}  //Same
  else {right = false;}

  if (front == true){return 0;} //go front if available
  if ((front == false)&&(left == true)){return 1;}  //go left of front not availible
  if ((front == false)&&(left == false)&&(right == true)){return -1;} //go right
  if ((front == false)&&(left==false)&&(right==false)){return 4;} //go backwards
  else return 4;  //all else fails, go backwards until out of mess.
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  drone_location.hori = 5;
  drone_location.vert = 8;
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
}

void loop() 
{//increments drone movement based on return value of sonicControl
  if (sonicControl(drone_location)==1){
    room[drone_location.vert][drone_location.hori] = 0;
    drone_location.hori -= 1;
    room[drone_location.vert][drone_location.hori] = 8;
  }
  else if (sonicControl(drone_location)==0){
    room[drone_location.vert][drone_location.hori] = 0;
    drone_location.vert -= 1;
    room[drone_location.vert][drone_location.hori] = 8;
  }
  else if (sonicControl(drone_location)==-1){
    room[drone_location.vert][drone_location.hori] = 0;
    drone_location.hori += 1;
    room[drone_location.vert][drone_location.hori] = 8;
  }
  else{
    room[drone_location.vert][drone_location.hori] = 0;
    drone_location.vert += 1;
    room[drone_location.vert][drone_location.hori] = 8; 
  }

  for (int i=0; i<rows; i++){
    for (int j = 0; j<col; j++){
      flatarray[(i*col)+j] = room[i][j];
    }
  }
  arrayPrint(flatarray);
  delay(1000);
}

void arrayPrint(int path[]) {
  int data;
  //0 == No Obstruction |Blank
  //1 == Obstruction    |Red
  //2 == Goal           |Green
  //8 == Drone          |White

  for (int i = 0; i < Size; i++) {
    data = path[i];
    Serial.println(data); 

    if (data == 1) {  //Make the LED red
      leds[i].setRGB( 15, 0, 0);
      //Serial.println("Red");
    }
    
    if (data == 2) {  //Make the LED Green
      leds[i].setRGB( 0, 15, 0);
      //Serial.println("Green");
    }
    
    if (data == 8) {  //Make the LED White
      leds[i].setRGB( 10, 10, 10);
      //Serial.println("White");
    }
    
  }
  FastLED.show();
  FastLED.clear();//Clears the LED to be updated again
}