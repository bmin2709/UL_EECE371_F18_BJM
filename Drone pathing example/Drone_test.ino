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
{1, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 1, 0, 0, 1},
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
{1, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 1, 0, 0, 1},
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
  drone_location.hori = 4;
  drone_location.vert = 7;
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
}