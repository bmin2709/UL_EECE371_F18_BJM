float matrix1[5][5] = {
    {1430, 234, 436, 435, 1231},
    {43214, 4326, 56, 48, 15},
    {144, 96, 5986, 48, 15},
    {134, 6, 56, 48, 15},
    {14, 689, 56, 4890, 15}
};

float matrix2 [5][5] = {
    {321310, 21311, 231234, 5, 2},
    {132124, 6, 56, 48, 15},
    {13134, 36, 56, 543548, 15},
    {1214, 64534, 56, 48, 15},
    {143154, 7715, 4, 8433, 9}
};

int matrix3 [5][5];
int col = 5;
int row = 5;
int time = micros();
int start_time, finish_time, sum, total_time;


void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);

}

void loop() {


start_time = micros();
  
for ( int i = 0; i < 5; i++) {
      for (int j = 0; j < 5; j++) {
        for (int k = 0; k < 5; k++) {
          sum = sum + matrix1[i][k]*matrix2[k][j];
        }
 
        matrix3[i][j] = sum;
        sum = 0;
      }
    }

finish_time = micros();

total_time = finish_time - start_time;

Serial.println(total_time);
Serial.print("");

}
