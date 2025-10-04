#include <Arduino.h>

#define ENCA 2 //Yellow wire
#define ENCB 3 //white wire

int pos=0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  //mỗi khi cạnh lên (RISING) xuất hiện ở ENCA thì chạy readEncoder()
}

void loop() {
  Serial.println(pos);
}
//trên cạnh lên của A, nếu B = 1 thì đang quay một chiều, B = 0 thì chiều ngược lại
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
    }else{
    pos--;
 }
}