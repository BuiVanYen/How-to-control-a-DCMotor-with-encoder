#include <Arduino.h>

#define ENCA 2 //Yellow wire
#define ENCB 3 //white wire

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
}

void loop() {
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  
  Serial.print(a*5);
  Serial.print(" ");
  Serial.println(b*5);
  Serial.println("-----");
}