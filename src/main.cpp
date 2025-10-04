#include <Arduino.h>
//Part 3 : Drive the motor
//Motor Terminal 1 --> motor driver output1
//Motor Terminal 2 --> motor driver output2'
//power supply 12V --> motor driver Vin/GND
//Motor driver gnd --> arduino GND
//Motor driver PWMA input --> arduino pin 5
//Motor driver AIN1 input --> arduino pin 7
//Motor driver AIN2 input --> arduino pin 6
#define ENCA 2 //Yellow wire
#define ENCB 3 //white wire
#define PWM 5 
#define IN1 7
#define IN2 6

int pos=0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  //mỗi khi cạnh lên (RISING) xuất hiện ở ENCA thì chạy readEncoder()
}

void loop() {
    setMotor(1, 25, PWM, IN1, IN2); //quay tiến
    delay(200);
    Serial.println(pos);
    setMotor(-1, 25, PWM, IN1, IN2); //quay lùi
    delay(200);
    Serial.println(pos);
    setMotor(0, 0, PWM, IN1, IN2); //dừng
    delay(200);
    Serial.println(pos);
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
    if(dir==1){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }else if(dir==-1){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }else{
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
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