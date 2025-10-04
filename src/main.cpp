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
long prevT=0;
float eprev=0;
float eintegral=0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  //mỗi khi cạnh lên (RISING) xuất hiện ở ENCA thì chạy readEncoder()
}

void loop() {
    // set target position
    int target = 1200; 
    //PID constants
    float Kp = 1;
    float Ki = 0;
    float Kd = 0;
    // time difference
    long currT = micros();

    float deltaT = ((float)(currT-prevT))/1.0e6; //s
    prevT = currT;

    // error
    int e = pos-target;// do đấu dây, nếu chạy ko đúng thì target-pos

    //derivative
    float derivative = (e-eprev)/deltaT;

    //integral
    eintegral=eintegral+e*deltaT;

    // control signal
    float u = Kp*e + Ki*eintegral + Kd*derivative;

    // motor power
    float pwr=fabs(u);
    if(pwr>255) {
        pwr=255;
    }

    // motor direction
    int dir=1;
    if(u<0){
        dir=-1;
    }
    //signal the motor
    setMotor(dir,pwr,PWM,IN1,IN2);

    //store previous error
    eprev=e;
    //print position
    Serial.print("Target: ");
    Serial.print(pos);
    Serial.println();
    
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