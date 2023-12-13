#include <util/atomic.h>
#include <Servo.h>

//WHEELS 
#define ENCA 2 // Left wheel encoder A
#define ENCB 4 // Left wheel encoder B
#define PWM 6 // Left wheel PWM
#define IN2 7 // Left wheel IN2
#define IN1 8 // Left wheel IN1

#define RENCA 3 // right wheel encoder A
#define RENCB 5 // right wheel encoder B
#define RPWM 11 // right wheel PWM
#define RIN2 10 // right wheel IN2
#define RIN1 9 // right wheel IN1

//ULTRASONIC SENSOR
long rduration, fduration, rcm, fcm;
const int rpingPin = 12; // right ultrasonic sensor ping pin
const int fpingPin = 13; // front ultrasonic sensor ping pin
int usp;
int fusp;

//PARAMETERS FOR PID
volatile int posi = 0;
int clearposi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
volatile int rposi = 0;

// The task from which to be performed
int task=1;

// servo motor initialization
Servo bottom_servo;
Servo top_servo;
int angle;

void setup() {
  // put your setup code here, to run once:
//  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(RENCA,INPUT);
  pinMode(RENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(RPWM,OUTPUT);
  pinMode(RIN1,OUTPUT);
  pinMode(RIN2,OUTPUT);
  //  interrupts to be detected from the encoders
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(RENCA),RreadEncoder,RISING);

  //  servo motor signal pins initialization
  bottom_servo.attach(1);
  top_servo.attach(0);

  // initial servo position
  bottom_servo.write(0);
  top_servo.write(90);
}

void loop() {
  //  tasks to be performed
  performance(task);
}


void performance(int task){
  
  switch (task) {
    case 1:
//    rotate 180 degrees
      rotate(1300);
      break;
    case 2:
//    follow the right wall with a safe distance of 18cm
//    till it reaches 20cm in front wall
      wall_follower(18, 20);
      break;
    case 3:
//    rotate 90 degrees
       if(clearposi == 1){
        posi = 0;
        clearposi=0;
      }
      rotate(680);
      break;
    case 4:
//    follow the right wall with a safe distance of 18cm
//    till it reaches 14cm in front wall
      wall_follower(18, 14);
      break;
    case 5:
//    automatically aligns itself towards the right wall
//    with a distance of 18cm.
      auto_adjust(18);
      break;
    case 6:
//    actuates the robot arm for the task 3
      actuate_arm();
      break;
    case 7:
      if(clearposi == 1){
        posi = 0;
        clearposi=0;
      }
//    rotate 90 degrees
      rotate(680);
      break;
    case 8:
//    automatically aligns itself towards the right wall
//    with a distance of 18cm.
      wall_follower(18, 20);
      break;
    case 9:
      if(clearposi == 1){
        posi = 0;
        clearposi=0;
      }
//    rotate 90 degrees
      rotate(680);
      break;
    case 10:
//    automatically aligns itself towards the right wall
//    with a distance of 18cm.
      wall_follower(18, 20);
      break;
    case 11:
      if(clearposi == 1){
        posi = 0;
        clearposi=0;
      }
//    rotate 90 degrees
      rotate(-680);
      break;
     case 12:
//     adjusts itself to move to a safe distance of 20 cm in
//     front of the robot
      front_auto_adjust(20);
      break;
    default:
      break;
  }
}

//function to move the robot arm
void actuate_arm() {

//  forward actuation
  for(int i=0; i<=70; i++){
    bottom_servo.write(i);
    int angle = 90-i;
    if(angle<=25){
      top_servo.write(25);
    }
    else{
      top_servo.write(angle);
    }
    delay(50);
  }

  angle = 25;
// backward actuation
  for(int i=70; i>=0; i--){
    bottom_servo.write(i);
    angle = angle+1;
    if(angle<=90){
      top_servo.write(angle);
    }
    else{
      top_servo.write(90);
    }
    delay(50);
  }
  task++;
}

//rotates the robot up the given set point
void rotate(int target){
  float kp = 4.9;
  float kd = 0.25;
  float ki = 0.00015;
  
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // error
  int e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  
  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);
  setMotor(dir,pwr,RPWM,RIN1,RIN2);

  // store previous error
  eprev = e;

  if(pos>=target-3 && pos<=target+3){
    task=task+1;
    setMotor(0,pwr,PWM,IN1,IN2);
    setMotor(0,pwr,RPWM,RIN1,RIN2);
    for(int i=0;i<=20;i++){
      distances();
      delay(10);
    }
  }
}

//sets motor direction, speed
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

//reads left wheel encoder pulses
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
//reads right wheel encoder pulses
void RreadEncoder(int tar){
  int r = digitalRead(RENCB);
  if(r > 0){
    rposi++;
  }
  else{
    rposi--;
  }

}

//calculate distances from right and front
void distances(){
  pinMode(rpingPin, OUTPUT);
  digitalWrite(rpingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rpingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(rpingPin, LOW);
  
  pinMode(rpingPin, INPUT);
  rduration = pulseIn(rpingPin, HIGH);

  pinMode(fpingPin, OUTPUT);
  digitalWrite(fpingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(fpingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(fpingPin, LOW);
  
  pinMode(fpingPin, INPUT);
  fduration = pulseIn(fpingPin, HIGH);

  usp = rduration/29/2;
  fusp = fduration/29/2;
}


//move with a safe distance of 'target'cm and 
//stop at a safe distance at 'ftarget'cm infront
void wall_follower(int target, int ftarget){
  distances();
  // PID constants
  float kp = 14.14;
  float kd = 0.875;
  float ki = 0.0035;
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = target - usp;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  eprev = e;

 if(usp>=target-3 && usp<=target+3){
    setMotor(-1,255,PWM,IN1,IN2);
    setMotor(1,255,RPWM,RIN1,RIN2);
  }
  else{
    setMotor(dir,pwr,PWM,IN1,IN2);
    setMotor(dir,pwr,RPWM,RIN1,RIN2);
  }

  if(fusp<ftarget){
    setMotor(0,0,PWM,IN1,IN2);
    setMotor(0,0,RPWM,RIN1,RIN2);
    task = task+1;
    clearposi = 1;
    delay(100);
  }
}

//adjust its front distance at 'target'cms
void front_auto_adjust(int target){
  distances();

  // PID constants
  float kp = 25.14;
  float kd = 0.425;
  float ki = 0.00015;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = target - fusp;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

 // signal the motor
    setMotor(dir,pwr,PWM,IN1,IN2);
    setMotor(-dir,pwr,RPWM,RIN1,RIN2);
    
  // store previous error
  eprev = e;

  if(target==fusp){
    setMotor(0,pwr,PWM,IN1,IN2);
    setMotor(0,pwr,RPWM,RIN1,RIN2);
    task++;
  }
}

//adjust its distance to right at 'target'cms
void auto_adjust(int target){
  distances();

  // PID constants
  float kp = 20.14;
  float kd = 0.25;
  float ki = 0.00015;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = target - usp;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

 // signal the motor
    setMotor(dir,pwr,PWM,IN1,IN2);
    setMotor(dir,pwr,RPWM,RIN1,RIN2);
    
  // store previous error
  eprev = e;

  if(target==usp){
    setMotor(0,pwr,PWM,IN1,IN2);
    setMotor(0,pwr,RPWM,RIN1,RIN2);
    task++;
  }
}
