#include <IRremote.h>
#include <Servo.h>
//IR contro
int RECV_PIN = 12;
unsigned long Key; 
#define IR_Go 0x00ff629d
#define IR_Back 0x00ffa857
#define IR_Left 0x00ff22dd
#define IR_Right 0x00ffc23d
#define IR_Stop 0x00ff02fd
#define IR_State 0x00ff42bd
IRrecv irrecv(RECV_PIN);
decode_results results;
//Ultrasonic
int Echo_Pin = A0;  
int Trig_Pin = A1;  
volatile int D_mix;
volatile int D_mid;
volatile int D_max;
volatile int Front_Distance;
volatile int Left_Distance;
volatile int Right_Distance;
volatile int Right_IR_Value;
volatile int Left_IR_Value;
//motor and servo control
Servo myservo;
#define Lpwm_pin 5          
#define Rpwm_pin 6           
int pinLB = 2;               
int pinLF = 4;               
int pinRB = 7;               
int pinRF = 8;              
bool operationFlag = false;  
int motorState = 0;          
typedef struct task {
  int state;
  unsigned long period;
  unsigned long elapsedTime;
  int (*TickFct)(int);
} task;

const unsigned short tasksNum = 4;
task tasks[tasksNum];

enum SM1_States { SM1_INIT,on1};
int SM1_Tick(int state1) {
  switch (state1) {  
    case SM1_INIT:
      state1 = on1;
      break;
    case on1:
      state1 = on1;
      break;
    default:
      break;
  }
  switch (state1) {  
    case SM1_INIT:
      state1 = on1;
      break;
    case on1:
      if (irrecv.decode(&results)) {  
        Key = results.value;
        irrecv.resume(); 
      }
      break;
    default:
      break;
  }
  return state1;
}

enum SM2_States { SM2_INIT, on2};
int SM2_Tick(int state2) {
  switch (state2) {  
    case SM2_INIT:
      state2 = on2;
      break;
    case on2:
      state2 = on2;
      break;
    default:
      break;
  }
  switch (state2) { 
    case SM2_INIT:
      state2 = on2;
      break;
    case on2:
      if (Key == IR_State) {
        operationFlag = !operationFlag;
        if (operationFlag == true) {
          Serial.println("Auto");
        }
        else{
          Serial.println("Manuel");
          motorState = 0; 
        }
        Serial.println("IR Receive:  *");
      } 
      else{
        if (operationFlag == false) { 
          switch (Key) {
            case IR_Go:
              motorState = 1;  
              Serial.println("IR Receive:  Go");
              break;
            case IR_Back:
              motorState = 2;  
              Serial.println("IR Receive:  Back");
              break;
            case IR_Left:
              motorState = 3;  
              Serial.println("IR Receive:  Left");
              break;
            case IR_Right:
              motorState = 4;  
              Serial.println("IR Receive:  Righ");
              break;
            case IR_Stop:
              motorState = 0;  
              Serial.println("IR Receive:  stop");
              break;
            default:
              break;
          }
        }
      }
      break;
    default:
      break;
  }
  Key = 0;  
  return state2;
}
float checkdistance() {
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  float distance = pulseIn(Echo_Pin, HIGH) / 58.00;
  delay(10);
  return distance;
}
enum SM3_States { SM3_INIT,on3};
int SM3_Tick(int state3) {
  switch (state3) {  
    case SM3_INIT:
      state3 = on3;
      break;
    case on3:
      state3 = on3;
      break;
    default:
      break;
  }
  switch (state3) {  
    case SM3_INIT:
      state3 = on3;
      break;
    case on3:
      if (operationFlag == true) {
        Front_Distance = checkdistance();                     
        Serial.println(Front_Distance);
        if ((Front_Distance < 35) && (Front_Distance > 0)) {  
          motorState = 0;
          delay(100);
          myservo.write(180);
          delay(500);
          Left_Distance = checkdistance();  
          delay(100);
          myservo.write(0);
          delay(500);
          Right_Distance = checkdistance();  
          delay(100);
          if (Left_Distance > Right_Distance) {  
            motorState = 3;  
            myservo.write(90);
            delay(100); 
          } 
          else {           
            motorState = 4;  
            myservo.write(90);
            delay(100); 
          }
        } 
        else{           
          motorState = 1;  
        }
      }
      break;
    default:
      break;
  }

  return state3;
}
enum SM4_States {SM4_INIT,on4,stop = 0,up,back,left,right};
int SM4_Tick(int state4) {
  switch (state4) {  
    case SM4_INIT:
      state4 = on4;
      break;
    case on4:
      state4 = on4;
      break;
    default:
      break;
  }
  switch (state4) { 
    case SM4_INIT:
      state4 = on4;
      break;
    case on4:
      switch (motorState) {
        case stop:
          digitalWrite(pinRB, HIGH);
          digitalWrite(pinRF, HIGH);
          digitalWrite(pinLB, HIGH);
          digitalWrite(pinLF, HIGH);
          break;
        case up:
          digitalWrite(pinRB, HIGH);
          digitalWrite(pinRF, LOW);
          digitalWrite(pinLB, HIGH);
          digitalWrite(pinLF, LOW);
          analogWrite(Lpwm_pin, 100);
          analogWrite(Rpwm_pin, 100);
          break;
        case back:
          digitalWrite(pinRB, LOW);
          digitalWrite(pinRF, HIGH);
          digitalWrite(pinLB, LOW);
          digitalWrite(pinLF, HIGH);
          analogWrite(Lpwm_pin, 100);
          analogWrite(Rpwm_pin, 100);
          break;
        case left:
          digitalWrite(pinRB, HIGH);
          digitalWrite(pinRF, LOW);
          digitalWrite(pinLB, LOW);
          digitalWrite(pinLF, HIGH);
          analogWrite(Lpwm_pin, 100);
          analogWrite(Rpwm_pin, 100);
          break;
        case right:
          digitalWrite(pinRB, LOW);
          digitalWrite(pinRF, HIGH);
          digitalWrite(pinLB, HIGH);
          digitalWrite(pinLF, LOW);
          analogWrite(Lpwm_pin, 100);
          analogWrite(Rpwm_pin, 100);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  return state4;
}
void setup() {
  unsigned char i = 0;
  tasks[i].state = SM1_INIT;
  tasks[i].period = 100;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &SM1_Tick;
  i++;
  tasks[i].state = SM2_INIT;
  tasks[i].period = 100;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &SM2_Tick;
  i++;
  tasks[i].state = SM3_INIT;
  tasks[i].period = 200;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &SM3_Tick;
  i++;
  tasks[i].state = SM4_INIT;
  tasks[i].period = 100;
  tasks[i].elapsedTime = 0;
  tasks[i].TickFct = &SM4_Tick;
  Serial.begin(9600);
  myservo.attach(A2);
  D_mix = 10;
  D_mid = 20;
  D_max = 100;
  Front_Distance = 0;
  Left_Distance = 0;
  Right_Distance = 0;
  myservo.write(90);
  pinMode(pinLB, OUTPUT);     
  pinMode(pinLF, OUTPUT);    
  pinMode(pinRB, OUTPUT);     
  pinMode(pinRF, OUTPUT);    
  pinMode(Lpwm_pin, OUTPUT);  
  pinMode(Rpwm_pin, OUTPUT);  
  pinMode(Trig_Pin, OUTPUT);
  pinMode(Echo_Pin, INPUT);
  irrecv.enableIRIn();        
  Serial.begin(9600);         
}

void loop() {
  unsigned char i;
  for (i = 0; i < tasksNum; ++i) {
    if ((millis() - tasks[i].elapsedTime) >= tasks[i].period) {
      tasks[i].state = tasks[i].TickFct(tasks[i].state);
      tasks[i].elapsedTime = millis();  
    }
  }
}
