#define EnPinA 2
#define EnPinB 3
#define MotorEn 8
#define MotorA 6
#define MotorB 9

#define EnCPR 1200

#define Kp 0.9
#define Ki 0
#define Kd 0
#define Cycle 2000
#define MotorDead 10

volatile unsigned int EnPos = EnCPR/2;
volatile unsigned int EnChA = 0;
volatile unsigned int EnChB = 0;

long CycleTime = 0;
unsigned int EnOutput = EnPos;
unsigned int SetPoint = 90;
float EnAngle = 0;
int EnTurns = 0;


void setup() {
  pinMode(EnPinA, INPUT);
  pinMode(EnPinB, INPUT);
  pinMode(MotorEn, OUTPUT);
  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, EnIsrA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, EnIsrB, CHANGE);
  // set up the Serial Connection
  Serial.begin (115200);
  Serial.println("Initialized");
  digitalWrite(MotorEn, HIGH);
}
void loop() {
   /*Serial */
  if (millis()>CycleTime) {
    Serial.print("SetPoint:");
    Serial.print(SetPoint);
    Serial.print(" ProcessControl:");
    Serial.println(EnAngle);
    if (SetPoint == 90) {
      SetPoint = 270;
    }
    else {
      SetPoint = 90;
    }
    CycleTime=millis()+Cycle;
  }
  /*Update*/
  EnOutput = EnPos;
  //EnAngle = EnOutput * 0.3;
  EnAngle = EnOutput * 360.0 / EnCPR;
  /*Control */
  float error = SetPoint - EnAngle;
  int control = error * Kp;
  if (abs(control) < MotorDead) {
    control = 0;
  }
  Motor(control);

}
// Interrupt on A changing state
void EnIsrA() {
  if (EnChB ^ EnChA) {
    if (EnPos == EnCPR) {
      EnPos = 0;
      EnTurns++;
    }
    else {
      EnPos++;
    }
  }
  else {
    if (EnPos == 0) {
      EnPos = EnCPR;
      EnTurns--;
    }
    else {
      EnPos--;
    }
  }
  EnChA = digitalRead(EnPinA);
}
// Interrupt on B changing state
void EnIsrB() {
  EnChB = digitalRead(EnPinB);
  if (EnChB ^ EnChA) {
    if (EnPos == EnCPR) {
      EnPos = 0;
      EnTurns++;
    }
    else {
      EnPos++;
    }
  }
  else {
    if (EnPos == 0) {
      EnPos = EnCPR;
      EnTurns--;
    }
    else {
      EnPos--;
    }
  }
}
void Motor(int pwm) {
  if (pwm == 0) {
    digitalWrite(MotorA, 0);
    digitalWrite(MotorB, 0);
  }
  //clamp pwm signal
  else if (pwm > 255) {
    Motor(255);
  }
  //clamp pwm signal
  else if (pwm < -255) {
    Motor(-255);
  }
  else if (pwm > 0) {
    digitalWrite(MotorB, 0);
    analogWrite(MotorA, pwm);
  }

  else {
    digitalWrite(MotorA, 0);
    analogWrite(MotorB, -pwm);
  }
}

