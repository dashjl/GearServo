//Pins
#define EnPinA 2
#define EnPinB 3
#define MotorEn 8
#define MotorA 9
#define MotorB 10
#define AngleSensor A1

//Contants
//25mm 4.4:1
//#define EnCPR 211.2
//25mm 9.7:1
//#define EnCPR 464.64
//37mm 18.75:1
#define EnCPR 1200
#define MotorDead 0
#define SerialDecimal 8
#define AngleSweep 300

//parameters

#define Cycle 10000
#define SetPointOne 120
#define SetPointTwo 240

volatile unsigned int EnPos = EnCPR / 2;
volatile unsigned int EnChA = 0;
volatile unsigned int EnChB = 0;

long CycleTime = 0;
unsigned int EnOutput = EnPos;
unsigned int SetPoint = SetPointOne;
float EnAngle = 0;
int EnTurns = 0;

//pid
double integrator = 0;
double error = 0;
double K = 1;
double P = 0.5;
double I = 0.001;
double D = 0;

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
  //Set PWM to 31250 Hz
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  //Set PWM to 3906 Hz
  //TCCR1B = TCCR1B & 0b11111000 | 0x02;
  //Set PWM to 488 Hz
  //TCCR1B = TCCR1B & 0b11111000 |0x03;
  
  //setPwmFrequency(10, 1024);
}
void loop() {
  /*Serial */
  if (millis() > CycleTime) {
    //At start of cycle, check for serial updates
    while (Serial.available() > 0) {
      char parameter = Serial.read();// get parameter byte
      float value = Serial.parseFloat();
      Serial.readStringUntil('\n');
      switch (parameter) {
        case 'k':
        case 'K':
          K = value;
          Serial.print("Param: ");
          Serial.print(parameter);
          Serial.print(" Set to: ");
          Serial.print(value,SerialDecimal);
          Serial.print(" Read as: ");
          Serial.println(K,SerialDecimal);
          break;
        case 'p':
        case 'P':
          P = value;
          Serial.print("Param: ");
          Serial.print(parameter);
          Serial.print(" Set to: ");
          Serial.print(value,SerialDecimal);
          Serial.print(" Read as: ");
          Serial.println(P,SerialDecimal);
          break;
        case 'i':
        case 'I':
          I = value;
          Serial.print("Param: ");
          Serial.print(parameter);
          Serial.print(" Set to: ");
          Serial.print(value,SerialDecimal);
          Serial.print(" Read as: ");
          Serial.println(I,SerialDecimal);
          break;
        case 'd':
        case 'D':
          D = value;
          Serial.print("Param: ");
          Serial.print(parameter);
          Serial.print(" Set to: ");
          Serial.print(value,SerialDecimal);
          Serial.print(" Read as: ");
          Serial.println(D,SerialDecimal);
          break;
        default:
          Serial.println("Invalid Statement");
          break;
      }
    }

    Serial.print("Error:");
    Serial.print(SetPoint - EnAngle);
    Serial.print(" Integrator:");
    Serial.println(integrator);
    integrator = 0;
    if (SetPoint == SetPointOne) {
      SetPoint = SetPointTwo;
    }
    else {
      SetPoint = SetPointOne;
    }
    CycleTime = millis() + Cycle;
  }
  /*Update*/
  EnOutput = EnPos;
  EnAngle = EnOutput * 360.0 / EnCPR;
  Serial.println(EnAngle);
  /*Control */
  error = SetPoint - EnAngle;
  integrator += error;
  int control = K * (error * P + integrator * I);
  if (abs(control) < MotorDead) {
    control = 0;
  }
  Motor(control);
}
// Interrupt on A changing state
void EnIsrA() {
  if (EnChB ^ EnChA) {
    EnPos++;
    if (EnPos == EnCPR) {
      EnPos = 0;
      EnTurns++;
    }
  }
  else {
    if (EnPos == 0) {
      EnPos = EnCPR;
      EnTurns--;
    }
    EnPos--;
  }
  EnChA = digitalRead(EnPinA);
}
// Interrupt on B changing state
void EnIsrB() {
  EnChB = digitalRead(EnPinB);
  if (EnChB ^ EnChA) {
    EnPos++;
    if (EnPos == EnCPR) {
      EnPos = 0;
      EnTurns++;
    }
  }
  else {
    if (EnPos == 0) {
      EnPos = EnCPR;
      EnTurns--;
    }
    EnPos--;
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


