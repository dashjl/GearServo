#include <EEPROM.h>

/*DEFINE PINS*/
#define EnPinA 2
#define EnPinB 3
#define MotorEn 8
#define MotorA 9
#define MotorB 10
#define AngleSensor 1

/*DEFINE PARAMETERS*/
#define MotorDead 1
#define SerialDecimal 8
#define AngleSweep 300
#define Cycle 10000
#define SetPointOne 120
#define SetPointTwo 240

/*ISR VARIABLES*/
volatile unsigned int EnPos = 0;
volatile unsigned int EnChA = 0;
volatile unsigned int EnChB = 0;

/*VARIABLES*/
long CycleTime = 0;
unsigned int EnOutput = EnPos;
unsigned int SetPoint = SetPointOne;
float EnAngle = 0;
int EnTurns = 0;
int control;

/* MOTOR INFO */
//25mm 4.4:1
//EnCPR 211.2
//25mm 9.7:1
//EnCPR 464.64
//37mm 18.75:1
//EnCPR 1200

/*CONTROLLER*/
byte select;

struct motor {
  double K;
  double P;
  double I;
  double D;
  float EnCPR;
};

int AddrPID;
double integrator = 0;
double error = 0;
motor motorPID;

/*EEPROM BLOCK
  0, select
  1, motor1
  sizeof(motor)+1, motor2
*/

void setup() {
  /*SETUP ENCODER */
  pinMode(EnPinA, INPUT);
  pinMode(EnPinB, INPUT);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, EnIsrA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, EnIsrB, CHANGE);

  /*SETUP MOTOR*/
  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);
  pinMode(MotorEn, OUTPUT);
  digitalWrite(MotorEn, HIGH);

  /*SETUP SERIAL*/
  Serial.begin (115200);
  Serial.println("Initialized");

  /*SETUP CONTROLLER*/
  EEPROM.get(0, select);
  LoadController();

  /* SETUP PWM FREQUENCY */
  //Set PWM to 31250 Hz
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  //Set PWM to 3906 Hz
  //TCCR1B = TCCR1B & 0b11111000 | 0x02;
  //Set PWM to 488 Hz
  //TCCR1B = TCCR1B & 0b11111000 |0x03;
}
void loop() {
  serial();
  inputs();
  control = controller();
  outputs(control);
}
void inputs() {
  EnOutput = EnPos;
  EnAngle = EnOutput * 360.0 / motorPID.EnCPR;
}
int controller() {
  error = SetPoint - EnAngle;
  integrator += error;
  int control = motorPID.K * (error * motorPID.P + integrator * motorPID.I);
  if (abs(control) < MotorDead) {
    control = 0;
  }
  return control;
}
void outputs(int pwm) {
  if (pwm == 0) {
    digitalWrite(MotorA, 0);
    digitalWrite(MotorB, 0);
  }
  //clamp pwm signal
  else if (pwm > 255) {
    outputs(255);
  }
  //clamp pwm signal
  else if (pwm < -255) {
    outputs(-255);
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
void serial() {
  //if (millis() > CycleTime) {
    //At start of cycle, check for serial updates
    while (Serial.available() > 0) {
      char parameter = Serial.read();// get parameter byte
      float value = Serial.parseFloat();
      Serial.readStringUntil('\n');
      switch (parameter) {
        case 'c':
        case 'C':
          DisplayController();
          break;
        case '?':
        case 'h':
        case 'H':
          Serial.println("Help, h, or ?: display this menu");
          Serial.println("C: display controller info");
          Serial.println("S: select controller 0 or 1");
          Serial.println("K: controller K value");
          Serial.println("P: controller P value");
          Serial.println("I: controller I value");
          Serial.println("D: controller D value");
          Serial.println("E: encoder counter per revolution");
          break;
        case 's':
        case 'S':
          select = value;
          EEPROM.put(0, select);
          LoadController();
          break;
        case 'e':
        case 'E':
          motorPID.EnCPR = value;
          EEPROM.put(AddrPID, motorPID);
          UpdateController(parameter, value);
          break;
        case 'k':
        case 'K':
          motorPID.K = value;
          EEPROM.put(AddrPID, motorPID);
          UpdateController(parameter, value);
          break;
        case 'p':
        case 'P':
          motorPID.P = value;
          EEPROM.put(AddrPID, motorPID);
          UpdateController(parameter, value);
          break;
        case 'i':
        case 'I':
          motorPID.I = value;
          EEPROM.put(AddrPID, motorPID);
          UpdateController(parameter, value);
          break;
        case 'd':
        case 'D':
          motorPID.D = value;
          EEPROM.put(AddrPID, motorPID);
          UpdateController(parameter, value);
          break;
        default:
          Serial.println("Invalid Statement");
          break;
      }
   // }

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
}
void LoadController() {
  if (select) {
    Serial.println("Selected Controller 1");
    AddrPID = sizeof(motor) + 1;
  }
  else {
    Serial.println("Selected Controller 0");
    AddrPID = 1;
  }
  EEPROM.get( AddrPID, motorPID );
}
void UpdateController(char parameter, float value) {
  Serial.print("Param: ");
  Serial.print(parameter);
  Serial.print(" Set to: ");
  Serial.println(value, SerialDecimal);
}
void DisplayController() {
  if (select) {
    Serial.println("Controller 1");
  }
  else {
    Serial.println("Controller 0");
  }
  Serial.print("Encoder CPR:");
  Serial.println(motorPID.EnCPR, 2);
  Serial.print("K:");
  Serial.print(motorPID.K, SerialDecimal);
  Serial.print(" P:");
  Serial.print(motorPID.P, SerialDecimal);
  Serial.print(" I:");
  Serial.print(motorPID.I, SerialDecimal);
  Serial.print(" D:");
  Serial.print(motorPID.D, SerialDecimal);

}

// Interrupt on A changing state
void EnIsrA() {
  if (EnChB ^ EnChA) {
    EnPos++;
    if (EnPos == motorPID.EnCPR) {
      EnPos = 0;
      EnTurns++;
    }
  }
  else {
    if (EnPos == 0) {
      EnPos = motorPID.EnCPR;
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
    if (EnPos == motorPID.EnCPR) {
      EnPos = 0;
      EnTurns++;
    }
  }
  else {
    if (EnPos == 0) {
      EnPos = motorPID.EnCPR;
      EnTurns--;
    }
    EnPos--;
  }
}
