#include <EEPROM.h>

/*DEFINE PINS*/
/* Motor
   Blue 5V Green 0V
   Yellow EnA White EnB
   Red 12V Black 0V
   25mm 4.4:1
   EnCPR 211.2
   25mm 9.7:1
   EnCPR 464.64
   37mm 18.75:1
   EnCPR 1200
*/
#define EnPinA 2
#define EnPinB 3
#define MotorEn 8
#define MotorA 9
#define MotorB 10
/* Potentiometer
   Orange 5V Brown 0V Red Output
*/
#define AngleSensor 1

/*DEFINE PARAMETERS*/
#define SerialDecimal 4

/*ISR VARIABLES*/
volatile int EnPos = 0;
volatile byte EnChA = 0;
volatile byte EnChB = 0;

unsigned int CycleCount, CycleTime, CycleFrequency;
/*VARIABLES*/
unsigned int EnOutput = 0;
unsigned int SetPoint = 0;
float EnAngle = 0;
int EnTurns = 0;
int control;

/*CONTROLLER*/
byte select;
byte enable;

struct motor {
  double K;
  double P;
  double I;
  double D;
  double EnCPR;
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

  /* SETUP CONTROLLER
     Load select with address 0
     Next load that controller
  */
  EEPROM.get(0, select);
  LoadController();
  Serial.println(motorPID.EnCPR);
  /* SETUP PWM FREQUENCY
     Set PWM to 31250 Hz 0x01
     Set PWM to 3906 Hz 0x02
     Set PWM to 488 Hz 0x03
  */
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
}
/* Main Loop */
void loop() {
  // Serial.println(EnAngle);
  CycleCount++;
  if(millis()>CycleTime){
    CycleFrequency=CycleCount*10;
    CycleTime+=100;
    CycleCount=0;
  }
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
  while (Serial.available() > 0) {
    digitalWrite(MotorEn, LOW);
    control = 0;
    char parameter = Serial.read();// get parameter byte
    float value = Serial.parseFloat();
    Serial.readStringUntil('\n');
    switch (parameter) {
      case 'c':
      case 'C':
        if (value) {
          select = value;
          EEPROM.put(0, select);
          LoadController();
          break;
        }
        else {
          DisplayController();
        }
        break;
      case 'v':
      case 'V':
        Serial.print("SV:");
        Serial.print(SetPoint);
        Serial.print(" PV:");
        Serial.print(EnAngle);
        Serial.print(" Error:");
        Serial.println(SetPoint - EnAngle);
        break;
      case 'f':
      case 'F':
        Serial.print("Controller Frequency:");
        Serial.println(CycleFrequency);
        break;
      case '?':
      case 'h':
      case 'H':
        Serial.println("Help, h, or ?: display this menu");
        Serial.println("C: select controller 1 or 2, no arg to display current");
        Serial.println("S: Setpoint Command");
        Serial.println("V: Display SV and CV");
        Serial.println("K: controller K value");
        Serial.println("P: controller P value");
        Serial.println("I: controller I value");
        Serial.println("D: controller D value");
        Serial.println("E: encoder counter per revolution");
        break;
      case 's':
      case 'S':       
        SetPoint = value;
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
  }
  digitalWrite(MotorEn, HIGH);
}
void LoadController() {
  if (select == 1) {
    Serial.println("Selected Controller 1");
    AddrPID = 1;
  }
  else if (select == 2) {
    Serial.println("Selected Controller 2");
    AddrPID = sizeof(motor) + 1;
  }
  else {
    Serial.println("Invalid Controller");
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
  if (select == 1) {
    Serial.println("Controller 1");
  }
  else if ( select == 2) {
    Serial.println("Controller 2");
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
  Serial.println(motorPID.D, SerialDecimal);

}

// Interrupt on A changing state
void EnIsrA() {
  if (EnChB ^ EnChA) {
    EnPos++;
  }
  else {
    EnPos--;
  }
  EnChA = digitalRead(EnPinA);
}
// Interrupt on B changing state
void EnIsrB() {
  EnChB = digitalRead(EnPinB);
  if (EnChB ^ EnChA) {
    EnPos++;
  }
  else {
    EnPos--;
  }
}
