
#include <EEPROM.h>
#define SerialDecimal 8
byte select;

struct motor {
  double K;
  double P;
  double I;
  double D;
  float EnCPR;
};

int AddrPID;
motor motorPID;

/*EEPROM BLOCK
  0, select
  1, motor1
  sizeof(motor)+1, motor2
*/


void setup() {
  Serial.begin(115200);
  Serial.println("Initialized");
  EEPROM.get(0, select);
  if (select) {
    AddrPID = sizeof(motor) + 1;
  }
  else {
    AddrPID = 1;
  }
  EEPROM.get( AddrPID, motorPID );
}

void loop() {
  while (Serial.available() > 0) {
    char parameter = Serial.read();// get parameter byte
    float value = Serial.parseFloat();
    Serial.readStringUntil('\n');
    switch (parameter) {
      case '?':
        Serial.println(sizeof(motorPID));
        Serial.print("K:");
        Serial.print(motorPID.K, SerialDecimal);
        Serial.print(" P:");
        Serial.print(motorPID.P, SerialDecimal);
        Serial.print(" I:");
        Serial.print(motorPID.I, SerialDecimal);
        Serial.print(" D:");
        Serial.println(motorPID.D, SerialDecimal);
        break;
      case 'k':
      case 'K':
        motorPID.K = value;
        EEPROM.put(AddrPID, motorPID);
        Serial.print("Param: ");
        Serial.print(parameter);
        Serial.print(" Set to: ");
        Serial.println(value, SerialDecimal);
        break;
      case 'p':
      case 'P':
        motorPID.P = value;
        EEPROM.put(AddrPID, motorPID);
        Serial.print("Param: ");
        Serial.print(parameter);
        Serial.print(" Set to: ");
        Serial.println(value, SerialDecimal);
        break;
      case 'i':
      case 'I':
        motorPID.I = value;
        EEPROM.put(AddrPID, motorPID);
        Serial.print("Param: ");
        Serial.print(parameter);
        Serial.print(" Set to: ");
        Serial.println(value, SerialDecimal);
        break;
      case 'd':
      case 'D':
        motorPID.D = value;
        EEPROM.put(AddrPID, motorPID);
        Serial.print("Param: ");
        Serial.print(parameter);
        Serial.print(" Set to: ");
        Serial.println(value, SerialDecimal);
        break;
      default:
        Serial.println("Invalid Statement");
        break;
    }
  }
}
