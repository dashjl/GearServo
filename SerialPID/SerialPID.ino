
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
  delay(2000);
  Serial.println("Initialized");
  EEPROM.get(0, select);
  LoadController();


}

void loop() {
  while (Serial.available() > 0) {
    char parameter = Serial.read();// get parameter byte
    float value = Serial.parseFloat();
    Serial.readStringUntil('\n');
    switch (parameter) {
      case '?':
        DisplayController();
        break;
      case 's':
      case 'S':
        select = value;
        EEPROM.put(0, select);
        LoadController();
        break;
      case 'e':
      case 'E':
        motorPID.EnCPR=value;
        EEPROM.put(AddrPID,motorPID);
        UpdateParameter(parameter,value);
        break;
      case 'k':
      case 'K':
        motorPID.K = value;
        EEPROM.put(AddrPID, motorPID);
        UpdateParameter(parameter, value);
        break;
      case 'p':
      case 'P':
        motorPID.P = value;
        EEPROM.put(AddrPID, motorPID);
        UpdateParameter(parameter, value);
        break;
      case 'i':
      case 'I':
        motorPID.I = value;
        EEPROM.put(AddrPID, motorPID);
        UpdateParameter(parameter, value);
        break;
      case 'd':
      case 'D':
        motorPID.D = value;
        EEPROM.put(AddrPID, motorPID);
        UpdateParameter(parameter, value);
        break;
      default:
        Serial.println("Invalid Statement");
        break;
    }
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
void UpdateParameter(char parameter, float value) {
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
  Serial.println(motorPID.EnCPR,SerialDecimal);  
  Serial.print("K:");
  Serial.print(motorPID.K, SerialDecimal);
  Serial.print(" P:");
  Serial.print(motorPID.P, SerialDecimal);
  Serial.print(" I:");
  Serial.print(motorPID.I, SerialDecimal);
  Serial.print(" D:");
  Serial.println(motorPID.D, SerialDecimal);
  
}

