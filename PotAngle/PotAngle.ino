#define AngleSensor 1
#define AngleSweep 300
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println( readVcc(), DEC );
  delay(1000);
}
int readAngle()
{
   //Read the raw sensor value
    int sensor_value = analogRead(AngleSensor);
    //Convert the sensor reading to degrees and return that value
    float voltage = (float)sensor_value * 5.0 / 1023; 
    float degrees = (voltage * AngleSweep) / 5.0; 
    return degrees;
}
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}
