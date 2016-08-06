// currentSensor_test.ino
// working as of 25 July 2016

const uint8_t currentSensorPin = A0;
double currentSensorRaw = 0;
double current = 0;

void setup() {
  pinMode(currentSensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  analogReadResolution(12);
  currentSensorRaw = (double) analogRead(A0);
  
  // New circuit is just 1k resistor series a 3.3V Zener Diode
  // the zero current value is 875
  // need to divide result by 200 to get actual current
  
  current = (currentSensorRaw - 2425)/250;
  
  Serial.print("\nCurrent: ");  
  Serial.print(current, 2);
  Serial.print(" A      ");
  Serial.print("Raw Value: ");
  Serial.print(currentSensorRaw, 4);
  
  delay(100);
}
