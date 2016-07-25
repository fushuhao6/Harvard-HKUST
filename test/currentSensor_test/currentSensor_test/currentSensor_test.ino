// currentSensor_test.ino
// working as of 25 July 2016

const uint8_t currentSensorPin = A8;
double currentSensorRaw = 0;
double current = 0;

void setup() {
  pinMode(currentSensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  analogReadResolution(12);
  currentSensorRaw = (double) analogRead(currentSensorPin);
  
  // VOLTAGE DIVIDER TAKES 0.66 of the output voltage from motor controller (between 0 and 5)
  // in order to center it around 1.65V. Now is 43.56mV/A or 54.0408 ~54 per amp
  // 0        2047      4095    value in currentSensorRaw
  // 0V      1.65V     3.3V     corresponds to these voltages raw, after divider
  
  current = currentSensorRaw - 2047;
  
  // Now centered around 0 so we have
  //  -2047       0       2047
  // -37.88A      0      37.88A
  // need to divide raw value by 54.08 to get amperage

  current = current / 54.08; // this is a constant value we need to get actual current
  
  Serial.print("\nCurrent: ");  
  Serial.print(current, 3);
  Serial.print("A");
  
  delay(100);
}
