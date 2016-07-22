char val; // variable to receive data from the serial port
int ledpin = 13; // LED connected to pin 48 (on-board LED)

void setup() {

  pinMode(ledpin, OUTPUT);  // pin 48 (on-board LED) as OUTPUT
  Serial2.begin(9600);       // start serial communication at 9600bps
}

void loop() {

  Serial2.print("speed");
  delay(100);                    // wait 100ms for next reading
} 
