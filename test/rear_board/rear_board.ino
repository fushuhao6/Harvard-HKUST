/*********************************************************************
  This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

  This example is for a 128x32 size display using I2C to communicate
  3 pins are required to interface (2 I2C and one reset)

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada  for Adafruit Industries.
  BSD license, check license.txt for more information
  All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
#define MAX_DUTY_CYCLE 90.0
#define MAX_ANGLE 140.0
#define MAX_PWM 360

#define MAIN_TIME 50

Adafruit_SSD1306 display(OLED_RESET);
const uint8_t P_LR = 13; //red led
const uint8_t BUTTON_A = 9;
const uint8_t P_PWM = 12; // PWM pin
const uint8_t communicate = 15;

// time configurations
unsigned long m_time;     // main time

uint16_t init_angle = 0;

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup()   {
  Serial.begin(9600);
  pinMode(BUTTON_A, INPUT);    // declare pushbutton as input
  pinMode(P_LR, OUTPUT);
  pinMode(P_PWM, OUTPUT);       // declare pwm pin
  
  m_time = millis();
  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(100);

  // Clear the buffer.
  display.clearDisplay();
}

bool led_status = false;
bool last_state = true;
bool state = true;

void loop() {
  if(millis() - m_time >= MAIN_TIME){
    m_time = millis();
    state = false;
    
    // clear everything before print
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

     // display the current degree and angle difference
    display.print("Degrees: ");

    int degree = 0;

    while(Serial.available()){
      degree = Serial.parseInt();
      display.print(degree);
    }

    // display.print("Angle: ");
    // display.println(abs(value - init_angle));
    display.display();

    // for the motor part 
    // analogWrite(P_PWM, get_duty_cycle(value) * MAX_PWM / 100);
    // analogWrite(P_LR, get_duty_cycle(value) * MAX_PWM / 100);
  }
}

bool button_push(uint8_t pin) {
  bool current_state = digitalRead(pin);
  if (last_state == false && current_state == true) {
    last_state = current_state;
    return true;
  }
  last_state = current_state;
  return false;
}

uint8_t get_duty_cycle(int value){
  uint16_t angle = abs(value - init_angle);
  if(angle < 3) return 0;

  // linear output for demo
//  uint8_t duty_cycle = angle * MAX_DUTY_CYCLE / MAX_ANGLE;
  
  // cubic output to motor
  uint16_t duty_cycle = angle * angle * angle * MAX_DUTY_CYCLE / (MAX_ANGLE * MAX_ANGLE * MAX_ANGLE);
  return duty_cycle < MAX_DUTY_CYCLE? duty_cycle:MAX_DUTY_CYCLE;
}


