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
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AS5145.h>
#include <string.h>

#define OLED_RESET 4
#define MAIN_TIME 50
#define MAX_DUTY_CYCLE 90.0
#define MAX_ANGLE 70.0
#define MAX_PWM 255
#define ANGLE_THRESHOLD 3
#define BREAK_THRESHOLD 20


Adafruit_SSD1306 display(OLED_RESET);
AS5145 encoder(20,21,5,6);      // data, clock, chip select, program input.
File logfile;


// pin configuration
const uint8_t P_LR = 13; //red led
const uint8_t BUTTON_A = 9;
const uint8_t P_PWMH = 12; // PWM pin
const uint8_t P_PWML = 11; // PWM pin
const uint8_t P_DIR = 10;  // direction pin, digital
const uint8_t Temp1 = 14;           // temp sensor, ADC, need to init
const uint8_t P_CurSenor = 18;      // current sensor, ADC, no need to init
const uint8_t P_VolSenor = 19;      // voltage sensor, ADC, no need to init
const uint8_t cardSelect = 4;       // SD card CS pin

// time configurations
unsigned long m_time;     // main time
unsigned long init_time;  // for sd card to store time

// angle and speed values
uint16_t abs_angle[2] = {0};
uint32_t measure_time[2] = {0};
float m_speed[2] = {0};

// temp config
int temp1 = 0;


#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif



void setup()   {
  Serial.begin(115200);          // communicate with Arduino Due
  pinMode(BUTTON_A, INPUT);    // declare pushbutton as input
  pinMode(P_LR, OUTPUT);       // declare red led
  pinMode(P_PWMH, OUTPUT);      // declare pwm pin
  pinMode(P_PWML, OUTPUT);      // declare pwm pin
  
  
  m_time = millis();

  
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    error(2);
  }
  char filename[10];
  strcpy(filename, "data.txt");
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    error(3);
  }
  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(200);

  // Clear the buffer.
  display.clearDisplay();
}

bool led_state = false;
bool last_state = true;
uint8_t print_count = 0;

/************************** /
/ Rear board main function  /
/ Deals with speed encoder, /
/ temp sensor, current,     /
/ voltage sensor,           /
/ PWM generation,           /
/ communication with front, /
/ **************************/
void loop() {
  if(millis() - m_time >= MAIN_TIME){
    m_time = millis();

    // measure temp (may have more temp sensors, and move this part to feather)
    temp1 = analogRead(Temp1);
    temp1 = (temp1 * 3300 / 1023.0 - 500) / 10;

    // current sensor and voltage sensor (have not implemented calculation)
    uint16_t current = analogRead(P_CurSenor);
    uint16_t voltage = analogRead(P_VolSenor);


    // get data from Arduino Due
    uint8_t pwm = 0;
    uint16_t break_angle = 0;
    char* error_code = (char*)malloc(10);
    String m_name("");
    for(int i = 0; i < 2; i++){
      if(Serial.available()){
        m_name = Serial.readStringUntil('\n');
        if(m_name == "pwm"){
          pwm = Serial.parseInt();
        } else if(m_name ==  "break_angle") {
          break_angle = Serial.parseInt();
        }
      }
    }


    // print and save data every 10 loops
    if(print_count >= 9){
      print_count = 0;

      logfile.print("pwm\t");         logfile.print(pwm);               logfile.print("\t");
      logfile.print("break_angle\t"); logfile.print(break_angle);       logfile.print("\t");
      logfile.print("current\t");     logfile.print(current);           logfile.print("\t");
      logfile.print("voltage\t");     logfile.println(voltage);
      
      // clear everything before print
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);

      // display the current degree and angle difference
      display.print("PWM: ");         display.println(pwm);
      display.print("break angle: "); display.println(break_angle);
   
      display.display();


      led_state = !led_state;
      digitalWrite(P_LR, led_state);  // LED blinking
    }

    print_count++;  
  }  
}





bool button_push(uint8_t pin) {

}


// absolute function for encoder
uint16_t m_abs(uint16_t val1, uint16_t val2){
  uint16_t result = abs(val1 - val2);
  if(result > 180)
    result = 360 - result;
  if(result < ANGLE_THRESHOLD)
    result = 0;
   return result;
}


// motor control code
void setMotor(uint8_t pwm_value, boolean dir)       // drive-coast mode, no e-breaking, DIR: L forward, H backward
{
  digitalWrite(P_DIR, dir);
  analogWrite(P_PWML, pwm_value);
  analogWrite(P_PWMH, pwm_value);
}


void setEBreaking(uint8_t pwm_value)
{
  analogWrite(P_PWMH, 0);
  analogWrite(P_PWML, pwm_value);
}


// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}





