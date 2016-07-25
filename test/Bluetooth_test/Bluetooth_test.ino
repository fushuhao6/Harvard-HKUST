#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

int ledpin = 13; // LED connected to pin 48 (on-board LED)


#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup() {

  pinMode(ledpin, OUTPUT);  // pin 48 (on-board LED) as OUTPUT
  Serial.begin(9600);       // start serial communication at 9600bps

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

void loop() {


      // clear everything before print
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);

    /*
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
    */

    while(Serial.available()){
      display.println(Serial.readStringUntil('\n'));
    }
    display.display();

    delay(500);
} 
