#include <AS5145.h>
#include <stdio.h>
#include <stdlib.h>

#define MAIN_TIME 50
#define MAX_DUTY_CYCLE 90.0
#define MAX_ANGLE 70.0
#define MAX_PWM 255
#define ANGLE_THRESHOLD 3

// absolute function for encoder
uint16_t m_abs(uint16_t val1, uint16_t val2);
uint8_t get_duty_cycle(uint16_t value);

AS5145 myAS5145(7,6,5,4); // data, clock, chip select, program input.
//AS5145 myAS5145(9);       // pwm input

// pin configurations
const uint8_t P_LR = 13; //red led
const uint8_t Temp1 = 55; // temp sensor
const uint8_t communicate = 56;  // communicate with feather
const uint8_t PWML = 2;
const uint8_t PWMH = 3;
const uint8_t DIR = 4;

// time configurations
unsigned long m_time;     // main time

// angle and speed values
uint16_t abs_angle[2] = {0};
float m_speed[2] = {0};
uint16_t init_angle = 0;

// temp config
int temp1 = 0;

// pwm value
int pwm_value = 0;
boolean pwm_state = 0;


void setup()
{
  pinMode(P_LR, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, LOW);
  digitalWrite(P_LR, LOW);

  init_angle = myAS5145.encoder_degrees();
  
  analogWrite(PWML, pwm_value);
  analogWrite(PWMH, pwm_value);
  delay(1000);
  
  Serial.begin(9600);
  Serial1.begin(9600);
  m_time = millis();
}

boolean led_state = false;
boolean pin_state = false;

uint8_t print_count = 0;

void loop()
{

  if(millis() - m_time >= MAIN_TIME){
    m_time = millis();

    // measure speed 
    abs_angle[1] = myAS5145.encoder_degrees();
    m_speed[1] = (abs_angle[1] - abs_angle[0]) * 1000 / 50;       // degrees per second
    abs_angle[0] = abs_angle[1];
    float rpm = abs_angle[1] * 60 / 360;

    Serial1.print(abs_angle[1]);

    // measure gas pedal
    uint16_t value = myAS5145.encoder_degrees();
    uint8_t pwm = get_pwm(value);    

    // measure break pedal

    

    // measure temp (may have more temp sensors)
    temp1 = analogRead(Temp1);
    temp1 = (temp1 * 3300 / 1023.0 - 500) / 10;



    // print every 10 loops
    if(print_count >= 10){
      print_count = 0;
        pwm_value += 10;
        if(pwm_value >= 200){
          pwm_value = 200;
        }


      Serial.print("init angle is: ");
      Serial.println(init_angle); 
      Serial.print("abs angle is: ");
      Serial.println(value); 
      Serial.print("gas pedal angle is: ");
      Serial.println(m_abs(value, init_angle));
      Serial.print("pwm is: ");
      Serial.println(pwm); 
      
      // Serial.print("PWM is: ");
      // Serial.println(pwm_value);

      analogWrite(PWML, pwm_value);
      analogWrite(PWMH, pwm_value);

      led_state = !led_state;
      digitalWrite(P_LR, led_state);  // LED blinking

      /*
      
      Serial.print("measured degrees: ");
      Serial.println(abs_angle[1]);

      Serial.print("measured temp: ");
      Serial.print(temp1);
      Serial.println("°C");
  
      Serial.println("");
      */
    
    }

    print_count++;  
  }


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

uint8_t get_pwm(uint16_t value){
  uint16_t angle = m_abs(value, init_angle);
  
  // cubic output to motor
  uint16_t pwm = angle * angle * MAX_DUTY_CYCLE * MAX_PWM / (100 * MAX_ANGLE * MAX_ANGLE);
  return pwm < MAX_PWM? pwm:MAX_PWM;
}

