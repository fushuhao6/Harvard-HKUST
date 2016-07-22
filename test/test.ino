#include <AS5145.h>
#include <stdio.h>
#include <stdlib.h>

#define MAIN_TIME 50
#define MAX_DUTY_CYCLE 90.0
#define MAX_ANGLE 70.0
#define MAX_PWM 255
#define ANGLE_THRESHOLD 3
#define BREAK_THRESHOLD 20

// absolute function for encoder
uint16_t m_abs(uint16_t val1, uint16_t val2);
uint8_t get_duty_cycle(uint16_t value);

// motor control functions
void setMotor(uint8_t pwm_value, boolean dir);    // drive-coast mode, no e-breaking, DIR: L forward, H backward
void setEBreaking(uint8_t pwm_value);

// encoders
AS5145 encoder(7,6,5,4);      // data, clock, chip select, program input.
AS5145 gas_pedal(7,6,5,4);    // data, clock, chip select, program input.
AS5145 break_pedal(7,6,5,4);  // data, clock, chip select, program input.

// pin configurations
const uint8_t P_LR = 13; //red led, init as output
const uint8_t Temp1 = 55; // temp sensor, ADC, need to init
const uint8_t PWML = 2;
const uint8_t PWMH = 3;
const uint8_t DIR = 4;            // init as output
const uint8_t CurSenPin = 8;      // ADC, no need to init
const uint8_t VolSenPin = 9;      // ADC, no need to init
const uint8_t DirButton = 30;     // to control motor direction, init as input

// time configurations
unsigned long m_time;     // main time

// angle and speed values
uint16_t abs_angle[2] = {0};
uint32_t measure_time[2] = {0};
float m_speed[2] = {0};
uint16_t gas_init_angle = 0;
uint16_t break_init_angle = 0;

// temp config
int temp1 = 0;

void setup()
{
  pinMode(P_LR, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(DirButton, INPUT);
  digitalWrite(DIR, LOW);
  digitalWrite(P_LR, LOW);

  gas_init_angle = gas_pedal.encoder_degrees();
  break_init_angle = break_pedal.encoder_degrees();
  abs_angle[0] = encoder.encoder_degrees();
  abs_angle[1] = abs_angle[0];
  measure_time[0] = millis();
  measure_time[1] = measure_time[0];
  
  analogWrite(PWML, 0);
  analogWrite(PWMH, 0);
  
  Serial.begin(9600);         // output to the terminal
  Serial1.begin(9600);        // communicate with feather
  Serial2.begin(9600);        // bluetooth
  m_time = millis();
  delay(1000);
}

boolean led_state = false;
uint8_t print_count = 0;


/************************** /
/ Front board main function /
/ Deals with two pedals,    /
/ bluetooth transmission,   /
/ communication with feather/
/ and probably LCD          /
/ **************************/
void loop()
{

  if(millis() - m_time >= MAIN_TIME){
    m_time = millis();

    // measure speed (move this part to feather)
    abs_angle[1] = encoder.encoder_degrees();
    measure_time[1] = millis();
    m_speed[1] = (abs_angle[1] - abs_angle[0]) * 1000 / (measure_time[1] - measure_time[0]);       // degrees per second
    // update time and angle
    abs_angle[0] = abs_angle[1];
    measure_time[0] = measure_time[1];
    float rpm = m_speed[1] * 60.0 / 360;

    // measure gas pedal
    uint16_t value = gas_pedal.encoder_degrees();
    uint8_t pwm = get_pwm(value);    

    // measure break pedal
    uint16_t break_angle = m_abs(break_pedal.encoder_degrees(), break_init_angle);

    // current sensor and voltage sensor (have not implemented calculation)
    uint16_t current = analogRead(CurSenPin);
    uint16_t voltage = analogRead(VolSenPin);

    // give pwm to the motor driver iff no breaking and power <= 420W
    if(break_angle <= ANGLE_THRESHOLD && (voltage * current) <= 420){
      
      // add direction code here if you have the button
      setMotor(pwm, LOW);
    } else if ((voltage * current) > 420 || (break_angle > ANGLE_THRESHOLD && break_angle < BREAK_THRESHOLD)) {       // else if overdrive or little breaking
      setMotor(0, LOW);       // shut down the motor, coasting
    } else {             // breaking hard
      uint8_t e_pwm = (break_angle - BREAK_THRESHOLD) / (MAX_ANGLE - BREAK_THRESHOLD) * MAX_PWM;
      if(e_pwm > 200)
        e_pwm = 200;
      setEBreaking(e_pwm);    // e-breaking
    }
    

    // measure temp (may have more temp sensors, and move this part to feather)
    temp1 = analogRead(Temp1);
    temp1 = (temp1 * 3300 / 1023.0 - 500) / 10;

    // store data into SD Card

    //communicate with feather
    Serial1.print("pwm\n");         Serial1.println(pwm);
    Serial1.print("break_angle\n"); Serial1.println(break_angle);
    Serial1.print("current\n");     Serial1.println(current);
    Serial1.print("voltage\n");     Serial1.println(voltage);

    

    // print every 10 loops
    if(print_count >= 10){
      print_count = 0;

      Serial.print("init angle is: ");
      Serial.println(gas_init_angle); 
      Serial.print("abs angle is: ");
      Serial.println(value); 
      Serial.print("gas pedal angle is: ");
      Serial.println(m_abs(value, gas_init_angle));
      Serial.print("pwm is: ");
      Serial.println(pwm); 

      Serial2.print("init angle is: ");
      Serial2.println(gas_init_angle); 
      Serial2.print("abs angle is: ");
      Serial2.println(value); 
      Serial2.print("gas pedal angle is: ");
      Serial2.println(m_abs(value, gas_init_angle));
      Serial2.print("pwm is: ");
      Serial2.println(pwm); 

      led_state = !led_state;
      digitalWrite(P_LR, led_state);  // LED blinking
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
  uint16_t angle = m_abs(value, gas_init_angle);
  
  // cubic output to motor
  uint16_t pwm = angle * angle * MAX_DUTY_CYCLE * MAX_PWM / (100 * MAX_ANGLE * MAX_ANGLE);
  return pwm < MAX_PWM? pwm:MAX_PWM;
}

// motor control code
void setMotor(uint8_t pwm_value, boolean dir)       // drive-coast mode, no e-breaking, DIR: L forward, H backward
{
  digitalWrite(DIR, dir);
  analogWrite(PWML, pwm_value);
  analogWrite(PWMH, pwm_value);
}


void setEBreaking(uint8_t pwm_value)
{
  analogWrite(PWMH, 0);
  analogWrite(PWML, pwm_value);
}












