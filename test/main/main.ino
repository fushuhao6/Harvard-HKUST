#include <AS5145.h>
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>

#define MAIN_TIME 50
#define MAX_ANGLE 55.0
#define MAX_PWM 255
#define ANGLE_THRESHOLD 3
#define BREAK_THRESHOLD 12.0
#define ENC_SIZE  6
#define DRIVER_NUM  6

// SD card
File myFile;

// motor control functions
void setMotor(uint8_t pwm_value, boolean dir);    // drive-coast mode, no e-breaking, DIR: L forward, H backward
void setEBreaking(uint8_t pwm_value);

// encoders
AS5145 break_pedal(7,6,5,53);       // data, clock, chip select, program input.
AS5145 gas_pedal(36,38,40,53);    // data, clock, chip select, program input.
AS5145 encoder(42,44,46,53);      // data, clock, chip select, program input.

// pin configurations
const uint8_t Temp1 = 54;          // temp sensor for battery 1, ADC0, need to init
const uint8_t Temp2 = 55;          // temp sensor for motor, ADC1, need to init

const uint8_t PWML = 8;
const uint8_t PWMH = 9;
const uint8_t DIR = 24;                   // init as output
const uint8_t CurSenPin = 62;             // ADC8, no need to init
const uint8_t CurSenIn = 63;              // ADC9, no need to init
const uint8_t P_VolSensor_MOT = 64;       // ADC10, no need to init
const uint8_t P_VolSensor_BAT = 65;       // ADC11, no need to init
const uint8_t DirButton = 30;             // to control motor direction, init as input
const uint8_t P_BreakLight = 32;          // to control the break light, init as output
const uint8_t P_LR = 47;          // Red LED, init as output
const uint8_t P_LY = 49;          // Yellow LED, init as output
const uint8_t P_LG = 51;          // Green LED, init as output

// time config
unsigned long m_time;     // main time
unsigned long init_time;  // for sd card to store time

// angle and speed values
uint8_t enc_buffer[ENC_SIZE] = {0};      // for averaging usage
uint8_t abs_angle[2] = {0};       // store past value
uint32_t measure_time[2] = {0}; 
float m_speed[2] = {0};
uint8_t gas_init_angle = 0;
uint8_t break_init_angle = 0;

// temp config
float temp1 = 0;        // temp sensor for battery 1
float temp2 = 0;        // temp sensor for motor

void setup()
{
  pinMode(P_LR, OUTPUT);
  pinMode(P_LY, OUTPUT);
  pinMode(P_LG, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(DirButton, INPUT);
  pinMode(PWMH, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(P_BreakLight, OUTPUT);
  //pinMode(P_VolSensor_MOT, INPUT);
  //pinMode(P_VolSensor_BAT, INPUT);
  digitalWrite(DIR, LOW);
  digitalWrite(P_LR, LOW);
  digitalWrite(P_LY, HIGH);
  digitalWrite(P_LG, LOW);
  digitalWrite(P_BreakLight, LOW);

  uint8_t init_buffer[ENC_SIZE] = {0};
  //while(gas_init_angle == 0){
    for(int i = 0; i < ENC_SIZE; i++){
        init_buffer[i] = break_pedal.encoder_degrees();
    }
    break_init_angle = average(init_buffer, ENC_SIZE);
  //}

  //while(break_init_angle == 0){
    for(int i = 0; i < ENC_SIZE; i++){
        init_buffer[i] = gas_pedal.encoder_degrees();
    }
    gas_init_angle = average(init_buffer, ENC_SIZE);
  //}
  
  abs_angle[0] = encoder.encoder_degrees();
  abs_angle[1] = abs_angle[0];
  measure_time[0] = millis();
  measure_time[1] = measure_time[0];
  
  analogWrite(PWML, 0);
  analogWrite(PWMH, 0);
  
  Serial.begin(9600);         // output to the terminal
  Serial2.begin(9600);        // bluetooth

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

    // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  char filename[15] = "\0";
  myFile = SD.open("data.txt", FILE_WRITE);
  Serial.println("Please input your name Bitch Ass Hoez!!!");
  Serial2.print("Please input your name Bitch Ass Hoez!!!\t");     Serial2.println("(less than 8 characters)");
  while(strcmp(filename, "\0") == 0){
    digitalWrite(P_LR, HIGH);
    if(Serial2.available()){
      delay(100);
      int i = 0;
      for(i = 0; i < 10 && Serial2.available(); i++){
        filename[i] = Serial2.read();
      }
      filename[i++] = '.';
      filename[i++] = 't';
      filename[i++] = 'x';
      filename[i++] = 't';
      filename[i++] = '\0';
      myFile.close();
      myFile = SD.open(filename, FILE_WRITE);
      digitalWrite(P_LR, HIGH);
      while(1){
        if (m_abs(break_pedal.encoder_degrees(), break_init_angle) > ANGLE_THRESHOLD){
          break;
        }
      }
      digitalWrite(P_LR, LOW);
      break;
    } else if (m_abs(break_pedal.encoder_degrees(), break_init_angle) > ANGLE_THRESHOLD){
      digitalWrite(P_LR, LOW);
      break;
    }
  }

  Serial2.print("filename is ");        Serial2.println(filename);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial2.println("initialization done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  
  m_time = millis();
  init_time = m_time;
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
  // loop, 50ms
  if(millis() - m_time >= MAIN_TIME){
    m_time = millis();

    // measure speed (move this part to feather)
    abs_angle[1] = encoder.encoder_degrees();
    measure_time[1] = millis();
    m_speed[1] = (m_abs(abs_angle[1], abs_angle[0])) * 1000 / (measure_time[1] - measure_time[0]);       // degrees per second
    // update time and angle
    abs_angle[0] = abs_angle[1];
    measure_time[0] = measure_time[1];
    float rpm = m_speed[1] * 60.0 / 360;


    // measure gas pedal (with smooth function)
    for(int i = 0; i < ENC_SIZE; i++){
      enc_buffer[i] = gas_pedal.encoder_degrees();
    }
    uint8_t value = average(enc_buffer, ENC_SIZE);
    uint8_t pwm = get_pwm(value);    

    // measure break pedal (with smooth function)
    for(int i = 0; i < ENC_SIZE; i++){
      enc_buffer[i] = break_pedal.encoder_degrees();
    }
    value = average(enc_buffer, ENC_SIZE);
    
    uint16_t break_angle = m_abs(value, break_init_angle);


    // current sensor and voltage sensor
    float cur_buffer = 0;
    for(int i = 0; i < 3; i++){
      cur_buffer += analogRead(CurSenPin);
    }
    float current = (cur_buffer / 3.0) / 1023.0 * 3.3 / 33 * 53;

    cur_buffer = 0;
    for(int i = 0; i < 3; i++){
      cur_buffer += analogRead(CurSenIn);
    }
    float input_current = (cur_buffer/ 3.0) / 1023.0 * 3.3 / 33 * 53;

    float real_cur = (current - input_current / 2) / 0.066;

    float mot_voltage = analogRead(P_VolSensor_MOT) / 1023.0 / 3.0 * 46.0 * 3.3;
    float bat_voltage = analogRead(P_VolSensor_BAT) / 1023.0 / 3.0 * 46.0 * 3.3;



    // measure temp (may have more temp sensors)
    temp1 = analogRead(Temp1);
    temp1 = (temp1 * 3300 / 1023.0 - 500) / 10;
    temp2 = (analogRead(Temp2) * 3300 / 1023.0 - 500) / 10;



    // give pwm to the motor driver iff no breaking and power <= 420W
    if(break_angle <= ANGLE_THRESHOLD /*&& (voltage * current) <= 420*/){
      
      //TODO: add direction code here if you have the button
      boolean dir = digitalRead(DirButton);
      digitalWrite(P_BreakLight, LOW);
      setMotor(pwm, dir);
    } else if (/*(voltage * current) > 420 ||*/ (break_angle > ANGLE_THRESHOLD /*&& break_angle < BREAK_THRESHOLD*/)) {       // else if overdrive or little breaking
      //setMotor(0, LOW);       // shut down the motor, coasting
      digitalWrite(P_BreakLight, HIGH);
    } 
    /*else {                  // breaking hard
      uint8_t e_pwm = (break_angle - BREAK_THRESHOLD) / (MAX_ANGLE - BREAK_THRESHOLD) * MAX_PWM;
      if(e_pwm > 200)
        e_pwm = 200;
      setEBreaking(e_pwm);    // e-breaking
      digitalWrite(P_BreakLight, HIGH);
    }
    */
    setMotor(pwm, digitalRead(DirButton));
    // print every 10 loops, 500ms
    if(print_count >= 9){
      print_count = 0;

      //Serial.print("init_angle is\n");    Serial.println(gas_init_angle);
      //Serial.print("pwm\t");              Serial.println(pwm);
      //Serial.print("raw data\t");         Serial.println(gas_pedal.encoder_degrees());
      //Serial.print("break_angle\n");      Serial.println(break_angle);
      Serial.print("rpm\t");         Serial.println(rpm);
      //Serial.print("abs_angle\t");         Serial.println(abs_angle[1]);
      Serial.print("current is \t");         Serial.println(current);
      Serial.print("Input current is \t");         Serial.println(input_current);
      Serial.print("Real current is \t");         Serial.println(real_cur);
      //Serial.print("Battery temp is \t");          Serial.println(temp1);
      //Serial.print("Motor temp is \t");           Serial.println(temp2);
      

      //Serial2.print("init_angle is\n");    Serial2.println(gas_init_angle);
      Serial2.print("pwm ");              Serial2.println(pwm);
      Serial2.print("rpm ");              Serial2.println(rpm);
      //Serial2.print("raw data\t");         Serial2.println(gas_pedal.encoder_degrees());
      //Serial2.print("Break angle\t");       Serial2.println(break_pedal.encoder_degrees());
      Serial2.print("current ");      Serial2.println(real_cur);
      Serial2.print("voltage ");      Serial2.println(mot_voltage);        
                 
      // store data to the SD card
      myFile.print("time\t");                       myFile.print(millis() - init_time);           myFile.print("ms\t");
      myFile.print("rpm\t");                        myFile.print(rpm);                            myFile.print("\t");
      myFile.print("Battery temp is \t");           myFile.print(temp1);                myFile.print("\t");
      myFile.print("Motor temp is \t");           myFile.print(temp2);                  myFile.print("\t");
      myFile.print("current\t");      myFile.print(real_cur);                        myFile.print("A\t");
      myFile.print("voltage\t");      myFile.print(mot_voltage);                    myFile.println("V"); 
      
      myFile.flush();

      led_state = !led_state;
      digitalWrite(P_LG, led_state);  // LED blinking
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
  uint8_t angle = m_abs(value, gas_init_angle);

  if(angle < ANGLE_THRESHOLD)
    return 0;
  // cubic output to motor
  uint16_t pwm = angle * angle * MAX_PWM / ( MAX_ANGLE * MAX_ANGLE);
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
  analogWrite(PWML, 0);
}


float average(uint8_t values[], uint8_t m_size){
  if(m_size >= 4){
    float m_buffer = 0;
    uint8_t max_index = 0, min_index = 1;
    // remove max and min
    for(int i = 0; i < m_size; i++){
      if(values[i] > values[max_index]){
        max_index = i;
      } else if(values[i] < values[min_index]){
        min_index = i;
      }
    }
    // average
    for(int i = 0; i < m_size; i++){
      if(i != min_index && i != max_index)
        m_buffer += values[i];
    }
    return m_buffer / (m_size - 2);
  } else {
    float m_buffer = 0;
    // average
    for(int i = 0; i < m_size; i++){
      m_buffer += values[i];
    }
    return m_buffer / m_size;
  }
}










