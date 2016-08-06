/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
#include <AS5145.h>
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>

#define MAIN_TIME 50
#define MAX_ANGLE 50.0
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
AS5145 gas_pedal(7,6,5,53);       // data, clock, chip select, program input.
AS5145 break_pedal(10,9,8,53);    // data, clock, chip select, program input.
AS5145 encoder(13,12,11,53);      // data, clock, chip select, program input.

// pin configurations
const uint8_t Temp1 = 54;          // temp sensor for battery 1, ADC0, need to init
const uint8_t Temp2 = 55;          // temp sensor for battery 2, ADC1, need to init
const uint8_t Temp3 = 56;          // temp sensor for battery 3, ADC2, need to init
const uint8_t Temp4 = 57;          // temp sensor for motor, ADC3, need to init
const uint8_t Temp5 = 58;          // temp sensor for motor, ADC4, need to init
const uint8_t Temp6 = 59;          // temp sensor for motor, ADC5, need to init
const uint8_t PWML = 2;
const uint8_t PWMH = 3;
const uint8_t DIR = 24;                   // init as output
const uint8_t CurSenPin = 62;             // ADC7, no need to init
const uint8_t CurSenIn = 63;              // ADC8, no need to init
const uint8_t P_VolSensor_MOT = 64;       // ADC9, no need to init
const uint8_t P_VolSensor_BAT = 65;       // ADC10, no need to init
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
int temp1 = 0;        // temp sensor for battery 1
int temp2 = 0;        // temp sensor for battery 2
int temp3 = 0;        // temp sensor for battery 3
int temp4 = 0;        // temp sensor for motor

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
  char filename[30] = "\0";
  myFile = SD.open("data.txt", FILE_WRITE);
  Serial.println("Please input your name BAH!!!");
  Serial2.println("Please input your name BAH!!!");
  while(strcmp(filename, "\0") == 0){
    digitalWrite(P_LR, HIGH);
    if(Serial2.available()){
      delay(100);
      for(int i = 0; i < 30 && Serial2.available(); i++){
        filename[i] = Serial2.read();
      }
      myFile.close();
      myFile = SD.open(filename, FILE_WRITE);
      digitalWrite(P_LR, LOW);
      Serial2.println("Serial2 break the loop!!");
      break;
    }
  }

  Serial2.print("filename is ");        Serial2.println(filename);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial2.println("initialization done.");
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.flush();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  
  m_time = millis();
  init_time = m_time;
  delay(1000);

  // re-open the file for reading:
  myFile = SD.open(filename);
  if (myFile) {
    Serial.println(filename);

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}


void loop() {
  // nothing happens after setup
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


uint8_t get_pwm(uint8_t value){
  uint8_t angle = m_abs(value, gas_init_angle);

  if(angle < ANGLE_THRESHOLD)
    return 0;
  // cubic output to motor
  uint8_t pwm = angle * angle * MAX_PWM / ( MAX_ANGLE * MAX_ANGLE);
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



