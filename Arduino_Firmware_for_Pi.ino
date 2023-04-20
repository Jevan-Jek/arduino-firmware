//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*                                                      *//
//*  EEEBot Firmware Code for the Arduino NANO           *//
//*  UoN EEEBot 2022                                     *//
//*  Nat Dacombe                                         *//
//********************************************************//

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

// read through the accompanying readme file - only then ask for help if you are still unsure

// DO NOT modify or edit any of this code - for Session 2, the Arduino NANO code is provided for you
// the only exception is modify the pin numbers for the motors (if the motors do not spin the correct way)

#include <Servo.h>
#include <Encoder.h>
#include <Wire.h>
//#include <FastLED.h>
//#define LED_PIN 7
//#define NUM_LEDS 6
//CRGB leds[NUM_LEDS];

#define servoPin 4

Servo myservo;        // create servo object to control a servo
float steeringAngle;  // variable to store the servo position

// either change your wire connections to the two encoders or change the pin numbers in the code
// to change whether the count increments or decrements when turning in a particular direction
Encoder enc1(2, 11);  // create an encoder object for encoder 1
Encoder enc2(3, 12);  // create an encoder object for encoder 2
long oldPos_enc1 = -999;  
long oldPos_enc2 = -999;
//long enc1_count;
//long enc2_count;
int16_t enc1_count;
int16_t enc2_count;

#define enA 5   // EnableA command line - should be a PWM pin
#define enB 6   // EnableB command line - should be a PWM pin

// name the motor control pins - replace the ** with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  // Channel A direction 
#define INb A1  // Channel A direction 
#define INc A2  // Channel B direction 
#define INd A3  // Channel B direction 


void setup() {
  Wire.begin(4);                // join I2C bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  myservo.attach(servoPin);  // attach our servo object to pin D4
  // the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)

  // configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

  // initialise serial communication
  Serial.begin(9600);
  Serial.println("Arduino Nano is Running"); // sanity check
  myservo.write(83);

  //LED Callibration
  //FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,500);
  //FastLED.clear();
  //FastLED.show();

  
}


void loop() {
  // update the two encoder values
  enc1_count = enc1.read(); //works  
  enc2_count = enc2.read(); //works
  if (enc1_count != oldPos_enc1) {
    oldPos_enc1 = enc1_count;
  }
  if (enc2_count != oldPos_enc2) {
    oldPos_enc2 = enc2_count;
  }
  //LED_lights();  
  //Serial.println(enc1_count);
}

void receiveEvent(int numBytes){
  //Set Up Vars
  int16_t servoAngle=83,leftMotorSpeed,rightMotorSpeed;
  double centreAngle=83, baseSpeed=100, K=0.4;
  int count=0;
  int16_t pid=0;
  
  //We'll recieve one byte at a time. Stop when none left
  while(Wire.available())
  {
    char c = Wire.read();    // receive a byte as character
    //Create Int from the Byte Array
    pid = c << (8 * count) | pid;
    count++;
  }
  //Print the Int out.
  //Serial.print("Error: "); 
  //Serial.println(error);

  
  servoAngle=centreAngle+pid;
  Serial.println(servoAngle);
  leftMotorSpeed=baseSpeed+(K*pid);
  rightMotorSpeed=baseSpeed-(K*pid);
  setSteeringAngle(servoAngle);
  runMotors(leftMotorSpeed, rightMotorSpeed);
  delay(10);
}

// this function executes when data is requested from the master device
void requestEvent(void)
{
  Wire.write((byte)((enc1_count & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(enc1_count & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((enc2_count & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(enc2_count & 0x000000FF));    
}


// function to set the steering angle
void setSteeringAngle(int servoAngle){
  servoAngle = constrain(servoAngle, 0, 180); // prevents the servo being set to 'impossible' angles
  myservo.write(servoAngle);
}


// function to run the motors
void runMotors(int leftMotorSpeed, int rightMotorSpeed){
  // limit the speed value between -255 and 255 as the PWM value can only be between 0 and 255 - the negative is handled below
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // vary the motor speeds - use the absolute value to remove the negative
  analogWrite(enA, abs(leftMotorSpeed));
  analogWrite(enB, abs(rightMotorSpeed));

  // if the speed value is negative, run the motor backwards
  if (leftMotorSpeed < 0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
  // else, run the motor forwards
  else {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);    
  }

  // if the speed value is negative, run the motor backwards
  if (rightMotorSpeed < 0) {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);    
  }
}

void LED_lights(){
  //Turn light from green to blue 
  // GREEN RED BLUE
  /*for(int i=0;i<NUM_LEDS;i++){ 
    leds[i]=CRGB(0,255-20*i,20*i);
    FastLED.setBrightness(6*i);
    FastLED.show();
    delay(50);
  }
  Turn light from red to green
  for(int i=NUM_LEDS;i>0;i--){
    leds[i]=CRGB(20*i,0, 255-20*i);
    FastLED.setBrightness(60-2*i);
    FastLED.show();
    delay(50);
  }*/
}
