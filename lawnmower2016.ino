
/* Arduino based robot lawnmower
©Patrik Hermansson 2016

-Uses a L298-based card for controlling two driver motors, left and rear.
-Cutting motor controlled by Arduino, and a relay.
-Led:s indicating high current drawn, low battery and if robot is near something. 
-Measures total current drawn with ACS712 current meter.
-Battery measured by resistor divider and Arduino internal AD-converter. 
-Measures distance to foreign objects with HC-SR04 sensor, 

 
Two electrical motors are used for moving and turning, controlled by a L298N-based driver
(code inspired by http://www.instructables.com/id/Arduino-Modules-L298N-Dual-H-Bridge-Motor-Controll/?ALLSTEPS)
LCD: https://alselectro.wordpress.com/2016/05/12/serial-lcd-i2c-module-pcf8574/

Wireless programming/control via ESP8266: http://www.patrikhermansson.se/?q=node/520
avrdude -C/home/patrik/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino2/etc/avrdude.conf -v -patmega328p -carduino -P net:192.168.1.130:23 -b57600 -D -Uflash:w:/tmp/builde2680df248d7f8adec6f6bb45c90d9b2.tmp/lawnmower2016.ino.hex:i

Reminder on Git:
git commit lawnmower2016.ino
git push origin master
*/

/*
HC-SR04 Ping distance sensor]
VCC to arduino 5v 
GND to arduino GND
Echo to Arduino pin 9 
Trig to Arduino pin 8

Red POS to Arduino pin 11 ?
Green POS to Arduino pin 10 ? 
560 ohm resistor to both LED NEG and GRD power rail ?
More info at: http://goo.gl/kJ8Gl
Original code improvements to the Ping sketch sourced from Trollmaker.com
Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
*/

// LCD, connected via I2C backpack. 
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C  lcd(0x20,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

// Connections for driver card
/*
 * When ENA enable IN1 IN2 control OUT1 OUT2
   When ENB enable IN3 IN4 control OUT3 OUT4

Framåt -> dir1PinA + dir1PinB hög, dir2PinA + dir2PinB låg?
 
 */

// Motor 1
int dir1PinA = 2;  // IN1
int dir2PinA = 4;  // IN2
int speedPinA = 3; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 5; // IN3
int dir2PinB = 7; //IN4
int speedPinB = 6; // Needs to be a PWM pin to be able to control motor speed

// Connections for HC-SR04
#define trigPin 8
#define echoPin 9

// Led for detect close distance
//#define lednear 7
//#define ledstuck 

bool status=true;
bool error=false;
bool run=false;

//Measuring Current Using ACS712
const int analogIn = A0;
int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
int RawValue= 0;
int ACSoffset = 2500; // Gets values slightly off (-0.10)
double Voltage = 0; // Input value from current sensor
double Amps = 0;

// Battery voltage meter
// A resistor divider connected to the battery and ground. 
#define lowbattled 13
#define voltsens A1   // Mid point of divider connected to input A1. 
float vPow = 5.02; // Voltage at the Arduinos Vcc and Vref. 
float r1 = 1000000;  // "Top" resistor, 1Mohm.
float r2 = 470000;   // "Bottom" resistor (to ground), 470 kohm. 

/*
#define cutmotor 5
#define currentsens A0
int dir;
*/

void setup() {
  Serial.begin (57600);
  Serial.println("Welcome to Lawnmower16!"); 

  // activate LCD module
  lcd.begin (16,2); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // set cursor to 0,0
  lcd.print("Welcome"); 
  delay(3000);

  // Define pins for HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Measure distance
  long dist = distance();
  lcd.setCursor(0, 1); // bottom left
  lcd.print("D: ");
  lcd.print(dist);
  delay(1000);
  
  /*
  pinMode(lednear, OUTPUT);
  // Test led near
  digitalWrite(lednear, HIGH);
  delay(500);
  digitalWrite(lednear, LOW);
  
  pinMode(ledstuck, OUTPUT);
  // Test ledstuck
  digitalWrite(ledstuck, HIGH);
  delay(500);
  digitalWrite(ledstuck, LOW);
  */

  //lcd.setCursor(0, 1); // bottom left
  //lcd.print("Led tests done"); 
     
  //Define L298N Dual H-Bridge Motor Controller Pins
  pinMode(dir1PinA,OUTPUT);
  pinMode(dir2PinA,OUTPUT);
  pinMode(speedPinA,OUTPUT);
  pinMode(dir1PinB,OUTPUT);
  pinMode(dir2PinB,OUTPUT);
  pinMode(speedPinB,OUTPUT);

  /*
  Serial.println("Spin up cut motor");
  pinMode(cutmotor, OUTPUT);
  digitalWrite(cutmotor, LOW);
  delay(2000);
  digitalWrite(cutmotor, HIGH);
  */
  
  pinMode(lowbattled, OUTPUT);
  pinMode(voltsens, INPUT);
  
  int battv = batt();
  Serial.print ("B: ");
  Serial.print(battv);
  Serial.println("V");

  delay(100);

  lcd.print("B: ");
  lcd.print(battv);
  
  current();

  delay(3000);

  Serial.println("Setup done");

/*
  // Hardware test, lets dance!
  lcd.setCursor(0, 1); // bottom left
  lcd.print("Dance!");
  delay(1000);
  fwd_slow();
  delay(4000);
  stop(); 
  
  rev_fast();
  delay(2000);
  stop();
  
  fwd_fast();
  delay(2000);
  stop();
  rev_fast();
  delay(1000);
  stop();
  rotateL();
  delay (1000);
  stop();
*/
  
  lcd.setCursor(0, 1); // bottom left
  lcd.print("All done");
  delay(500);

}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
      int command = Serial.parseInt();
      Serial.println("Got serial data");
      // look for the newline. That's the end of your sentence:
      if (Serial.read() == '\n') {
        Serial.print("Got: ");
        Serial.println(command);
        if (command==1) {
          Serial.print ("Run!");
          run=true;
        }
        else if (command==2) {
          Serial.print ("Stop!");     
          run=false;     
        }
      }
  }
  lcd.clear();
  Serial.println("----");
  
  // Measure distance
  long dist = distance();
  lcd.setCursor(0, 0); // top left
  lcd.print("D:");
  lcd.print(dist);
  lcd.print(" ");
  delay(100);
  Serial.print("D: ");
  Serial.println(dist);
  // Do something if we are near an object
  if (dist<1000) {
    stop();
    delay(1000);
    rotateL();
    delay(3000);
    stop();
    delay(100);

  }
  
  // Check battery voltage
  int battv = batt();
  // battv is like '126' (12.6V)
  int binteger = battv/10;  // = 12
  int temp = (binteger*10); // = 120
  int bdec = battv - temp;  // = 6 
  String batt_text = String(binteger + "." + bdec);
  
  lcd.print("B:");
  lcd.print(batt_text);
  lcd.print(" ");
  
  Serial.print ("B: ");
  Serial.print(batt_text);
  Serial.println("V");
  /*
  Serial.print ("Battv: ");
  Serial.println (battv);
  Serial.print("bint: ");
  Serial.println(binteger);
  Serial.print("bdec: ");
  Serial.println(bdec);
  */
  // Do something if the battery is low
  /*
  if ((binteger)<=9) {
    Serial.println("BATTERY LOW!");
    Serial.println("B:" + batt_text);
    lcd.clear();
    lcd.home();
    lcd.print ("BATTERY LOW!");
    lcd.setCursor(0, 1); // bottom left
    lcd.print ("B:");
    lcd.print(batt_text);
    // Set error flag
    error=true;
  }
*/
  // Check current
  int cAmp = current();
  lcd.print("I:");
  lcd.print(cAmp);
  lcd.print(" ");
  // Do something if the current is too high
  /*if (cAmp>){
    error=true;
    }
  */

  if (error==false && run==true) {
    Serial.println ("All ok, forward");
    //fwd_slow();
  }
  else if (error==false){
    stop();
    lcd.setCursor(0, 1); // bottom left
    lcd.print ("           ");
    lcd.print("ERROR");
  }

  // Heartbeat on LCD
  if (status) {
    status=false;
    lcd.setCursor(15, 1); 
    lcd.print("|");
  }  
  else {
    status=true;
    lcd.setCursor(15, 1); 
    lcd.print("-");
  }

  // Wait for next round
  delay(2000); 
 
}

//float batt() {
int batt() {

// Does this have to be a float?
  
  // Check battery monitor
  // (10.80V from PSU. 3.39 after divider)
  //Serial.println("DC VOLTMETER");
  //Serial.print("Maximum Voltage: ");
  //Serial.print((int)(vPow / (r2 / (r1 + r2))));
  //Serial.println("V");
  // Read AD and convert value
  int adcvalue = analogRead(voltsens);  // No need for a float here?
  vPow = 15;
  int volt = (adcvalue * vPow) / 1024.0;
  int volt2 = volt / (r2 / (r1 + r2));
  Serial.print("volt: ");
  Serial.println(volt2);
  float v = (adcvalue * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  // Correction
  v2=v2-0.2;
  /*
  Serial.print("Battery monitor ADC: ");
  Serial.println(adcvalue);
  Serial.print("Battery voltage: ");
  Serial.print(v2);
  Serial.println(" volt.");
*/
  // Convert to int
  v2=v2*10;
  int batt=(int) v2;
  //Serial.println(batt);

  /*  
  lcd.home();
  lcd.print("U="); 
  lcd.print(v2); 
  lcd.print("V"); 
  */
  return batt;
}

//double current(){
int current(){
    
  // Measure total power consumption
  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1024.0) * 5020; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  //Serial.print("Amps ADC value: ");
  //Serial.println(RawValue);
  Serial.print("Calculated amps ");
  Serial.println(Amps);
  
  lcd.print(" ");
  lcd.print("I="); 
  lcd.print(Amps); 
  lcd.print("A");  
}

long distance() {
  // Measure distance to sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

}

