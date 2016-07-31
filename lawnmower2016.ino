
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

Reminder on Git:
git commit lawnmower_15.ino
git push origin master
*/

/*
HC-SR04 Ping distance sensor]
VCC to arduino 5v GND to arduino GND
Echo to Arduino pin 13 Trig to Arduino pin 12
Red POS to Arduino pin 11
Green POS to Arduino pin 10
560 ohm resistor to both LED NEG and GRD power rail
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
  Serial.begin (9600);
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
  lcd.print("Dist: ");
  lcd.print(dist);
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

  lcd.setCursor(0, 1); // bottom left
  lcd.print("Led tests done"); 
     
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
  
  batt();
  current();

  delay(3000);

  Serial.println("Setup done");

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
  lcd.setCursor(0, 1); // bottom left
  lcd.print("All done");
  

}

void loop() {
  // Measure distance
  long dist = distance();
  lcd.setCursor(0, 1); // bottom left
  lcd.print("Dist: ");
  lcd.print(dist);

  Serial.println("--------------------");
  Serial.print("Distance: ");
  Serial.println(dist/100);
  
  //lcd.clear();
  batt();
  current();
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
  delay(3000); 
 
}

void batt() {
  // Check battery monitor
  // (10.80V from PSU. 3.39 after divider)
  Serial.println("DC VOLTMETER");
  Serial.print("Maximum Voltage: ");
  Serial.print((int)(vPow / (r2 / (r1 + r2))));
  Serial.println("V");
  // Read AD and convert value
  float adcvalue = analogRead(voltsens);
  float v = (adcvalue * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  // Correction
  v2=v2-0.2;
  Serial.print("Battery monitor ADC: ");
  Serial.println(adcvalue);
  Serial.print("Battery voltage: ");
  Serial.print(v2);
  Serial.println(" volt.");
  
  lcd.home();
  lcd.print("U="); 
  lcd.print(v2); 
  lcd.print("V"); 
}

void current(){
  // Measure total power consumption
  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1024.0) * 5020; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  Serial.print("Amps ADC value: ");
  Serial.println(RawValue);
  Serial.print("Calculated amps ");
  Serial.println(Amps);
  
  lcd.print(" ");
  lcd.print("I="); 
  lcd.print(Amps); 
  lcd.print("A");  
}

void fwd_fast() {
  lcd.setCursor (0,1); 
  lcd.print("Fwd fast");

  analogWrite(speedPinA, 255);//Sets speed variable via PWM 
  analogWrite(speedPinB, 255);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);
}
void fwd_slow() {
  // Framåt -> dir1PinA + dir1PinB hög, dir2PinA + dir2PinB låg?

  lcd.setCursor (0,1); 
  lcd.print("Fwd slow");

  analogWrite(speedPinA, 100);//Sets speed variable via PWM 
  analogWrite(speedPinB, 100);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);
}
void rev_fast() {
  lcd.setCursor (0,1); 
  lcd.print("Rev fast");
  analogWrite(speedPinA, 255);//Sets speed variable via PWM 
  analogWrite(speedPinB, 255);
  digitalWrite(dir1PinA, HIGH);
  digitalWrite(dir2PinA, LOW);
  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, LOW);
}
void rev_slow() {
  lcd.setCursor (0,1); 
  lcd.print("Rev slow");
  analogWrite(speedPinA, 100);//Sets speed variable via PWM 
  analogWrite(speedPinB, 100);
  digitalWrite(dir1PinA, HIGH);
  digitalWrite(dir2PinA, LOW);
  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, LOW);
}
void stop() {
  lcd.setCursor (0,1); 
  lcd.print("Stop");
  Serial.println("Stop");
  analogWrite(speedPinA, 0);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  analogWrite(speedPinB, 0);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);
}
void rotateL() {
  lcd.setCursor (0,1); 
  lcd.print("RotateL      ");
  analogWrite(speedPinA, 100);//Sets speed variable via PWM 
  analogWrite(speedPinB, 100);
  digitalWrite(dir1PinA, HIGH);
  digitalWrite(dir2PinA, LOW);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);
}
void rotateR() {
  lcd.setCursor (0,1); 
  lcd.print("RotateR          ");
  analogWrite(speedPinA, 100);//Sets speed variable via PWM 
  analogWrite(speedPinB, 100);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, LOW);
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

