
void fwd_fast() {
  lcd.setCursor (0,1); 
  lcd.print("Fwd fast");
  Serial.println("Fwd fast");

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
  Serial.println("Fwd slow");

  analogWrite(speedPinA, 100);//Sets speed variable via PWM 
  analogWrite(speedPinB, 100);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);
}
void rev_fast() {
  lcd.setCursor (0,1); 
  lcd.print("Rew fast");
  Serial.println("Rew fast");
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
  Serial.println("Rew slow");
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
  Serial.println("Rotate L");
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
  Serial.println("Rotate R");
  analogWrite(speedPinA, 100);//Sets speed variable via PWM 
  analogWrite(speedPinB, 100);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, LOW);
}

  
