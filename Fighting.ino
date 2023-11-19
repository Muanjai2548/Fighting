#include <Arduino.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int E1 = 10;
int M1 = 12;
int E2 = 11;
int M2 = 13;
int sw = A0;

int sensorA = 0;
int sensorB = 0;
int sensorC = 0;

int step = 0;

void setup() {
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

void stop() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 0);  // PWM
  analogWrite(E2, 0);  // PWM
}


void forward() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 255);  // PWM //
  analogWrite(E2, 255);  // PWM
}

void backward() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, 255);  // PWM
  analogWrite(E2, 255);  // PWM
}

void left() {
  digitalWrite(M1, LOW); // direction 
  digitalWrite(M2, HIGH); // direction 
  analogWrite(E1, 255);  // PWM SPEED
  analogWrite(E2, 255);  // PWM SPEED
}

void right() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, 255);  // PWM
  analogWrite(E2, 255);  // PWM
}

void loop() {
  if (digitalRead(sw == 0)) {

    int analogA = analogRead(A1);
    int analogB = analogRead(A2);
    int analogC = analogRead(A3);
//    Serial.print("SA: ");
//    Serial.println(analogA);
//    Serial.print("SB: ");
//    Serial.println(analogB);
//    Serial.print("SC: ");
//    Serial.println(analogC);

    if (analogA > 700) {
      sensorA = 0;
    } else {
      sensorA = 1;
    }

    if (analogB > 700) {
      sensorB = 0;
    } else {
      sensorB = 1;
    }

    if (analogC > 700) {
      sensorC = 0;
    } else {
      sensorC = 1;
    }
    
    if (sensorB == 0 & sensorA == 1) {
      left();
      delay(200);
    } else if (sensorB == 1 & sensorA == 0) {
      right();
      delay(200);
    } else {
      forward();
    }
  }

}
