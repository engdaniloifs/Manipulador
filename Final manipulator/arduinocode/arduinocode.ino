#include <Servo.h>

struct Point {
  int a1;  // base angle
  int a2;  // arm angle
};

const int MAX_POINTS = 50;
Point path_angle[MAX_POINTS];
int points_number = 0;


Servo base_servo;  // create Servo object to control a servo

Servo arm_servo;
// twelve Servo objects can be created on most boards

int ledvermelho = 3;
int ledverde = 4;


void setup() {
  
  Serial.begin(9600);
  base_servo.attach(9);
  arm_servo.attach(10);

  pinMode(ledvermelho, OUTPUT);
  pinMode(ledverde, OUTPUT);
  digitalWrite(ledvermelho, HIGH);
  digitalWrite(ledverde, LOW);
  base_servo.write(90);
  arm_servo.write(90);
  
  receivePath();

  digitalWrite(ledvermelho, LOW);
}

void loop() 
{
  static int i = 0; 
  digitalWrite(ledverde, HIGH);
  base_servo.write(path_angle[i].a1);
  arm_servo.write(path_angle[i].a2);
  delay(3000);
  i++;
  while(i!= points_number)
  {
    base_servo.write(path_angle[i].a1);
    arm_servo.write(path_angle[i].a2);
    delay(500);
    i++;
  }
  digitalWrite(ledvermelho, HIGH);
  delay(3000);

}


void receivePath() {

  while (!Serial.available());

  String input = Serial.readStringUntil('\n');
  points_number = input.toInt();

  if (points_number > MAX_POINTS) points_number = MAX_POINTS;

  for (int i = 0; i < points_number; i++) {
    while (!Serial.available());
    String line = Serial.readStringUntil('\n');
    int commaIndex = line.indexOf(',');

    if (commaIndex > 0) {
      path_angle[i].a1 = line.substring(0, commaIndex).toInt();
      path_angle[i].a2 = line.substring(commaIndex + 1).toInt();
    }
  }
}