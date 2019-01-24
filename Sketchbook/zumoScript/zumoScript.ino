#include <ZumoShield.h>
#include <NewPing.h>

#define TRIGGER_PIN  6 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

ZumoMotors motors;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int speed = 100; // max speed is 255
bool object = false;

void setup(void)
{
  int i;
  for(i=5;i<=8;i++)
  pinMode(i, OUTPUT);
  Serial.begin(9600);
  Serial.println("setup");
}

void loop(void)
{
  while (Serial.available() < 1) {} // Wait until a character is received
  char val = Serial.read();
  switch(val) {
    case 'w'://Move Forward
    case 'W':
      forward (speed);
      break;
    case 's'://Move Backwards
    case 'S':
      reverse (speed);
      break;
    case 'a'://Turn Left
    case 'A':
      left (speed);
      break;
    case 'd'://Turn Right
    case 'D':
      right (speed);
      break;
    case 'r': //Do a room
    case 'R':
        // left or right?
        while (Serial.available() < 1) {} // Wait until a character is received
        val = Serial.read();
        if(val == 'l' || val == 'L') {
            roomLeft();
        } else if(val == 'r' || val == 'R') {
            roomRight();
        }
        break;
    default:
      stop();
      break;
  }
}

void stop(void) //Stop
{
  motors.setSpeeds(0,0);
}

void forward(int speed)
{
  motors.setSpeeds(speed, speed);
}

void reverse (int speed)
{
  motors.setSpeeds(-speed, -speed);
}

void left (int speed)
{
  motors.setSpeeds(-speed, speed);
}

void right (int speed)
{
  motors.setSpeeds(speed, -speed);
}

void roomRight() {
  // turn right 90 degrees
  right(speed);
  //tune this to get turn right
  delay(100);
  // move forward a bit 
  forward(speed);
  delay(100);
  stop();
  scanRoom();
  Serial.println("ready");
}

void roomLeft() {
  // turn left 90 degrees
  left(speed);
  // tune this to get turn right
  delay(100);
  // move forward a bit 
  forward(speed);
  delay(100);
  stop();
  scanRoom();
  Serial.println("ready");
}

void scanRoom() {
  object = false;
  // tune this till we can nolonger detect objects
  left(150);
  // tune this untill we 360
  long currentTime = millis();
  long endTime = currentTime + 2700;
  while(currentTime < endTime) {
    currentTime = millis();
    int distance = sonar.ping_cm();
    if(distance <= 20) {
      object = true;
    }
  }
  stop();
  if(object == true) 
    Serial.println("Object in room");
  Serial.flush();
}
