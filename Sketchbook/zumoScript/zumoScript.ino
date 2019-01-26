#include <ZumoShield.h>
#include <NewPing.h>

#define TRIGGER_PIN  6 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define QTR_THRESHOLD 400

ZumoMotors motors;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
ZumoReflectanceSensorArray reflectanceSensors;
// Define an array for holding sensor values.
#define NUM_SENSORS 6
unsigned int sensorValues[NUM_SENSORS];

int speed = 100; // max speed is 255
bool object = false;

void setup(void)
{
  //initialise and calibrate reflectance sensors
  reflectanceSensors.init();
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);        // turn on LED to indicate we are in calibration mode
  unsigned long startTime = millis();
  while(millis() - startTime < 5000)   // make the calibration take 10 seconds
  {
    reflectanceSensors.calibrate();
  }
  digitalWrite(13, LOW);         // turn off LED to indicate we are through with calibration
  
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
    case 'G'://start
    case 'g':
      break;
    case 'C'://continue
    case 'c':
      break;
    case 'e'://End of Junction
    case 'E':
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

void autoMode() {
  Serial.println("Starting auto mode");
  bool blocked = false;
  while(blocked != true) {
    reflectanceSensors.readLine(sensorValues);
    if (sensorValues[2] > QTR_THRESHOLD || sensorValues[3] > QTR_THRESHOLD ) {
      // if the middle sensors detect line, stop
      blocked = true;
      Serial.println("Obstruction"); 
    } else if (sensorValues[0] > QTR_THRESHOLD) {
      // if leftmost sensor detects line, reverse and turn to the right
      motors.setSpeeds(0, -speed); 
    } else if (sensorValues[5] > QTR_THRESHOLD) {
      // if rightmost sensor detects line, reverse and turn to the left
      motors.setSpeeds(-speed, 0);
    } else {
      // otherwise, go straight
      forward(speed);
    }
    if(Serial.read() == '0') {
      blocked = true;
    }
  }
  Serial.println("Stopping");
  stop();
  //let the gui know we are ready for more instructions
  Serial.println("ready");
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
