#include <ZumoShield.h>
#include <NewPing.h>

#define TRIGGER_PIN  6 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define QTR_THRESHOLD 400
// constants for compass 
#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

LSM303 compass;
ZumoMotors motors;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
ZumoReflectanceSensorArray reflectanceSensors;
// Define an array for holding sensor values.
#define NUM_SENSORS 6
unsigned int sensorValues[NUM_SENSORS];

int speed = 200; // max speed is 255
bool object = false;
bool started = false;
bool returning = false;
int ends = 0;

void setup(void)
{
  calibrateRSA();
  calibrateCompass();
  
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
      startOrEnd();
      break;
    case 'C'://continue
    case 'c':
      autoMode();
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

void startOrEnd(){
  //flag that we have started
  started = !started;
  if (started == false) {
    returning = false;
    digitalWrite(13, LOW);
  } 
}

void endOfJunction() {
  //TODO: Task 5 and 6
  ends++;
  if(ends < 2) {
    // TODO: Task5
  } else {
    // Task 6; TODO: alt behaviour on rooms
    ends = 0;
    //flag returning
    returning = true;
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
  delay(400);
  // move forward a bit 
  forward(speed);
  delay(400);
  stop();
  scanRoom();
  Serial.println("ready");
}

void roomLeft() {
  // turn left 90 degrees
  left(speed);
  // tune this to get turn right
  delay(400);
  // move forward a bit 
  forward(speed);
  delay(400);
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
  long endTime = currentTime + 2500;
  while(currentTime < endTime) {
    currentTime = millis();
    int distance = sonar.ping_cm();
    if(distance <= 20) {
      object = true;
    }
  }
  stop();
  if(object == true) {
     if(!returning) {
        Serial.println("Object in room");
     } else {
      digitalWrite(13, HIGH);
     }
  }
  Serial.flush();
  clearSerial();
}

// Clear the serial, Dont want to queue commands while the arduino is busy.
void clearSerial() {
  while(Serial.available()) {
    Serial.read();
  }
}


// RSA code comes from Calibrate sensor example of Zumo library
void calibrateRSA() {
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
}

// All compass code was adapted from the compass example of the zumo library mostly the same however
// its also a complete mess
void calibrateCompass() {
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  unsigned char index;
  // Initiate LSM303
  compass.init();
  // Enables accelerometer and magnetometer
  compass.enableDefault();
  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(speed);
  motors.setRightSpeed(-speed);
  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;
}

// Converts x and y components of a vector to a heading in degrees.
// This function is used instead of LSM303::heading() because we don't
// want the acceleration of the Zumo to factor spuriously into the
// tilt compensation that LSM303::heading() performs. This calculation
// assumes that the Zumo is always level.
template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}


void turnDegrees(int angle) {
    float heading, relative_heading;
    int comSpeed;
    static float target_heading = target_heading = fmod(averageHeading() + angle, 360);
    while(abs(relative_heading) < 2) {
      // Heading is given in degrees away from the magnetic vector, increasing clockwise
      heading = averageHeading();
      // This gives us the relative heading with respect to the target angle
      relative_heading = relativeHeading(heading, target_heading);
      // To avoid overshooting, the closer the Zumo gets to the target
      // heading, the slower it should turn. Set the motor speeds to a
      // minimum base amount plus an additional variable amount based
      // on the heading difference.
      comSpeed = speed*relative_heading/180;
      if (comSpeed < 0)
        comSpeed -= (speed * 0.5);
        else
        comSpeed += (speed * 0.5);
      motors.setSpeeds(comSpeed, -comSpeed);
    }
    // Turn off motors and wait a short time to reduce interference from motors
    motors.setSpeeds(0, 0);
    delay(100);
}
