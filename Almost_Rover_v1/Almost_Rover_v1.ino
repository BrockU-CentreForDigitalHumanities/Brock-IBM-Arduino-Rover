/*
   Code developed for IBM by Interactive Arts and Science 3P03
   Brock University

   component code from Arduino Examples and the following sources:
   HC-SR04 Ping distance sensor:
   Original (10 Sept 2011): http://www.trollmaker.com/article3/arduino-and-hc-sr04-ultrasonic-sensor
   Modified (15 May 2012): http://winkleink.blogspot.com.au/2012/05/arduino-hc-sr04-ultrasonic-distance.html
   Modified (10 Nov 2012): http://arduinobasics.blogspot.com.au/2012/11/arduinobasics-hc-sr04-ultrasonic-sensor.html
*/

// Lines beginning with # are preprocessor directives
// #include includes another source code file
// #define is a macro definition

// For including SD Card reader libraries
#include <SPI.h>
#include <SD.h>
// Servo pin is set in the Setup function
#include <Servo.h>

/* ===========================================================
   Custom section! You can change the values of the parameters
   in this section to visibly affect the actions of the robot!
   ===========================================================
*/

// Starting angle offset from hardware startpoint -30 to 30
#define SERVO_START_OFFSET -20
// Change the speed of the servo sweep
#define SERVO_SPEED 5 //5 is fast, 1 is slow
// Number of degrees to sweep infront of the rover
#define SERVO_ARC 135
// Speed is a value between 0-255 representing
// the relative speed of the motor
#define FORWARD_SPEED 200
// Log level is a value between 1 and 3 which determines what
// logging to do to the SD card
#define LOG_LEVEL 2


// The number of milliseconds (1/1000th of a second) to reverse for
#define REVERSE_TIME 500
// Range at which rover avoids by backing up and turning
#define AVOID_RANGE 10
// Range at which rover avoids by turning away from obstical
#define TURN_RANGE 40
// Time spent turning when required
#define AVOID_TURN_TIME 10

//REMOVE FOR FINAL CODE RELEASE
#define debug true
/*
   If LOG_LEVEL is greater or equal to 2 then we run the code
   in between the #if and #endif
   #if LOG_LEVEL >= 2
   #endif
   LOG_LEVEL 1
     Only record time when avoid
   LOG_LEVEL 2
     Record status every 180 ticks
   LOG_LEVEL 3
     Record status every 18 ticks
*/

#if LOG_LEVEL == 2
  #define LOG_TICK 90
#elif LOG_LEVEL == 3
  #define LOG_TICK 18
#else
  #define LOG_TICK 255 // Never
#endif

/* ===========================================================
   End custom section
   ===========================================================
*/
// Left Motor terminal 1 - Negative (forward) to Motor Control Chip 6
// Arduino Digital Pin 2 to Motor Control Chip 7 
#define MOTOR_PIN_A 2 
// Left Motor terminal 2 -  Positive (forward) to Motor Control Chip 3
// Arduino Digital Pin 3 to Motor Control Chip 2
#define MOTOR_PIN_B 3 
// Right Motor terminal 1 - Negative (forward) to Motor Control Chip 14
// Arduino Digital Pin 4 to Motor Control Chip 15
#define MOTOR_PIN_C 4 
// Right Motor terminal 2 - Positive (forward) to Motor Control Chip 11
// Arduino Digital Pin 5 to Motor Control Chip 10
#define MOTOR_PIN_D 5 
// Sound and LED pin plays the audio when we're avoiding obsticals
#define SOUND_PIN 6

#define ECHO_PIN 8 // Echo Pin
#define TRIG_PIN 9 // Trigger Pin

#define CHIP_SELECT 10 // SD card pin CS

#define MAXIMUM_RANGE 150 // If range is greater than this we continue forward
#define MINIMUM_RANGE 0 // If range is less than this we continue forward



#define DATA_FILE "log-data.txt"
// Servo object
Servo myServo;

// Represents the number of times loop() has been run
// byte is just an int that can take values between 0-255
byte i;

/*
   setup() is run once when the arduino starts
*/
void setup() {
  // Open serial port (if we're connected to computer for debugging)
  Serial.begin (9600);

  // For servo, used attach method rather than setting pinMode
  myServo.attach(7);

  // Set pinmode to output for all motor pins
  pinMode(MOTOR_PIN_A, OUTPUT); // Set pin 2 as output
  pinMode(MOTOR_PIN_B, OUTPUT); // Set pin 3 as output
  pinMode(MOTOR_PIN_C, OUTPUT); // Set pin 4 as output
  pinMode(MOTOR_PIN_D, OUTPUT); // Set pin 5 as output

  pinMode(SOUND_PIN, OUTPUT);

  // Set distance sensor pins Tigger/OUT, Echo/IN
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set ChipSelect for SD Card
  pinMode(10, OUTPUT);
  i = 1;
  // Initialize SD Card
  initSDCard();

  // Necessary to generate truely random sequences
  // we grab some random value from pin 0 which due to
  // background electromagnetic noise (from radio stations even!) 
  // will have some value!
  randomSeed(analogRead(0));

  // Successful initialization play start up sound
  startupIndicator();
}



/*
   loop() is a function run by the arduino continuously
   restarting each time it reaches the end
*/
void loop() {
  // Servo position
  static int servoPos;
  // Distance to obstacle
  static int distanceToObs = 0;

  // Sweep back and forth between 30 and 120
  // This variable pos can take values between 00 and 180
  // It smoothly moves from 0 -> 180, then back 180 -> 00

  // abs(i) returns the absolute value of i the absolute value
  // is the value of a number without the sign(always positive)
  // This whole section is just a series of transformations that make the arm sweep from
  // 90 - SERVO_ARC to 90 + SERVO_ARC it also does not stutter when i == 180
  servoPos = 90 + (abs(((double)(i * SERVO_SPEED % 180) / 180) - 0.5) * 2 - 0.5) * SERVO_ARC;
  if (debug) {
    Serial.print("Servo Pos: ");
    Serial.println(servoPos);
  }

  // Move the servo to the position pos + SERVO_START_OFFSET
  // from the 0 degree position on the servo
  myServo.write(servoPos + SERVO_START_OFFSET);

  // Allow time for servo to catch up.
  delay(1);

  // Send out a ping be sure to set the default timout to
  // some number within the max range (3000 microseconds is 50cm)
  distanceToObs = testForDistance();

  if (debug) {
    Serial.print("Distance: ");
    Serial.println(distanceToObs);
  }
  // If the distance detected by the sensors is greater than the maximum range
  // or less than the minimum range move forward
  if (distanceToObs >= MAXIMUM_RANGE || distanceToObs <= MINIMUM_RANGE) {
    forward();
  } else {
    if (distanceToObs < AVOID_RANGE) {
      avoid(servoPos);
    } else if (distanceToObs <= TURN_RANGE && servoPos < 90) {
      turnRight();
      delay(AVOID_TURN_TIME);
    } else if (distanceToObs <= TURN_RANGE && servoPos >= 90) {
      turnLeft();
      delay(AVOID_TURN_TIME);
    } else {
      forward();
    }
  }

  // Increase the position of the servo by one degree
  i++;
  if (debug) {
    Serial.print("i: ");
    Serial.println(i);
  }
  // This corrects for overflow errors by not incrementing int i past its maximum.
  if ((i % 180) == 0)
    i = 1;
}


String initWithTimeStamp() {
  // create a timestamp at he beginning of a string
  String dataString = "";
  // Store as number of seconds
  static unsigned long times; 
  times = millis() / 1000;
  // hours
  dataString += (unsigned long)(((double)times / 60) / 60) % 24;
  dataString += ":";
  // minutes
  dataString += (unsigned long)((double)times / 60) % 60;
  dataString += ":";
  // seconds
  dataString += (times % 60);
  return dataString;
}

String addData(String dataString, String newdata) {
  dataString += ",";
  dataString = dataString + newdata;
  return dataString;
}

void writeToDataFile(String dataString) {
  File dataFile = SD.open(DATA_FILE, FILE_WRITE);

  // If the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // Print to the serial port too:
    if (debug) {
      Serial.print("Data String to SD Card: ");
      Serial.println(dataString);
    }
  } else {
    // If the file isn't open, pop up an error:
    Serial.println("Error opening datalog.txt on SD card");
  }
}

void chirp() {
  tone(SOUND_PIN, 440.0, 100);
  delay(100);
  tone(SOUND_PIN, 1280, 100);
  delay(100);
  tone(SOUND_PIN, 680, 100);
  delay(100);
  digitalWrite(SOUND_PIN, LOW);
}

/*
   Function that is used to return the distance that the sensor is reporting
   @return the distance in cm to the surface the ultrasonic sensor is sensing
*/
int testForDistance() {
  static long distance; // Distance to surface
  static long duration; // Duration used to calculate distance
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 3000);

  // Calculate the distance (in cm) based on the speed of sound.
  distance = duration / 58.2;

  return distance;
}

/*
   Avoid obstacles by turning the opposite direction of the
   detected obstacles
   @param p the position of the sensors when we sense the obstacle.
   The value is a value of 0-180 in degrees.
*/
void avoid(unsigned int p) {
  stopAll();
  reverse();
  delay(REVERSE_TIME);
  chirp();
  stopAll();
  log("avoiding");
  if (p >= 90) {
    turnLeft();
  } else {
    turnRight();
  }
  delay(AVOID_TURN_TIME);
  stopAll();
  forward();
}


/*
  Functions for Movement
  - Forward - Low, High, Low, High
  - Reverse - High, Low, High, Low
  - Left - Low, High, High, Low
  - Right - High, Low, Low, High
*/
void forward() {
  // Stop wheel one from turning
  digitalWrite(MOTOR_PIN_A, LOW);
  // Wheel one starts moving forward at FORWARD_SPEED
  analogWrite(MOTOR_PIN_B, FORWARD_SPEED);
  // Stop wheel two from turning
  digitalWrite(MOTOR_PIN_C, LOW);
  // Wheel two starts moving forward at FORWARD_SPEED
  analogWrite(MOTOR_PIN_D, FORWARD_SPEED);
  // Log if necessary
  log("forward");
}

void reverse() {
  digitalWrite(MOTOR_PIN_A, HIGH);
  digitalWrite(MOTOR_PIN_B, LOW);
  digitalWrite(MOTOR_PIN_C, HIGH);
  digitalWrite(MOTOR_PIN_D, LOW);
  // We log in avoid, since reverse is only called
  // within avoid we can log in that function
}

void turnLeft() {
  digitalWrite(MOTOR_PIN_A, LOW);
  digitalWrite(MOTOR_PIN_B, HIGH);
  digitalWrite(MOTOR_PIN_C, HIGH);
  digitalWrite(MOTOR_PIN_D, LOW);
  log("left");
}

void turnRight() {
  digitalWrite(MOTOR_PIN_A, HIGH);
  digitalWrite(MOTOR_PIN_B, LOW);
  digitalWrite(MOTOR_PIN_C, LOW);
  digitalWrite(MOTOR_PIN_D, HIGH);
  log("right");
}

void stopAll() {
  digitalWrite(MOTOR_PIN_A, LOW);
  digitalWrite(MOTOR_PIN_B, LOW);
  digitalWrite(MOTOR_PIN_C, LOW);
  digitalWrite(MOTOR_PIN_D, LOW);
}

/*
   Overloaded function that runs log but doesn't always log
*/
void log(String status) {
  log(status, false);
}

/*
   We only use log if LOG_LEVEL is greater then or equal to 2
   It takes a status and only logs if the tick counter is equal
   to the LOG_TICK value (180 or 18)
   @param status specify the status to log
   @param always if this is true, always log the status
   even if its not the correct time to do so
*/
void log(String status, bool always) {
  // Only run log every so often based on the logging level
  // or if we have explicitly said to always log this message
  // avoid for instance, is always logged
  if (always || (i % LOG_TICK == 0)) {
    String result = initWithTimeStamp();
    result = addData(result, status);
    writeToDataFile(result);
  }
}


void initSDCard() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
  } else {
    Serial.println("Card initialized.");
  }
}

void startupIndicator() {
  // Generate arpeggio based on a random number
  // You can customize this!
  static int randomFirstNote = random(200, 1200);

  for (int j = 0; j < 8; j++) {
    tone(SOUND_PIN, randomFirstNote + (((j + 4) % 8) * 150), 100);
    delay(100);
    digitalWrite(SOUND_PIN, LOW);
  }
}

