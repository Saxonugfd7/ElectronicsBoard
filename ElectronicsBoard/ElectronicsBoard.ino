// SD Card Module
#include <SPI.h>
#include <SD.h>

// DC Motor & Motor Module - L298N
#include <L298N.h>

// Pin definition
const unsigned int IN1 = 7;
const unsigned int IN2 = 8;
const unsigned int EN = 9;

// Create one motor instance
L298N motor(IN1, IN2);

// Moisture Sensor
#define moisturePin A5

// Line Sensor
#define lineSensorPin 3    // Line Sensor (light). HIGH or LOW values.

// GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
SoftwareSerial ss(4, 3);

// Real Time Clock (RTC)
#include "RTClib.h"
RTC_Millis rtc;     // Software Real Time Clock (RTC)
DateTime rightNow;  // used to store the current time.

// Traffic Lights - LED Outputs
#define ledRed A0
#define ledYellow A1
#define ledGreen A2

// Sonar - HC-SR04
#define echoPin 22 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 23 //attach pin D3 Arduino to pin Trig of HC-SR04

//Potentiometer
#define pot A3

// Servo
#include <Servo.h>
Servo myservo;

// Piezo Buzzer
#define piezoPin 5

// Crash Sensor / Button
#define crashSensor 7

// SD Card
#define SDpin 53

// IR Remote
#include "Adafruit_NECremote.h"
#define IRpin         24
Adafruit_NECremote remote(IRpin);

void setup() {
  Serial.begin(9600);           // Open serial communications and wait for port to open:
  while (!Serial) {
    delay(1);                   // wait for serial port to connect. Needed for native USB port only
  }

  // SD Card initialisation
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDpin)) {
    Serial.println("initialization failed!");
    while (1);
  }

  // Real Time Clock (RTC)
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));

  Serial.println("initialization done.");
  logEvent("System Initialisation...");

  // Traffic Lights - LED Outputs
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  // GPS
  ss.begin(4800);

  //Potentiometer
  pinMode(pot, INPUT);

  // Piezo Buzzer
  pinMode(piezoPin, OUTPUT);

  // DC Motor & Motor Module - L298N
  motor.setSpeed(70);

  // Moisture Sensor
  pinMode(moisturePin, INPUT);

  // Servo
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  // Line Sensor
  pinMode(lineSensorPin, OUTPUT);

  // Sonar - HC-SR04
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  //Built in LED
  pinMode(13, OUTPUT);

  // Crash Sensor / Button
  pinMode(crashSensor, INPUT);
  SPI.begin();
}

void loop() {
  motorDC();
  doorAlarm();
  //  remoteDecode();
  delay(250);

}


String remoteDecode() {
  int c = remote.listen(5);  // seconds to wait before timing out!
  // Or you can wait 'forever' for a valid code
  //int c = remote.listen();  // Without a #, it means wait forever
  if (c >= 0) {
    switch (c) {

        Serial.println("Code is :" + c);
      // Top keys
      case 70:
        Serial.println("UP");
        break;
      case 21:
        Serial.println("DOWN");
        break;
      case 68:
        Serial.println("LEFT");

        break;
      case 67:
        Serial.println("RIGHT");
        break;
      case 64:
        Serial.println("OK");
        break;
      // Numbers
      case 22:
        Serial.println("1");
        break;
      case 25:
        Serial.println("2");
        break;
      case 13:
        Serial.println("3");
        break;
      case 12:
        Serial.println("4");
        break;
      case 24:
        Serial.println("5");
        break;
      case 94:
        Serial.println("6");
        break;
      case 8:
        Serial.println("7");
        break;
      case 28:
        Serial.println("8");
        break;
      case 90:
        Serial.println("9");
        break;
      case 82:
        Serial.println("0");
        break;

      // # and *
      case 66:
        Serial.println("*");
        break;
      case 74:
        Serial.println("#");
        break;


      // otherwise...

      default:
        Serial.println("Code is :" + c);
        break;
    }

  }
}

int getSonarDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;
}

void doorAlarm() {
  int doorThreshold = 10;
  int doorDistance = getSonarDistance();
  if (doorDistance < doorThreshold) {
    tone(piezoPin, 1000); // Send 1KHz sound signal...
    delay(100);
    tone(piezoPin, 500); // Send 1KHz sound signal...
    delay(100);
    String distanceStr = String(doorDistance);
    String thresholdStr = String(doorThreshold);
    String eventToLog = "Alarm activated, " + distanceStr + "<" + thresholdStr;
    logEvent(eventToLog);
  } else {
    noTone(piezoPin);
  }
}


void motorDC() {
  //  motor.forward();
  //
  //  // Alternative method:
  //  // motor.run(L298N::FORWARD);
  //
  //  //print the motor status in the serial monitor
  //  printSomeInfo();
  //
  //  delay(3000);
  //
  //  // Stop
  //  motor.stop();
  //
  //  // Alternative method:
  //  // motor.run(L298N::STOP);
  //
  //  printSomeInfo();
  //
  //  // Change speed
  //  motor.setSpeed(255);
  //
  //  delay(3000);
  //
  //  // Tell the motor to go back (may depend by your wiring)
  //  motor.backward();
  //
  //  // Alternative method:
  //  // motor.run(L298N::BACKWARD);
  //
  //  printSomeInfo();
  //
  //  motor.setSpeed(120);
  //
  //  delay(3000);
  //
  //  // Stop
  //  motor.stop();
  //
  //  printSomeInfo();
  //
  //  delay(3000);
}

/*
  Print some informations in Serial Monitor
*/
void printSomeInfo()
{
  Serial.print("Motor is moving = ");
  Serial.print(motor.isMoving());
  Serial.print(" at speed = ");
  Serial.println(motor.getSpeed());
}



void logEvent(String dataToLog) {
  /*
     Log entries to a file on an SD card.
  */
  // Get the updated/current time
  DateTime rightNow = rtc.now();

  // Open the log file
  File logFile = SD.open("events.csv", FILE_WRITE);
  if (!logFile) {
    Serial.print("Couldn't create log file");
    abort();
  }

  // Log the event with the date, time and data
  logFile.print(rightNow.year(), DEC);
  logFile.print(",");
  logFile.print(rightNow.month(), DEC);
  logFile.print(",");
  logFile.print(rightNow.day(), DEC);
  logFile.print(",");
  logFile.print(rightNow.hour(), DEC);
  logFile.print(",");
  logFile.print(rightNow.minute(), DEC);
  logFile.print(",");
  logFile.print(rightNow.second(), DEC);
  logFile.print(",");
  logFile.print(dataToLog);

  // End the line with a return character.
  logFile.println();
  logFile.close();
  Serial.print("Event Logged: ");
  Serial.print(rightNow.year(), DEC);
  Serial.print(",");
  Serial.print(rightNow.month(), DEC);
  Serial.print(",");
  Serial.print(rightNow.day(), DEC);
  Serial.print(",");
  Serial.print(rightNow.hour(), DEC);
  Serial.print(",");
  Serial.print(rightNow.minute(), DEC);
  Serial.print(",");
  Serial.print(rightNow.second(), DEC);
  Serial.print(",");
  Serial.println(dataToLog);
}
