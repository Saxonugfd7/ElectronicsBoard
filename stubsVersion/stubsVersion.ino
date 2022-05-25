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
#define echoPin 6 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin A4 //attach pin D3 Arduino to pin Trig of HC-SR04

//Potentiometer
#define pot A3

// Servo
#include <Servo.h>
Servo myservo;

// Piezo Buzzer
#define piezoPin 8

// Crash Sensor / Button
#define crashSensor 7

// Adafruit nRF8001 Module
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 40
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 41
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void setup() {
  Serial.begin(9600);           // Open serial communications and wait for port to open:
  while (!Serial) {
    delay(1);                   // wait for serial port to connect. Needed for native USB port only
  }

  // SD Card initialisation
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
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

  // Bluetooth - nRF8001
  BTLEserial.setDeviceName("Mega"); /* 7 characters max! */
  BTLEserial.begin();

  // GPS
  ss.begin(4800);

  //Potentiometer
  pinMode(pot, INPUT);

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
}

void loop() {
  bluetoothConnectivity();
  motorDC();
  doorAlarm();
}

/*
  Reads distance value from Sonar (HC-SR04). 
  If less than threshold, activate Piezo buzzer
  @param None
  @return void
*/
void doorAlarm() {

}

/*
  Test Code for DC Motor Usage
  @param None
  @return void
*/
void motorDC() {

}

/*
  Print some information on the motor state in Serial Monitor
  @param None
  @return void
*/
void printSomeInfo()
{

}

/*
    Takes command entered from Bluetooth connection and executes functionality
    E.g. "1" - Write HIGH to builtin LED
    "0" - Write LOW to builtin LED
    @param bleCommand - string accepted from BLE Uart Connection
    @return void
*/
void bluetoothCommandReceived(String bleCommand) {

}

/*
    Handles BLE connectivity.
    Taken from library functionality.
    @param None
    @return void
*/
void bluetoothConnectivity() {

}

/*
    Log entries to a file on an SD card, and outputs to Serial Monitor
    @param dataToLog - string to save on SD card, timestamped.
    @return void
*/
void logEvent(String dataToLog) {


}
