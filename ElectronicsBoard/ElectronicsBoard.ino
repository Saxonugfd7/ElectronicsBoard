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
L298N motor(EN, IN1, IN2);

// Moisture Sensor
#define moisturePin A5

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

// Bluetooth - nRF8001
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

String command = "";

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    This function is called whenever select ACI events happen
*/
/**************************************************************************/
void aciCallback(aci_evt_opcode_t event)
{
  switch (event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      break;
    default:
      break;
  }
}

/**************************************************************************/
/*!
    This function is called whenever data arrives on the RX channel
*/
/**************************************************************************/
void rxCallback(uint8_t *buffer, uint8_t len)
{
  Serial.print(F("Received "));
  Serial.print(len);
  Serial.print(F(" bytes: "));
  for (int i = 0; i < len; i++) {
    //Serial.print((char)buffer[i]);
    command += (char)buffer[i];
  }


  // MARK: Calculates if it's a !B command.

  Serial.println("Command: " + command);

  String newCommand = "";
  newCommand += command.charAt(0);
  newCommand += command.charAt(1);

  Serial.println("Here it is!: " + newCommand + "<end>");

  if (newCommand == "!B") {
    Serial.println("Controller Command received.");
  }




  //TODO: get the code for each button and programme it.
  // Each button has a press down state and a let go state.
  // This will be useful to continue to run a command until the user lets go of a button.



  Serial.print(F(" ["));

  for (int i = 0; i < len; i++)
  {
    Serial.print(" 0x"); Serial.print((char)buffer[i], HEX);
  }
  Serial.println(F(" ]"));




  command = "";
  /* Echo the same data back! */
  uart.write(buffer, len);
}

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
  Serial.println("initialization done.");
  logEvent("System Initialisation...");

  // Traffic Lights - LED Outputs
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  // Bluetooth - nRF8001
  uart.setRXcallback(rxCallback);
  uart.setACIcallback(aciCallback);
  uart.setDeviceName("RyanBLE"); /* 7 characters max! */
  uart.begin();

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


// Sonar - HC-SR04
pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
}

void loop() {
  // put your main code here, to run repeatedly:

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
