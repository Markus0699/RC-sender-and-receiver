/*  Program for developement of an homemade RC-car. This program will control the hardware of the vehicle.
    The vehicle will be controlled by a remote control that will send data to this microcontroller.
    This program is designed to run on a Arduino Pro Mini.
    Date: 24-1-2022


    To-Do:
    - Validation for booleans
    - Battery voltage reading
    - Battery voltage sending to remote
*/

#include <Arduino.h>
#include <LibPrintf.h>
#include <Servo.h>

// NRF24L01 related
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h> // https://github.com/tmrh20/RF24/
/* NRF24L01 Pro Mini pinout:
 * CE   = 7
 * CSN  = 8
 * SCK  = 13
 * MOSI = 11
 * MISO = 12
 * IRO  = NC

 * NRF not working debug tips:
- Check hardware connections
- Check if CE and CSN defined correctly
- Check if address is consistent with both NRF24L01 modules
- Check if functions are called correctly
- Check if dataPackage is consistent with both NRF24L01 modules
- Check hardware module with testprogram for NRF24L01
*/

// Lights
const byte receivedLED = 6;     // LED that lights up when data is received
const byte interferenceLED = 4; // LED that lights up when received data is not within the expected range
const byte headLight = A1;
const byte tailLight = A2;

// Horn
const byte horn = 9;

// Battery voltage monitoring
const byte batteryValue = A3;

// Data types
struct dataPackage // Max size of this struct is 32 bytes, that's NRF24L01 buffer limit
{
  int16_t rightX = -1;              // 0...1023, -1 = uninitialized
  int16_t leftX = -1;               // 0...1023, -1 = uninitialized
  int16_t rightY = -1;              // 0...1023, -1 = uninitialized
  int16_t leftY = -1;               // 0...1023, -1 = uninitialized
  int8_t mode = 4;                  // 0 = idle, 1 = easy, 2 = pro, 3 = debug, 4 = not connected
  int8_t throttleSensitifity = -1;  // 0...100, -1 = uninitialized
  int8_t steerSensitifity = -1;     // 0...100, -1 = uninitialized
  bool rightJoystickButton = false; // 0 = released, 1 = pressed
  bool leftJoystickButton = false;  // 0 = released, 1 = pressed
  bool ackButton = false;           // 0 = released, 1 = pressed
  bool backButton = false;          // 0 = released, 1 = pressed
  bool auxButton1 = false;          // 0 = released, 1 = pressed
  bool auxButton2 = false;          // 0 = released, 1 = pressed
  bool brake = false;               // 0 = off, 1 = on
  bool honk = false;                // 0 = off, 1 = on
  bool headLight = false;           // 0 = off, 1 = on
  bool tailLight = false;           // 0 = off, 1 = on
};

// Objects
RF24 radio(7, 8);      // CE, CSN
Servo motorcontroller; // Controls the speed of the vehicle
Servo servo;           // Controls the steering of the vehicle

// Prototypes
void debugReceivedSerial();
void debugStatusSerial();
void waitForRemote();
void idleMode();
void easyMode();
void proMode();
void debugMode();
bool validateData(dataPackage data);
void updateAccessoires();
void receiveData();
void updatePwmDevices();
void isConnected();

// Global variables
const byte idle = 0;  // Statemachine options
const byte easy = 1;  // Statemachine options
const byte pro = 2;   // Statemachine options
const byte debug = 3; // Statemachine options
const byte notConnected = 4;
const uint64_t address[2] = {0xA40F7CA5F7LL, 0x32FA46D0E2LL}; // First address for communication from the vehicle, second address for communication to the vehicle. Second address unused for now.
dataPackage rawData;                                          // Create a variable with the above structure
dataPackage rxData;                                           // Create a variable with the above structure

void setup()
{
  radio.begin(); // Start NRF24L01
  radio.openReadingPipe(0, address[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  Serial.begin(9600); // For debugging purposes

  // Lights
  pinMode(receivedLED, OUTPUT);
  pinMode(interferenceLED, OUTPUT);
  pinMode(headLight, OUTPUT);
  pinMode(tailLight, OUTPUT);

  // PWM devices
  motorcontroller.attach(3);
  servo.attach(5);
}

void loop()
{
  switch (rxData.mode) // Statemachine
  {
  case notConnected:
    waitForRemote();
    break;
  case idle:
    idleMode();
    break;
  case easy:
    easyMode();
    break;
  case pro:
    proMode();
    break;
  case debug:
    debugMode();
    break;
  }
}

unsigned long headLightBlink = 0; // Makes the headlight and tail light blink when in idle mode
// Function that is called when the vehicle is not connected to the remote
void waitForRemote()
{
  motorcontroller.write(90);
  servo.write(90);
  receiveData();
  // debugStatusSerial();                  // DEBUG
  if (millis() - headLightBlink > 1500) // Show that the vehicle is in idle mode by blinking the headlight and tail light
  {
    headLightBlink = millis();
    digitalWrite(headLight, !digitalRead(headLight));
    digitalWrite(tailLight, !digitalRead(tailLight));
  }
}

// Function that is called when the vehicle is in idle mode
void idleMode()
{
  receiveData();
  isConnected();
  // debugStatusSerial(); // DEBUG
  updateAccessoires();
  motorcontroller.write(90); // Stop the motor
}

// Function that is called when the vehicle is in easy mode
void easyMode()
{
  receiveData();
  isConnected();
  updateAccessoires();
  updatePwmDevices();
}

// Function that is called when the vehicle is in pro mode
void proMode()
{
  receiveData();
  isConnected();
  // debugStatusSerial(); // DEBUG
  updateAccessoires();
  updatePwmDevices();
}

// Function that is called when the vehicle is in debug mode
void debugMode()
{
  receiveData();
  isConnected();
}

unsigned long lastReceive = 0; // The time the last data was received
// Function that checks if the vehicle is still connected to the remote
void isConnected()
{
  if (millis() - lastReceive > 3000)
  {
    rxData.mode = notConnected;
  }
}

// Check if data is received, if so read the data and validate
void receiveData()
{
  if (radio.available()) // Data received
  {
    digitalWrite(receivedLED, HIGH);           // Turn on the received LED
    lastReceive = millis();                    // Make a timestamp for the last time data was received
    radio.read(&rawData, sizeof(dataPackage)); // Read data
    // debugReceivedSerial();                     // For debugging purposes
    if (validateData(rawData)) // Check if data is valid
    {
      if (rxData.mode == notConnected) // Play a sound when remote vehicle picks up communication with remote
      {
        tone(horn, 220, 500);
        delay(500);
        tone(horn, 880, 500);
        delay(1000);
      }
      rxData = rawData; // Transfer received data to rxData
    }
    else
      digitalWrite(interferenceLED, HIGH); // Data is invalid, turn on the interference LED
    digitalWrite(receivedLED, LOW);        // Turn off the received LED
  }
}

// Checks if the data received from the remote is valid
bool validateData(dataPackage check)
{
  // Check if all values are within the valid range, if not return false
  // Haven't found a good way to check for booleans
  if (check.rightX > 1023 || check.rightX < -1)
  {
    printf("case1\n");
    return false;
  }
  if (check.rightY > 1023 || check.rightY < -1)
  {
    printf("case2\n");
    return false;
  }
  if (check.leftX > 1023 || check.leftX < -1)
  {
    printf("case3\n");
    return false;
  }
  if (check.leftY > 1023 || check.leftY < -1)
  {
    printf("case4\n");
    return false;
  }
  if (check.mode > 3 || check.mode < 0)
  {
    printf("case5\n");
    return false;
  }
  if (check.throttleSensitifity > 100 || check.throttleSensitifity < -1)
  {
    printf("case6\n");
    return false;
  }
  if (check.steerSensitifity > 100 || check.steerSensitifity < -1)
  {
    printf("case7\n");
    return false;
  }
  return true;
}

// Update hardware features based on the data received from the remote. For example head lights, tail lights, horn, etc.
void updateAccessoires()
{
  if (rxData.headLight)
    digitalWrite(headLight, HIGH);
  else
    digitalWrite(headLight, LOW);

  if (rxData.tailLight)
    digitalWrite(tailLight, HIGH);
  else
    digitalWrite(tailLight, LOW);

  if (rxData.honk)
  {
    tone(horn, 220, 500);
  }

  if (rxData.brake)
  {
    motorcontroller.write(0);
  }
}

// Update the servo and motorcontroller with the most recent data
void updatePwmDevices()
{
  const byte upperBoundary = 180; // The upper boundary of the servo
  const byte middlepoint = upperBoundary / 2;

  // printf("\n\n\n"); // DEBUG
  unsigned int max = middlepoint + (float)middlepoint / 100 * rxData.steerSensitifity; // Calculate the maximum value of steering
  unsigned int min = middlepoint - (float)middlepoint / 100 * rxData.steerSensitifity; // Calculate the minimum value of steering
  unsigned int steerPosition = map(rxData.leftX, 0, 1023, min, max);                   // Calculate the position of the servo
  servo.write(steerPosition);                                                          // Update the servo
  // printf("Steer max: %i, min: %i, pos: %i\n", max, min, steerPosition); // DEBUG

  max = middlepoint + (float)middlepoint / 100 * rxData.throttleSensitifity; // Calculate the maximum value of throttle
  min = middlepoint - (float)middlepoint / 100 * rxData.throttleSensitifity; // Calculate the minimum value of throttle
  unsigned int throttle = map(rxData.rightY, 0, 1023, min, max);             // Calculate the value of the throttle
  motorcontroller.write(throttle);                                           // Update the motor controller
  // printf("Throttle max: %i, min: %i, pos: %i\n", max, min, throttle); // DEBUG
}

unsigned long lastSerial = 0; // Keeps track of the last time serial data was sent
// Prints all relevant data that has been received from the remote to the serial monitor. Used for debugging purposes
void debugReceivedSerial()
{
  if (millis() - lastSerial > 1000)
  {
    lastSerial = millis(); // Updating lastSerial
    printf("\n\n\n");
    printf("Received data:\n");
    printf("mode: %i\n", rawData.mode);
    printf("rightX: %i\n", rawData.rightX);
    printf("rightY: %i\n", rawData.rightY);
    printf("leftX: %i\n", rawData.leftX);
    printf("leftY: %i\n", rawData.leftY);
    printf("rightButton: %i\n", rawData.rightJoystickButton);
    printf("leftButton: %i\n", rawData.leftJoystickButton);
    printf("ackButton: %i\n", rawData.ackButton);
    printf("backButton: %i\n", rawData.backButton);
    printf("auxButton1: %i\n", rawData.auxButton1);
    printf("auxButton2: %i\n", rawData.auxButton2);
    printf("brake: %i\n", rawData.brake);
    printf("honk: %i\n", rawData.honk);
    printf("headLight: %i\n", rawData.headLight);
    printf("tailLight: %i\n", rawData.tailLight);
    printf("throttleSensitifity: %i\n", rawData.throttleSensitifity);
    printf("steeringSensitifity: %i\n", rawData.steerSensitifity);
  }
}

unsigned long lastStatusSerial = 0; // Keeps track of the last time serial data was sent
// Prints all relevant data of the status of the vehicle to the serial monitor. Used for debugging purposes
void debugStatusSerial()
{
  if (millis() - lastStatusSerial > 1000)
  {
    lastStatusSerial = millis(); // Updating lastSerial
    printf("\n\n\n");
    printf("Vehicle status:\n");
    printf("mode: %i\n", rxData.mode);
    printf("rightX: %i\n", rxData.rightX);
    printf("rightY: %i\n", rxData.rightY);
    printf("leftX: %i\n", rxData.leftX);
    printf("leftY: %i\n", rxData.leftY);
    printf("rightButton: %i\n", rxData.rightJoystickButton);
    printf("leftButton: %i\n", rxData.leftJoystickButton);
    printf("ackButton: %i\n", rxData.ackButton);
    printf("backButton: %i\n", rxData.backButton);
    printf("auxButton1: %i\n", rxData.auxButton1);
    printf("auxButton2: %i\n", rxData.auxButton2);
    printf("brake: %i\n", rxData.brake);
    printf("honk: %i\n", rxData.honk);
    printf("headLight: %i\n", rxData.headLight);
    printf("tailLight: %i\n", rxData.tailLight);
    printf("throttleSensitifity: %i\n", rxData.throttleSensitifity);
    printf("steeringSensitifity: %i\n", rxData.steerSensitifity);
  }
}