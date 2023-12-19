/*  Program for development of RC remote on NRF24L01+ modules and OLED display.
    This program is the sender part of the RC remote. Designed to run on a Teensy LC.
    Date: 24-1-2023


    To-Do:
    -Adding interrupts for button presses, waiting for Teensy LC
    -2 way communication for battery voltage monitoring of the vehicle
*/

#include <Arduino.h>
#include <LibPrintf.h>
#include <EEPROM.h>

// OLED Display related
#include <U8g2lib.h> // https://github.com/olikraus/U8g2_Arduino
// OLED on Teensy:    A4(SDA), A5(SCL)
// I2C address OLED = 0x3C

// NRF24L01 related
#include <RF24.h> // https://github.com/tmrh20/RF24/
#include <nRF24l01.h>
/* NRF24L01 Teensy pinout:
 * CE   = D9
 * CSN  = D10
 * SCK  = D13
 * MOSI = D11
 * MISO = D12
 * IRO  = NC

 * NRF not working debug tips:
- Check hardware connections
- Check if CE and CSN defined correctly
- Check if address is consistent with both NRF24L01 modules
- Check if functions are called correctly
- Check if dataPackage is consistent with both NRF24L01 modules
- Check hardware module with testprogram for NRF24L01
*/

// Joysticks
const byte rightX = A0;
const byte rightY = A1;
const byte rightJoystickButton = 2;
const byte leftX = A2;
const byte leftY = A3;
const byte leftJoystickButton = 3;

// Buttons
const byte backButton = 4;
const byte ackButton = 5;
const byte auxButton1 = 6;
const byte auxButton2 = 7;

// Indicators
const byte sendLED = 8; // LED that indicates that data is being sent

// Battery voltage monitoring
const byte batteryValue = A6;

// Data types
struct dataPackage // Max size of this struct is 32 bytes, that's NRF24L01 buffer limit
{
  int16_t rightX = -1;              // 0...1023, -1 = uninitialized
  int16_t leftX = -1;               // 0...1023, -1 = uninitialized
  int16_t rightY = -1;              // 0...1023, -1 = uninitialized
  int16_t leftY = -1;               // 0...1023, -1 = uninitialized
  int8_t mode = 0;                  // 0 = idle, 1 = easy, 2 = pro, 3 = debug
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
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE); // 128x64 1.3 inch OLED, I2C, Uno, Nano, Mini Pro don't have enough RAM so use page_buffer
RF24 radio(9, 10);                                               // Divining CE and CSN pins

// Prototypes
void sendData(const byte mode);
void drawStartupScreen();
void drawMenu(byte *state);
void drawEasyScreen(byte *state);
void drawProScreen(byte *state);
void drawDebugScreen(byte *state);
void drawHeader(const char *menuName);
bool risingEdge(byte button);
void updateAccessoires();
void debugSerial();
void drawBasicInfo();
void drawEditProSettings();
void drawValueSet();
int readJoystick(byte joystick);

// Global variables
const uint64_t address[2] = {0xA40F7CA5F7LL, 0x32FA46D0E2LL}; // First address for communication to the vehicle, second address for communication to the remote. Second address unused for now.
const uint8_t *textFont = u8g2_font_6x13_tf;
const uint8_t *headerFont = u8g2_font_helvB10_te;
const byte idle = 0; // Statemachine states
const byte easy = 1;
const byte pro = 2;
const byte debug = 3;
const int joyStickLowTrigger = 400;  // Joystick trigger value on low side
const int joyStickHighTrigger = 600; // Joystick trigger value on high side
dataPackage txData;                  // Data to be sent to the vehicle

void setup()
{
  radio.begin(); // Start NRF24L01
  radio.openWritingPipe(address[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  oled.begin(); // Start OLED

  Serial.begin(9600); // For debugging purposes

  // Joysticks
  pinMode(rightJoystickButton, INPUT_PULLUP);
  pinMode(leftJoystickButton, INPUT_PULLUP);

  // Buttons
  pinMode(ackButton, INPUT_PULLUP);
  pinMode(backButton, INPUT_PULLUP);
  pinMode(auxButton1, INPUT_PULLUP);
  pinMode(auxButton2, INPUT_PULLUP);

  // Indicators
  pinMode(sendLED, OUTPUT);

  drawStartupScreen();
}

byte state = idle; // Start value of the statemachine
void loop()
{
  switch (state) // Statemachine
  {
  case idle:
    drawMenu(&state);
    break;
  case easy:
    drawEasyScreen(&state);
    break;
  case pro:
    drawProScreen(&state);
    break;
  case debug:
    drawDebugScreen(&state);
    break;
  }
}

unsigned long previousSend = 0; // Used to limit the send rate

// Gather all current statuses of all input devices and sends it to the RC car
void sendData(const byte mode)
{
  unsigned int sendDelay;

  if (mode == idle || mode == debug) // Send data slower in idle and debug mode
    sendDelay = 2000;
  else
    sendDelay = 0;
  if (millis() - previousSend >= sendDelay)
  {
    previousSend = millis();
    digitalWrite(sendLED, HIGH);
    txData.rightX = analogRead(rightX);
    txData.rightY = analogRead(rightY);
    txData.leftX = analogRead(leftX);
    txData.leftY = analogRead(leftY);
    txData.rightJoystickButton = !digitalRead(rightJoystickButton); // Invert the value because the button is pulled up
    txData.leftJoystickButton = !digitalRead(leftJoystickButton);   // Invert the value because the button is pulled up
    txData.ackButton = !digitalRead(ackButton);                     // Invert the value because the button is pulled up
    txData.backButton = !digitalRead(backButton);                   // Invert the value because the button is pulled up
    txData.auxButton1 = !digitalRead(auxButton1);                   // Invert the value because the button is pulled up
    txData.auxButton2 = !digitalRead(auxButton2);                   // Invert the value because the button is pulled up
    radio.write(&txData, sizeof(dataPackage));                      // Send data via NRF24L01
    digitalWrite(sendLED, LOW);
    // debugSerial(); // For debugging purposes
  }
}

// Draws a little startup annimation on the screen
void drawStartupScreen()
{
  txData.mode = idle;
  for (int y = -50; y < 80; y += 2) // Move the text down on the screen
  {
    if (risingEdge(ackButton)) // Skip startup annimation
    {
      return;
    }
    oled.firstPage(); // Start drawing process
    do
    {
      oled.setFont(u8g2_font_ncenB14_tr);
      oled.drawStr(50, y, "RC");
      oled.drawStr(15, y + 15, "Controller");
      oled.setFont(u8g2_font_luIS10_tf);
      oled.drawStr(25, y + 35, "By: Markus");
    } while (oled.nextPage()); // While still drawing
  }
}

byte menuOffset = 0; // Offset for the cursor position in the idle
// Prompts user with all control options of the vehicle. Returns selected mode by the user when choice is made
void drawMenu(byte *state)
{
  txData.mode = idle;
  const byte yDistance = 15; // Y-distance between objects (header object excluded)
  const byte xDistance = 10; // X-distance between objects

  while (true) // Loop until user has made a choice of control mode
  {
    sendData(idle);   // Send data to the RC car
    oled.firstPage(); // Start drawing process
    do
    {
      drawHeader("Menu");
      oled.setFont(textFont);
      oled.drawStr(0, (yDistance * 2 + menuOffset), ">");
      oled.drawStr(xDistance, yDistance * 2, "Easy");
      oled.drawStr(xDistance, yDistance * 3, "Pro");
      oled.drawStr(xDistance, yDistance * 4, "Debug");

    } while (oled.nextPage()); // While still drawing

    // Check for user joystick input
    int joystickValue = readJoystick(rightY);
    if (joystickValue > joyStickHighTrigger && menuOffset < 2 * yDistance)
    {
      menuOffset += yDistance;
    }
    else if (joystickValue < joyStickLowTrigger && menuOffset > 1)
    {
      menuOffset -= yDistance;
    }

    // Check for user mode choice
    if (risingEdge(ackButton))
    {
      if (menuOffset == 0)
        *state = easy;
      else if (menuOffset == yDistance)
        *state = pro;
      else
        *state = debug;
      return;
    }
  }
}

// Draws all information for a beginning user
void drawEasyScreen(byte *state)
{
  txData.mode = easy;
  txData.throttleSensitifity = 40;
  txData.steerSensitifity = 50;

  while (risingEdge(backButton) == false) // Stay in this mode until the user presses the back button
  {
    updateAccessoires(); // Reading all input devices and updating the car features, like: lights, horn, etc.
    sendData(easy);      // Send data to the RC car
    oled.firstPage();    // Start drawing process
    do
    {
      drawHeader("Easy");
      drawBasicInfo();         // Draws all basic information needed for the user
    } while (oled.nextPage()); // While still drawing
  }
  *state = idle; // Return to idle mode
}

// Draws all information for a advanced user
void drawProScreen(byte *state)
{
  txData.mode = pro;
  txData.throttleSensitifity = EEPROM.read(0);        // Read throttle sensitivity from EEPROM
  txData.steerSensitifity = EEPROM.read(1);           // Read steering sensitivity from EEPROM
  const byte yDistance = oled.getDisplayHeight() / 4; // Y-distance between objects (header object excluded)
  const byte xDistance = oled.getDisplayWidth() / 2;  // X-distance between objects

  while (risingEdge(backButton) == false) // Stay in this mode until the user presses the back button
  {
    updateAccessoires(); // Reading all input devices and updating the car features, like: lights, horn, etc.
    sendData(pro);       // Send data to the RC car
    oled.firstPage();    // Start drawing process
    do
    {
      drawHeader("Pro");
      drawBasicInfo();                  // Draws all basic information needed for the user
      oled.setCursor(0, yDistance * 4); // Add advanced information to the screen
      oled.print("TH: ");
      oled.print(txData.throttleSensitifity);
      oled.print("%");
      oled.setCursor(xDistance, yDistance * 4);
      oled.print("ST: ");
      oled.print(txData.steerSensitifity);
      oled.print("%");
    } while (oled.nextPage()); // While still drawing

    if (risingEdge(ackButton)) // If user presses the acknowledge button, enter edit mode
      drawEditProSettings();
  }
  *state = idle; // Return to idle mode
}

// Draws all information for the developer on the screen
void drawDebugScreen(byte *state)
{
  txData.mode = debug;
  const byte yDistance = oled.getDisplayHeight() / 4; // Y-distance between objects (header object excluded)
  const byte xDistance = oled.getDisplayWidth() / 3;  // X-distance between objects
  bool page = 0;                                      // Keeps track of which page the user is on

  while (risingEdge(backButton) == false) // Stay in this mode until the user presses the back button
  {
    sendData(debug);  // Send data to the RC car
    oled.firstPage(); // Start drawing process
    do
    {
      drawHeader("Debug");
      oled.setFont(textFont);
      if (page == 0) // First page is information about joysticks and battery voltages
      {
        oled.setCursor(0, yDistance * 2);
        oled.print((String) "LX:" + analogRead(leftX));
        oled.setCursor(xDistance + 5, yDistance * 2);
        oled.print((String) "LY:" + analogRead(leftY));
        oled.setCursor(xDistance * 2 + 10, yDistance * 2);
        oled.print((String) "LSW:" + digitalRead(leftJoystickButton));
        oled.setCursor(0, yDistance * 3);
        oled.print((String) "RX:" + analogRead(rightX));
        oled.setCursor(xDistance + 5, yDistance * 3);
        oled.print((String) "RY:" + analogRead(rightY));
        oled.setCursor(xDistance * 2 + 10, yDistance * 3);
        oled.print((String) "RSW:" + digitalRead(rightJoystickButton));
        oled.setCursor(0, yDistance * 4);
        oled.print((String) "RA:" + (analogRead(batteryValue)));
        oled.setCursor(xDistance + 5, yDistance * 4);
        oled.print((String) "VA:" + "NC");
      }
      else // Second page is about the auxiliary buttons
      {
        oled.setCursor(0, yDistance * 2);
        oled.print((String) "AB1:" + digitalRead(auxButton1));
        oled.setCursor(xDistance + 5, yDistance * 2);
        oled.print((String) "AB2:" + digitalRead(auxButton2));
      }
    } while (oled.nextPage()); // While still drawing

    // Switch infomation tabs when rightY joystick is moved
    int joystickValue = readJoystick(leftX);
    if (joystickValue > joyStickHighTrigger || joystickValue < joyStickLowTrigger) // If the rightY joystick is moved, switch pages
    {
      page = !page;
    }
  }
  *state = idle; // Return to idle mode
}

// Draws on the first row of every screen the name of the current page
void drawHeader(const char *menuName)
{
  oled.setFont(headerFont);
  const byte y = 15;
  const byte x = ((oled.getDisplayWidth() - (oled.getUTF8Width(menuName))) / 2);
  oled.drawStr(x, y, menuName);
  oled.drawHLine(0, y + 1, oled.getDisplayWidth());
}

bool lastButtonState[] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Keeps track of the last button state
// Rising edge detection function for all buttons
bool risingEdge(byte button)
{
  byte index;
  switch (button) // Assigning index to a button
  {
  case rightJoystickButton:
    index = 0;
    break;
  case leftJoystickButton:
    index = 1;
    break;
  case ackButton:
    index = 2;
    break;
  case backButton:
    index = 3;
    break;
  case auxButton1:
    index = 4;
    break;
  case auxButton2:
    index = 5;
    break;
  default: // If invalid button is passed, print error message and return false
    printf("Error: risingEdge() function called with invalid button argument.");
    return false;
  }

  bool buttonState = digitalRead(button);
  if (buttonState != lastButtonState[index]) // Detecting change
  {
    lastButtonState[index] = buttonState; // Updating lastButtonState
    if (buttonState == LOW)               // Button pressed
    {
      return true; // Rising edge detected
    }
  }
  delay(2);     // Debounce
  return false; // No rising edge detected
}

// Reading all input devices and updating the car features, like: lights, horn, etc.
void updateAccessoires()
{
  if (digitalRead(rightJoystickButton) == LOW)
    txData.honk = true;
  else
    txData.honk = false;

  if (risingEdge(leftJoystickButton))
    txData.headLight = !txData.headLight;

  if (risingEdge(auxButton2))
    txData.tailLight = !txData.tailLight;

  if (digitalRead(auxButton1) == LOW)
    txData.brake = true;
  else
    txData.brake = false;
}

unsigned long lastSerial = 0; // Keeps track of the last time serial data was sent
// Prints all data that also will be sent to the RC car to the serial monitor. Used for debugging purposes
void debugSerial()
{
  if (millis() - lastSerial > 1000)
  {
    lastSerial = millis(); // Updating lastSerial
    printf("\n\n\n");
    printf("mode: %i\n", txData.mode);
    printf("rightX: %i\n", txData.rightX);
    printf("rightY: %i\n", txData.rightY);
    printf("leftX: %i\n", txData.leftX);
    printf("leftY: %i\n", txData.leftY);
    printf("rightJoystickButton: %i\n", txData.rightJoystickButton);
    printf("leftJoystickButton: %i\n", txData.leftJoystickButton);
    printf("ackButton: %i\n", txData.ackButton);
    printf("backButton: %i\n", txData.backButton);
    printf("auxButton1: %i\n", txData.auxButton1);
    printf("auxButton2: %i\n", txData.auxButton2);
    printf("brake: %i\n", txData.brake);
    printf("honk: %i\n", txData.honk);
    printf("headLight: %i\n", txData.headLight);
    printf("tailLight: %i\n", txData.tailLight);
    printf("throttleSensitifity: %i\n", txData.throttleSensitifity);
    printf("steeringSensitifity: %i\n", txData.steerSensitifity);
  }
}

// Draws all basic information needed for the user
void drawBasicInfo()
{
  const byte yDistance = oled.getDisplayHeight() / 4; // Y-distance between each object
  const byte xDistance = oled.getDisplayWidth() / 2;  // X-distance between each object
  oled.setFont(textFont);
  if (txData.headLight == HIGH) // Draw headlight status
    oled.drawStr(0, yDistance * 2, "HL: On");
  else
    oled.drawStr(0, yDistance * 2, "HL: Off");

  if (txData.tailLight == HIGH) // Draw taillight status
    oled.drawStr(xDistance, yDistance * 2, "TL: On");
  else
    oled.drawStr(xDistance, yDistance * 2, "TL: Off");
  oled.setCursor(0, yDistance * 3);
  oled.print("RV: "); // Draw battery voltage of the remote
  oled.print((analogRead(batteryValue) * 0.003225287) * 3, 1);
  oled.print("V");
  oled.setCursor(xDistance, yDistance * 3);
  oled.print("VV: "); // Draw battery voltage of the vehicle
  oled.print(12.6, 1);
  oled.print("V");
}

// Draws the menu for editing the throttle and steering sensitivity in pro mode
void drawEditProSettings()
{
  txData.mode = idle;                                               // Make sure the vehicle doesn't run away while editing the settings
  unsigned long previousBlink = 0;                                  // Keeps track of the last time the selected value blinked
  unsigned long scrollCooldown = 0;                                 // Slows the scrollling of possible values of the selected value
  bool showCurrentValue = true;                                     // Keeps track of whether the selected value should be shown or not
  bool valueHighlighted = false;                                    // Keeps track of whether the selected value is highlighted or not
  bool page = 0;                                                    // Keeps track of the current page (0 = throttle, 1 = steering)
  byte buffer = txData.throttleSensitifity;                         // Keeps track of the current value of the selected item
  byte yDistance = oled.getDisplayHeight() / 4;                     // Y-distance between each object
  String header[] = {"Edit Throttle", "Edit Steering"};             // Header for each page
  String row1[] = {"Throttle Sensitivity", "Steering Sensitivity"}; // Row 1 for each page
  String row2[] = {"TR = ", "ST = "};                               // Row 2 for each page

  while (true) // Loop until user presses the back button
  {
    sendData(debug);  // Send data to the RC car
    oled.firstPage(); // Start drawing process
    do
    {
      drawHeader(header[page].c_str());
      oled.setFont(textFont);
      byte x = ((oled.getDisplayWidth() - (oled.getUTF8Width(row1[page].c_str()))) / 2);                     // Calculate the x-position of the row 1
      oled.drawStr(x, yDistance * 2.3, row1[page].c_str());                                                  // Draw row 1
      x = ((oled.getDisplayWidth() - (oled.getUTF8Width(((String)row2[page] + buffer + "%").c_str()))) / 2); // Calculate the x-position of the row 2
      oled.setCursor(x, yDistance * 3.3);
      oled.print(row2[page]); // Draw row 2
      if (showCurrentValue)   // Draw the current value of the selected item
        oled.print((String)buffer + "%");
    } while (oled.nextPage()); // While still drawing

    // Blinking of value when selected
    if ((millis() - previousBlink > 700) && valueHighlighted) // If the selected value should blink
    {
      previousBlink = millis();             // Reset the blinking timer
      showCurrentValue = !showCurrentValue; // Toggle the current selected value
    }

    // Scrolling of the selected value
    if ((millis() - scrollCooldown >= 100)) // If the scroll cooldown has expired, allow joystick input again
    {
      scrollCooldown = millis();                                                        // Reset the scroll cooldown
      if (valueHighlighted && (analogRead(rightY) > joyStickHighTrigger) && buffer > 5) // If the selected value is highlighted, value doesn't go below 5% and the joystick is moved to the up
      {
        buffer -= 5;
        showCurrentValue = true;
        previousBlink = millis();
      }
      else if (valueHighlighted && (analogRead(rightY) < joyStickLowTrigger) && buffer < 100) // If the selected value is highlighted, value doesn't exceed 100% and the joystick is moved to the down
      {
        buffer += 5;
        showCurrentValue = true;
        previousBlink = millis();
      }
    }

    // Button input
    if (risingEdge(ackButton)) // If the user presses the select button on an item
    {
      if (valueHighlighted == false) // Highlight the selected value
        valueHighlighted = true;
      else // Save the selected value
      {
        valueHighlighted = false;    // Unhighlight the selected value
        showCurrentValue = true;     // Make sure the current value isn't hidden
        EEPROM.update(page, buffer); // Save the selected value to the EEPROM
        if (page == 0)               // First page is about throttle
          txData.throttleSensitifity = buffer;
        else // Second page is about steering
          txData.steerSensitifity = buffer;
        drawValueSet();
      }
    }
    else if (risingEdge(backButton)) // If the user presses the back button
    {
      if (valueHighlighted == false) // If the user presses the back button and no value is highlighted, return to pro screen
      {
        txData.mode = pro;
        break;
      }
      else // If the user presses the back button and a value is highlighted, unhighlight the value
      {
        valueHighlighted = false;
        showCurrentValue = true;
        buffer = EEPROM.read(page);
      }
    }

    // Switch edit tabs when rightY joystick is moved
    int joystickValue = readJoystick(leftY);
    if ((joystickValue > joyStickHighTrigger || joystickValue < joyStickLowTrigger) && valueHighlighted == false) // If the rightY joystick is moved, switch pages
    {
      page = !page;
      if (page == 0) // First page is about throttle
        buffer = txData.throttleSensitifity;
      else // Second page is about steering
        buffer = txData.steerSensitifity;
    }
  }
}

// Let the user know that the value has been set
void drawValueSet()
{
  const char *prompt = "Value Set!"; // Prompt to be drawn
  oled.setFont(headerFont);
  byte x = (oled.getDisplayWidth() - (oled.getUTF8Width(prompt))) / 2; // Calculate the x-position of the prompt
  byte y = oled.getDisplayHeight() / 2 + 5;                            // Calculate the y-position of the prompt
  oled.firstPage();                                                    // Start drawing process
  do
  {
    oled.drawStr(x, y, prompt);
  } while (oled.nextPage()); // While still drawing
  delay(500);                // Message is shown for a small amount of time
}

// Read joystick input for menu navigation. If joystick is not homed, output will be suppressed
bool joystickHomed[] = {true, true, true, true};
int readJoystick(byte joystick)
{
  byte index;

  switch (joystick)
  {
  case leftY:
    index = 0;
    break;
  case leftX:
    index = 1;
    break;
  case rightY:
    index = 2;
    break;
  case rightX:
    index = 3;
    break;
  default:
    printf("Error: readJoystick() function called with invalid joystick argument.");
    return 512;
  }

  if ((analogRead(joystick) < joyStickHighTrigger) && (analogRead(joystick) > joyStickLowTrigger)) // Prevent endless switching between pages when joystick is moved
    joystickHomed[index] = true;

  if ((analogRead(joystick) > joyStickHighTrigger || analogRead(joystick) < joyStickLowTrigger) && joystickHomed[index] == true)
  {
    joystickHomed[index] = false;
    return analogRead(joystick);
  }
  return 512;
}