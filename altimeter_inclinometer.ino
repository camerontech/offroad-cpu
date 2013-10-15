#include <Wire.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

const int VERSION = 1;

unsigned long millisCounter = 0;
unsigned long minMaxAltitudeMillsCounter = 0;

// Menu and display modes
const String menuText[][2] = {{"         Incline","                "},
                              {"        Altitude","                "},
                              {"           Track","        Altitude"},
                              {"         Min/Max","        Altitude"},
                              {"       Calibrate","        Altitude"},
                              {"            Zero","         Incline"},
                              {"     Temperature","                "},
                              {"             Set","      Brightness"},
                              {"         Factory","           Reset"}};
const unsigned int INCLINE = 0;
const unsigned int ALTITUDE = 1;
const unsigned int TRACK = 2;
const unsigned int MINMAX = 3;
const unsigned int CALIBRATE = 4;
const unsigned int ZERO = 5;
const unsigned int TEMPERATURE = 6;
const unsigned int BRIGHTNESS = 7;
const unsigned int RESET = 8;
const unsigned int MENU = 9;
const unsigned int MENU_LENGTH = 9;

// Keep track of where we are
unsigned int mode;
int displayMenuItem;

// Display in (m)etric or (i)mperial
char unit;

// Display character for brightness meter
const unsigned int BLOCK_CHAR = 255;
const int MAX_BRIGHTNESS = 255;
const int MIN_BRIGHTNESS = 1;
const int BRIGHTNESS_INCREMENT = 32;
int brightness;


/////////////////////
// Three-way button
/////////////////////
const unsigned int UP = 12;
const unsigned int PUSH = 11;
const unsigned int DOWN = 10;

int unsigned lastState = 0;
int unsigned buttonState = 0;


//////////////////////
// Altimeter
//////////////////////

// analog input pins
const unsigned int X_PIN = A0;
const unsigned int Y_PIN = A1;
const unsigned int Z_PIN = A2;

// max/min analog values

// Duemilanove
// xMax:753 xMin:234 yMax:784 yMin:266 zMax:743 zMin:221
// const int X_MIN = 234;
// const int X_MAX = 753;
// const int Y_MIN = 266;
// const int Y_MAX = 784;
// const int Z_MIN = 221;
// const int Z_MAX = 743;

// Leonardo
// xMax:732 xMin:239 yMax:775 yMin:271 zMax:732 zMin:238
// const int X_MIN = 284;
// const int X_MAX = 778;
// const int Y_MIN = 308;
// const int Y_MAX = 815;
// const int Z_MIN = 247;
// const int Z_MAX = 753;

// Uno
// xMax:738 xMin:230 yMax:767 yMin:255 zMax:781 zMin:271
const unsigned int X_MAX = 741;
const unsigned int X_MIN = 223;
const unsigned int Y_MAX = 768;
const unsigned int Y_MIN = 255;
const unsigned int Z_MAX = 802;
const unsigned int Z_MIN = 280;

// So that we can zero out the inclinometer readings
int lastPitch, lastRoll;
int pitchOffset, rollOffset;

// ASCII character for degree symbol
const unsigned int DEGREE_CHAR = 223;


//////////////////////
// Barometer
//////////////////////

const int BMP085_ADDRESS = 0x77;  // I2C address of BMP085

const unsigned char OSS = 3;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;

short temperature;
long pressure;

// Use these for altitude conversions
const float p0 = 101325;     // Pressure at sea level (Pa)

// User-defined offset based some something like GPS readings
float calibrateAltitudeOffset = 0;
float calibrateAltitudeDisplay;

// When user wants to track altitude change we start with the current altitude
float trackingAltitudeOffset = 0;

// Keep track of minimum and maximum altitude
float minAltitude;
float maxAltitude;

// convert from meters to feet
float metersToFeet = 3.28084;




////////////////////
// Display
////////////////////

// --- SPECIAL COMMAND DEFINITIONS
const int BACKLIGHT_COMMAND = 128;    // 0x80
const int SPECIAL_COMMAND = 254;      // 0xFE
const int BAUD_COMMAND = 129;         // 0x81

// --- ARDUINO PIN DEFINITIONS
uint8_t RSPin = 0;
uint8_t RWPin = 1;
uint8_t ENPin = 4;
uint8_t D4Pin = 5;
uint8_t D5Pin = 6;
uint8_t D6Pin = 7;
uint8_t D7Pin = 8;
uint8_t BLPin = 9;

char inKey;                     // Character received from serial input
uint8_t Cursor = 0;             // Position of cursor, 0 is top left, (rows*columns)-1 is bottom right
uint8_t LCDOnOff = 1;           // 0 if LCD is off
uint8_t blinky = 0;             // Is 1 if blinky cursor is on
uint8_t underline = 0;          // Is 1 if underline cursor is on
uint8_t splashScreenEnable = 1; // 1 means splash screen is enabled
uint8_t rows = 2;               // Number rows, will be either 2 or 4
uint8_t columns = 16;           // Number of columns, will be 16 or 20
uint8_t characters;             // rows * columns

// initialize the LCD at pins defined above
LiquidCrystal lcd(RSPin, RWPin, ENPin, D4Pin, D5Pin, D6Pin, D7Pin);



// EEPROM memory addresses

// display
const int LCD_BACKLIGHT_ADDRESS = 1;  // backlight setting
const int BAUD_ADDRESS = 2;           // Baud rate setting
const int SPLASH_SCREEN_ADDRESS = 3;  // splash screen on/off
const int ROWS_ADDRESS = 4;           // number of rows
const int COLUMNS_ADDRESS = 5;        // number of columns

const int VERSION_ADDRESS = 10;
const int UNIT_ADDRESS = 11;
const int PITCH_OFFSET_ADDRESS = 12;
const int ROLL_OFFSET_ADDRESS = 13;
const int MODE_ADDRESS = 14;
//const int CALIBRATE_ALTITUDE_OFFSET_ADDRESS = 16;
//const int TRACKING_ALTITUDE_OFFSET_ADDRESS = 32;


void setup() {
  //Serial.begin(9600);

  // Use 3.3v as our analog reference since that's what our
  // altimeter outputs as a max
  analogReference(EXTERNAL);

  // Set EEPROM variables if the version changes
  if (EEPROM.read(VERSION_ADDRESS) != VERSION) {
    factoryReset();
  }

  setupVariables();
  setupDisplay();
  setupButton();
  setupAccelerometer();
  setupBarometer();
}

void loop() {

  buttonCheck();

  // updateMinMaxAltitude();
  saveMode();

  if (mode == INCLINE) {
    loopInclinometer();
  } else if (mode == ALTITUDE) {
    loopAltimeter();
  } else if (mode == TRACK) {
    loopTrack();
  } else if (mode == MINMAX) {
    loopMinMax();
  } else if (mode == CALIBRATE) {
    loopCalibrate();
  } else if (mode == TEMPERATURE) {
    loopTemperature();
  } else if (mode == BRIGHTNESS) {
    loopBrightness();
  } else if (mode == MENU) {
    loopMenu();
  }
}


////////////
// Setups //
////////////

void factoryReset() {
  EEPROM.write(UNIT_ADDRESS, 1);
  EEPROM.write(PITCH_OFFSET_ADDRESS, 0);
  EEPROM.write(ROLL_OFFSET_ADDRESS, 0);
  EEPROM.write(MODE_ADDRESS, INCLINE);
  //EEPROM.write(CALIBRATE_ALTITUDE_OFFSET_ADDRESS, 0);
  //EEPROM.write(TRACKING_ALTITUDE_OFFSET_ADDRESS, 0);

  EEPROM.write(VERSION_ADDRESS, VERSION);
}

// Saves the current mode to EEPROM
void saveMode() {
  EEPROM.write(MODE_ADDRESS, mode);
}

void setupVariables() {

  if (EEPROM.read(UNIT_ADDRESS) == 0) {
    unit = 'm';
  } else {
    unit = 'i';
  }

  pitchOffset = EEPROM.read(PITCH_OFFSET_ADDRESS);
  rollOffset = EEPROM.read(ROLL_OFFSET_ADDRESS);
  mode = displayMenuItem = EEPROM.read(MODE_ADDRESS);
  //calibrateAltitudeOffset = EEPROM.read(CALIBRATE_ALTITUDE_OFFSET_ADDRESS) / 10.0;
  //calibrateAltitudeDisplay = getAltitude() - calibrateAltitudeOffset;
  //trackingAltitudeOffset = EEPROM.read(TRACKING_ALTITUDE_OFFSET_ADDRESS) / 10.0;

}

void setupDisplay() {
  lcd.begin(16, 2);

  // Set up the backlight
  pinMode(BLPin, OUTPUT);
  brightness = EEPROM.read(LCD_BACKLIGHT_ADDRESS);
  // setBacklight(EEPROM.read(LCD_BACKLIGHT_ADDRESS));
  setBrightness();

  // Splash screen
  lcd.print("  Cameron Tech  ");
  delay(2000);
  lcd.clear();
  lcd.print(" Inclinometer & ");
  moveToSecondLine();
  lcd.print("   Altimeter    ");
  delay(2000);
  lcd.clear();
}


void setupButton() {
  pinMode(UP, INPUT);
  pinMode(PUSH, INPUT);
  pinMode(DOWN, INPUT);

  // Set pull-up resistors
  digitalWrite(UP, HIGH);
  digitalWrite(PUSH, HIGH);
  digitalWrite(DOWN, HIGH);
}


void setupAccelerometer() {
}


void setupBarometer() {
  Wire.begin();
  bmp085Calibration();
  resetMinMaxAltitude();
}


void buttonCheck() {
  if (digitalRead(UP) == LOW) {
    buttonState = UP;
  } else if (digitalRead(PUSH) == LOW) {
    buttonState = PUSH;
  } else if (digitalRead(DOWN) == LOW) {
    buttonState = DOWN;
  } else {
    buttonState = 0;
  }

  if (buttonState != lastState) {
    if (buttonState != 0) {
      buttonClick();
    }
    lastState = buttonState;
  }

  // debounce delay
  delay(25);
}


void buttonClick() {
  if (mode == MENU) {
    if (buttonState == DOWN) {
      displayMenuItem++;
      if (displayMenuItem == MENU_LENGTH) {   // rollover to beginning of list
        displayMenuItem = 0;
      }
    } else if (buttonState == UP) {
      displayMenuItem--;
      if (displayMenuItem < 0) {              // rollover to end of list
        displayMenuItem = MENU_LENGTH - 1;
      }
    } else if (buttonState == PUSH) {
      if (displayMenuItem == ZERO) {          // viewing the "zero incline" menu option
        zeroInclinometer();
        mode = INCLINE;
        displayMenuItem = INCLINE;
      } else if (displayMenuItem == RESET) {  // Factory Reset feature
        factoryReset();
        setupVariables();
      } else{
        mode = displayMenuItem;               // select menu item
        startMode();
      }
    }
  } else {
    if (mode == CALIBRATE) {  // when in calibrate mode, up/down change values, push goes back to menu
      if (buttonState == UP) {
        incrementAltimeterCalibration();
      } else if (buttonState == DOWN) {
        decrementAltimeterCalibration();
      } else if (buttonState == PUSH) {
        saveAltitudeCalibration();
        displayMenuItem = ALTITUDE;
        mode = ALTITUDE;
      }
    } else if (mode == BRIGHTNESS) { // when in brightness mode, up/down changes brightness, push goes back to menu
      if (buttonState == UP) {
        increaseBrightness();
      } else if (buttonState == DOWN) {
        decreaseBrightness();
      } else {
        saveBrightness();
        mode = MENU;
      }
    } else {                  // when not in calibrate mode, up/down goes to menu, push is optional function
      if (buttonState == UP || buttonState == DOWN) {        // Button pressed up or down to go into the menu
        mode = MENU;
      } else if (mode == TRACK && buttonState == PUSH) {     // Button clicked while tracking altitude
        resetTrackingAltitude();
      } else if (mode == MINMAX && buttonState == PUSH) {    // Button clicked while viewing min/max
        resetMinMaxAltitude();
      } else if (mode == ALTITUDE || mode == TEMPERATURE && buttonState == PUSH) { // Button clicked on altimeter
        // Switch between meters and feet
        switchUnit();
      }
    }
  }
}

// Called when a menu item is selected but before looping through that mode
void startMode() {
  if (mode == CALIBRATE) {
    startCalibration();
  }
}




////////////
// Loops  //
////////////

void loopMenu() {
  if (millis() - millisCounter > 250) {
    lcd.clear();
    lcd.print(menuText[displayMenuItem][0]);
    moveToSecondLine();
    lcd.print(menuText[displayMenuItem][1]);

    resetCounter();
  }
}

void loopInclinometer() {
  if (millis() - millisCounter > 250) {

    // sample the voltages
    delay(10);
    int unsigned x = analogRead(X_PIN);
    delay(10);
    int unsigned y = analogRead(Y_PIN);
    delay(10);
    int unsigned z = analogRead(Z_PIN);

    // convert to range of -90 to +90 degrees
    int xAng = map(x, X_MIN, X_MAX, -90, 90);
    int yAng = map(y, Y_MIN, Y_MAX, -90, 90);
    int zAng = map(z, Z_MIN, Z_MAX, -90, 90);

    // convert radians to degrees
    int pitch = -(RAD_TO_DEG * (atan2(-yAng, -zAng) + PI));
    int roll = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);

    // convert left roll and forward pitch to negative degrees
    if (pitch < -180) {
      pitch = pitch + 360;
    }
    if (roll > 180) {
      roll = roll - 360;
    }

    // TODO: actually sample pitch/roll at zero time
    lastPitch = pitch;
    lastRoll = roll;

    // Take into account any offset
    pitch -= pitchOffset;
    roll -= rollOffset;

    // Now we need to re-add the offset in case we try to roll around 180/-180 again
    if (pitch < -180) {
      pitch += 360;
    } else if (pitch > 180) {
      pitch -= 360;
    }
    if (roll < -180) {
      roll += 360;
    } else if (roll > 180) {
      roll -= 360;
    }

    // int correctedPitch = map(pitch, 0, 360, -180, 180);
    // int correctedRoll = map(roll, 0, 360, -180, 180);

    // write the pitch and roll to the second line
    displayIncline(pitch, roll);

    resetCounter();
  }
}


void loopAltimeter() {
  if (millis() - millisCounter > 1000) {

    moveToFirstLine();
    lcd.print("    Altitude    ");
    moveToSecondLine();
    outputAltitudeLine(false, NULL);

    resetCounter();
  }
}

void loopTrack() {
  if (millis() - millisCounter > 1000) {
    moveToFirstLine();
    lcd.print(" Track Altitude ");
    moveToSecondLine();
    outputAltitudeLine(true, NULL);

    resetCounter();
  }
}

void loopMinMax() {
  if (millis() - millisCounter > 1000) {
    moveToFirstLine();
    lcd.print("   Min    Max   ");
    moveToSecondLine();

    // Output minimum altitude
    int beforeMin = center(altitudeWidth(minAltitude), 8) / 16;
    for (int i=0; i<beforeMin; i++) {
      lcd.print(" ");
    }

    displayAltitudeWithUnit(minAltitude);

    int afterMin = center(altitudeWidth(minAltitude), 8) % 16;
    for (int i=0; i<afterMin; i++) {
      lcd.print(" ");
    }

    // Output maximum altitude
    int beforeMax = center(altitudeWidth(maxAltitude), 8) / 16;
    for (int i=0; i<beforeMax; i++) {
      lcd.print(" ");
    }

    displayAltitudeWithUnit(maxAltitude);

    int afterMax = center(altitudeWidth(maxAltitude), 8) % 16;
    for (int i=0; i<afterMax; i++) {
      lcd.print(" ");
    }

    resetCounter();
  }
}

void loopCalibrate() {
  if (millis() - millisCounter > 250) {
    moveToFirstLine();
    lcd.print("Current Altitude");
    moveToSecondLine();

    if (calibrateAltitudeDisplay == NULL) {
      calibrateAltitudeDisplay = getAltitude();
    }
    outputAltitudeLine(false, calibrateAltitudeDisplay);
    resetCounter();
  }
}

void loopTemperature() {
  if (millis() - millisCounter > 1000) {
    moveToFirstLine();
    lcd.print("  Temperature   ");
    moveToSecondLine();

    temperature = bmp085GetTemperature(bmp085ReadUT());

    lcd.print("     ");

    if (unit == 'i') {
      lcd.print(temperature*0.18+32,1);
      lcd.write(DEGREE_CHAR);
      lcd.write("F");
    } else {
      lcd.print(temperature/10.0,1);
      lcd.write(DEGREE_CHAR);
      lcd.write("C");
    }
    lcd.print("      ");
    resetCounter();
  }
}

void loopBrightness() {
  if (millis() - millisCounter > 250) {

    moveToFirstLine();
    lcd.print("   Brightness   ");
    moveToSecondLine();

    int unsigned dots = map(brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS, 1, 16);
    for (int i=0; i<dots; i++) {
      lcd.write(BLOCK_CHAR);
    }
    for (int i=0; i<16-dots; i++) {
      lcd.write(" ");
    }
    resetCounter();
  }
}




//////////////////////////////
// Random helpers
//////////////////////////////

// Returns a number that has the left and right padding values "encoded" into it.
// Divide by 16 to get the left padding and MOD by 16 to get the right padding.
//
// 71 means 4 characters to the left (0x47 / 16) and 7 to the right (0x47 % 16)
int center(int chars, int width) {
  int left, right;
  left = right = (width - chars) / 2;
  right += (width - chars) % 2;

  return (left * 16) + right;
}



//////////////////////////////
// Counters
//////////////////////////////

// Resets the millisecond counter
void resetCounter() {
  millisCounter = millis();
}

// A separate counter that keeps track of when we record min/max altitude
// behind the scenes
void resetMinMaxCounter() {
  minMaxAltitudeMillsCounter = millis();
}




//////////////////////////////
// Display functions
//////////////////////////////

void moveToFirstLine() {
  lcd.setCursor(0,0);
}


void moveToSecondLine() {
  lcd.setCursor(0,1);
}


void increaseBrightness() {
  brightness += BRIGHTNESS_INCREMENT;
  if (brightness > MAX_BRIGHTNESS) {
    brightness = MAX_BRIGHTNESS;
  }
  setBrightness();
}


void decreaseBrightness() {
  brightness -= BRIGHTNESS_INCREMENT;
  if (brightness < MIN_BRIGHTNESS) {
    brightness = MIN_BRIGHTNESS;
  }
  setBrightness();
}


void setBrightness() {
  analogWrite(BLPin, brightness);
  // EEPROM.write(LCD_BACKLIGHT_ADDRESS, brightness);
}


void saveBrightness() {
  EEPROM.write(LCD_BACKLIGHT_ADDRESS, brightness);
}



//////////////////////////////
// Incline functions
//////////////////////////////

void zeroInclinometer() {
  pitchOffset = lastPitch;
  EEPROM.write(PITCH_OFFSET_ADDRESS, pitchOffset);
  rollOffset = lastRoll;
  EEPROM.write(ROLL_OFFSET_ADDRESS, rollOffset);
}


// Writes the current pitch/roll to the screen
void displayIncline(int first, int second) {
  // Move to the start of the second line
  moveToFirstLine();
  lcd.print("  Pitch   Roll  ");
  moveToSecondLine();

  // convert int values to strings for output
  String firstString = String(first);
  String secondString = String(second);

  // pad spaces before pitch value
  String output = "  ";
  if (firstString.length() < 4) {
    output += " ";
  }
  if (firstString.length() == 1) {
    output += " ";
  }
  output += firstString;

  // write pitch value
  lcd.print(output);
  lcd.write(DEGREE_CHAR);

  int outputLength = output.length() + 1;

  // pad spaces before pitch value
  output = "";
  for (int i=outputLength; i<10; i++) {
    output += " ";
  }
  if (firstString.length() < 3) {
    output += " ";
  }
  if (secondString.length() == 1) {
    output += " ";
  }
  output += secondString;

  // pad spaces before roll value
  lcd.print(output);
  lcd.write(DEGREE_CHAR);

  outputLength += output.length() + 1;

  // fill the rest of the line with blanks
  for (int i=outputLength; i<16; i++) {
    lcd.print(" ");
  }

}




//////////////////////////////
// Altitude functions
//////////////////////////////

// Zeros out the tracking altitude
void resetTrackingAltitude() {
  trackingAltitudeOffset = getAltitude() + calibrateAltitudeOffset;
  // EEPROM.write(TRACKING_ALTITUDE_OFFSET_ADDRESS, trackingAltitudeOffset * 10);
}


void resetMinMaxAltitude() {
  minAltitude = getAltitude();
  maxAltitude = minAltitude;
}


// Save min/max values every second
void updateMinMaxAltitude() {
  if (millis() - minMaxAltitudeMillsCounter > 1000) {
    float altitude = getAltitude();

    if (altitude > maxAltitude) {
      maxAltitude = altitude;
    }
    if (altitude < minAltitude) {
      minAltitude = altitude;
    }

    resetMinMaxCounter();
  }
}


// Swaps between metric/imperial measurements
void switchUnit() {
  if (unit == 'i') {
    unit = 'm';
    EEPROM.write(UNIT_ADDRESS, 0);
  } else {
    unit = 'i';
    EEPROM.write(UNIT_ADDRESS, 1);
  }
}


// Gets the current altitude
float getAltitude() {
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  return (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));
}


// Write an entire line with the altitude centered on the line
void outputAltitudeLine(bool includeTrackingOffset, float overrideAltitude) {
  float altitude;
  moveToSecondLine();

  if (overrideAltitude == NULL) {
    altitude = getAltitude();
  } else {
    altitude = overrideAltitude;
  }

  if (includeTrackingOffset) {
    altitude -= trackingAltitudeOffset;
  }

  // pad with some white space
  int before = center(altitudeWidth(altitude), 16) / 16;
  for (int i=0; i<before; i++) {
    lcd.print(" ");
  }

  displayAltitudeWithUnit(altitude);

  // end with white space
  int after = center(altitudeWidth(altitude), 16) % 16;
  for (int j=0; j<after; j++) {
    lcd.print(" ");
  }
}


// Write the only the altitude to the screen
void displayAltitudeWithUnit(float altitude) {
  float calibratedAltitude = altitude + calibrateAltitudeOffset;
  float num;
  char unitChar;
  int decimals;

  if (unit == 'i') {
    int intAltitude = round(calibratedAltitude * metersToFeet);
    num = round(calibratedAltitude * metersToFeet);
    unitChar = '\'';
    decimals = 0;
  } else {
    num = round(calibratedAltitude * 10) / 10.0;
    unitChar = 'm';
    decimals = 1;
  }

  lcd.print(num, decimals);
  lcd.print(unitChar);
}


// Width of altitude itself (including unit)
int altitudeWidth(float altitude) {
  int altitudeChars = 1;   // start with one character for the unit

  if (unit == 'i') {
    int intAltitude = round(altitude * metersToFeet);
    altitudeChars = altitudeChars + String(intAltitude).length();
  } else {
    int intAltitude = round(altitude * 10);
    altitudeChars = altitudeChars + String(intAltitude).length() + 1; // add one character for the decimal in meters
  }

  return altitudeChars;
}




///////////////////////
// Altitude calibration functions
///////////////////////

// Increments the altitude calibration offset
void incrementAltimeterCalibration() {
  if (unit == 'i') {
    calibrateAltitudeDisplay += (1 / metersToFeet);
  } else {
    calibrateAltitudeDisplay++;
  }
}


// Decrements the altitude calibration offset
void decrementAltimeterCalibration() {
  if (unit == 'i') {
    calibrateAltitudeDisplay -= (1 / metersToFeet);
  } else {
    calibrateAltitudeDisplay--;
  }
}


// Restarts the calibration process
void startCalibration() {
  calibrateAltitudeDisplay = getAltitude() + calibrateAltitudeOffset;
  calibrateAltitudeOffset = 0;
}


// Save the calibration offset
void saveAltitudeCalibration() {
  // Reset to 0 so we get a raw altitude reading from scratch
  calibrateAltitudeOffset = 0;
  calibrateAltitudeOffset = calibrateAltitudeDisplay - getAltitude();
  //Serial.print("calibrateAltitudeOffset: ");
  //Serial.println(calibrateAltitudeOffset);
  //EEPROM.write(CALIBRATE_ALTITUDE_OFFSET_ADDRESS, calibrateAltitudeOffset * 10);
}




///////////////////////
// Raw barometer functions
///////////////////////

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration() {
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}


// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut) {
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);
}


// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up) {
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}


// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address) {
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}


// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address) {
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}


// Read the uncompensated temperature value
unsigned int bmp085ReadUT() {
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}


// Read the uncompensated pressure value
unsigned long bmp085ReadUP() {
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);

  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}
