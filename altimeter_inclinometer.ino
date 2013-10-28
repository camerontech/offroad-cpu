#include <Wire.h>
#include <LiquidCrystal.h>
#include <EEPROMEx.h>
#include <I2Cdev.h>
#include <BMP085.h>
#include <ADXL345.h>
#include <HMC5883L.h>


const int VERSION = 2;

unsigned long millisCounter = 0;
unsigned long minMaxAltitudeMillsCounter = 0;

// Menu and display modes
const String menuText[][2] = {{"Incline        ","               "},
                              {"Altitude       ","               "},
                              {"Compass        ","               "},
                              {"Temperature    ","               "},
                              {"Track          ","Altitude       "},
                              {"Min/Max        ","Altitude       "},
                              {"Calibrate      ","Altitude       "},
                              {"Set            ","Brightness     "},
                              {"Factory        ","Reset          "}};
const unsigned int INCLINE = 0;
const unsigned int ALTITUDE = 1;
const unsigned int COMPASS = 2;
const unsigned int TEMPERATURE = 3;
const unsigned int TRACK = 4;
const unsigned int MINMAX = 5;
const unsigned int CALIBRATE = 6;
const unsigned int BRIGHTNESS = 7;
const unsigned int RESET = 8;
const unsigned int MENU = 9;
const unsigned int MENU_LENGTH = 9;

// Keep track of where we are
int mode;
int displayMenuItem;
int lastMode;

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
// Accelerometer
//////////////////////

ADXL345 accel;

const int X_MIN = -254;
const int X_MAX = 269;
const int Y_MIN = -248;
const int Y_MAX = 261;
const int Z_MIN = -283;
const int Z_MAX = 235;

// So that we can zero out the inclinometer readings
int pitchOffset, rollOffset;


//////////////////////
// Barometer
//////////////////////

BMP085 barometer;
int32_t lastMicros;
float lastAltitude = 0;

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
// Magnetometer
////////////////////
HMC5883L mag;



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

byte upArrow[8] = {
  B00000,
  B00100,
  B01110,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000
};

byte downArrow[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B01110,
  B00100,
  B00000
};

byte degree[8] = {
  B01100,
  B10010,
  B10010,
  B01100,
  B00000,
  B00000,
  B00000,
  B00000
};

byte foot[8] = {
  B01000,
  B01000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

byte click[8] = {
  B00000,
  B00000,
  B00100,
  B01110,
  B00100,
  B00000,
  B00000,
  B00000
};

byte UP_ARROW = 0;
byte DOWN_ARROW = 1;
byte DEGREE = 2;
byte FOOT = 3;
byte CLICK = 4;


///////////////////////////
// EEPROM memory addresses
///////////////////////////

// display
const int LCD_BACKLIGHT_ADDRESS = EEPROM.getAddress(sizeof(int));              // backlight setting
const int BAUD_ADDRESS = EEPROM.getAddress(sizeof(int));                       // Baud rate setting
const int SPLASH_SCREEN_ADDRESS = EEPROM.getAddress(sizeof(int));              // splash screen on/off
const int ROWS_ADDRESS = EEPROM.getAddress(sizeof(int));                       // number of rows
const int COLUMNS_ADDRESS = EEPROM.getAddress(sizeof(int));                    // number of columns

// sensors
const int VERSION_ADDRESS = EEPROM.getAddress(sizeof(int));                   // version
const int UNIT_ADDRESS = EEPROM.getAddress(sizeof(byte));                      // metric or imperial
const int PITCH_OFFSET_ADDRESS = EEPROM.getAddress(sizeof(int));              // pitch degree offset
const int ROLL_OFFSET_ADDRESS = EEPROM.getAddress(sizeof(int));               // roll degree offset
const int MODE_ADDRESS = EEPROM.getAddress(sizeof(int));                      // which menu item is showing
const int CALIBRATE_ALTITUDE_OFFSET_ADDRESS = EEPROM.getAddress(sizeof(float)); // altitude offset


void setup() {
  analogReference(EXTERNAL);

 //  Serial.begin(9600);
 //  Serial.print("LCD_BACKLIGHT_ADDRESS: ");
 //  Serial.println(LCD_BACKLIGHT_ADDRESS);
 //  Serial.print("BAUD_ADDRESS: ");
	// Serial.println(BAUD_ADDRESS);
 //  Serial.print("SPLASH_SCREEN_ADDRESS: ");
	// Serial.println(SPLASH_SCREEN_ADDRESS);
 //  Serial.print("ROWS_ADDRESS: ");
	// Serial.println(ROWS_ADDRESS);
 //  Serial.print("COLUMNS_ADDRESS: ");
	// Serial.println(COLUMNS_ADDRESS);

 //  // sensors
 //  Serial.print("VERSION_ADDRESS: ");
	// Serial.println(VERSION_ADDRESS);
 //  Serial.print("UNIT_ADDRESS: ");
	// Serial.println(UNIT_ADDRESS);
 //  Serial.print("PITCH_OFFSET_ADDRESS: ");
	// Serial.println(PITCH_OFFSET_ADDRESS);
 //  Serial.print("ROLL_OFFSET_ADDRESS: ");
	// Serial.println(ROLL_OFFSET_ADDRESS);
 //  Serial.print("MODE_ADDRESS: ");
	// Serial.println(MODE_ADDRESS);
 //  Serial.print("CALIBRATE_ALTITUDE_OFFSET_ADDRESS: ");
	// Serial.println(CALIBRATE_ALTITUDE_OFFSET_ADDRESS);


  // Set EEPROM variables if the version changes
  if (EEPROM.readInt(VERSION_ADDRESS) != VERSION) {
    memoryReset();
  }

  Wire.begin();

  setupVariables();
  setupDisplay();
  setupButton();
  setupAccelerometer();
  setupBarometer();
  setupMagnetometer();

  // Holding the switch in the down position at startup performs a factory reset
  if (digitalRead(DOWN) == LOW) {
    factoryReset();
  }
}

void loop() {

  // Were any buttons pressed?
  buttonCheck();

  // To properly record the min/max altitude we need to check it on every cycle
  updateMinMaxAltitude();

  if (mode == INCLINE) {
    loopInclinometer();
  } else if (mode == ALTITUDE) {
    loopAltimeter();
  } else if (mode == COMPASS) {
    loopCompass();
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
  memoryReset();
  setupVariables();
  setBrightness();
}

void memoryReset() {
  EEPROM.writeInt(LCD_BACKLIGHT_ADDRESS, 255);               // max brightness
  EEPROM.writeByte(UNIT_ADDRESS, 'i');                       // default to imperial
  EEPROM.writeInt(PITCH_OFFSET_ADDRESS, 0);                  // no pitch offset
  EEPROM.writeInt(ROLL_OFFSET_ADDRESS, 0);                   // no roll offset
  EEPROM.writeInt(MODE_ADDRESS, INCLINE);                    // default to inclinometer
  EEPROM.writeFloat(CALIBRATE_ALTITUDE_OFFSET_ADDRESS, 0.0); // no altitude offset

  EEPROM.writeInt(VERSION_ADDRESS, VERSION);
}

// Saves the current mode to EEPROM
void saveMode() {
  EEPROM.writeInt(MODE_ADDRESS, mode);
}

// Reads variables from EEPROM and sets their RAM equivalents
void setupVariables() {
  brightness = EEPROM.readInt(LCD_BACKLIGHT_ADDRESS);
  unit = EEPROM.readByte(UNIT_ADDRESS);
  pitchOffset = EEPROM.readInt(PITCH_OFFSET_ADDRESS);
  rollOffset = EEPROM.readInt(ROLL_OFFSET_ADDRESS);
  mode = displayMenuItem = EEPROM.readInt(MODE_ADDRESS);
  calibrateAltitudeOffset = EEPROM.readFloat(CALIBRATE_ALTITUDE_OFFSET_ADDRESS);
  calibrateAltitudeDisplay = NULL;
}

void setupDisplay() {
  lcd.begin(16, 2);

  // custom characters
  lcd.createChar(0, upArrow);
  lcd.createChar(1, downArrow);
  lcd.createChar(2, degree);
  lcd.createChar(3, foot);
  lcd.createChar(4, click);

  // Set up the backlight
  pinMode(BLPin, OUTPUT);
  brightness = EEPROM.readInt(LCD_BACKLIGHT_ADDRESS);
  setBrightness();

  // Splash screen
  lcd.clear();
  lcd.print("  Cameron Tech  ");
  moveToSecondLine();
  lcd.print("  Offroad CPU   ");
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
  accel.initialize();
}


void setupBarometer() {
  barometer.initialize();
  resetMinMaxAltitude();
}


void setupMagnetometer() {
  mag.initialize();
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
  delay(10);
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
      if (displayMenuItem == RESET) {         // Factory Reset feature
        factoryReset();
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
        startMode();
      }
    } else if (mode == BRIGHTNESS) { // when in brightness mode, up/down changes brightness, push goes back to menu
      if (buttonState == UP) {
        increaseBrightness();
      } else if (buttonState == DOWN) {
        decreaseBrightness();
      } else {
        saveBrightness();
        mode = lastMode;
        displayMenuItem = lastMode;
        startMode();
      }
    } else {                                                 // in most modes up/down goes to menu, push is optional function
      if (buttonState == UP || buttonState == DOWN) {        // Button pressed up or down to go into the menu
        lastMode = mode;
        mode = MENU;
      } else if (mode == TRACK && buttonState == PUSH) {     // Button clicked while tracking altitude
        resetTrackingAltitude();
      } else if (mode == MINMAX && buttonState == PUSH) {    // Button clicked while viewing min/max
        resetMinMaxAltitude();
      } else if (mode == ALTITUDE || mode == TEMPERATURE && buttonState == PUSH) { // Button clicked on altimeter or thermometer
        switchUnit();
      } else if (mode == INCLINE && buttonState == PUSH) {   // Button clicked while viewing incline
        zeroInclinometer();
      }
    }
  }
}

// Called when a menu item is selected but before looping through that mode
void startMode() {
  saveMode();
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
    lcd.write(UP_ARROW);
    moveToSecondLine();
    lcd.print(menuText[displayMenuItem][1]);
    lcd.write(DOWN_ARROW);

    resetCounter();
  }
}

void loopInclinometer() {
  if (millis() - millisCounter > 250) {

    moveToFirstLine();
    lcd.print("  Pitch   Roll ");
    lcd.write(CLICK);
    moveToSecondLine();

    int pitch, roll;
    getIncline(pitch, roll, false);
    displayIncline(roll, pitch);

    resetCounter();
  }
}


void loopAltimeter() {
  if (millis() - millisCounter > 500) {

    moveToFirstLine();
    lcd.print("    Altitude   ");
    lcd.write(CLICK);
    moveToSecondLine();
    outputAltitudeLine(false, NULL);

    resetCounter();
  }
}


void loopCompass() {
  if (millis() - millisCounter > 500) {

    int x, y, z;
    mag.getHeading(&x, &y, &z);
    float heading = atan2(y, x);
    if (heading < 0) {
      heading += 2 * M_PI;
    }
    heading = heading * 180 / M_PI;

    String degree = String(int(heading)) + char(DEGREE);
    String compass = degreeToCardinal(heading);

    moveToFirstLine();
    lcd.print("     Heading    ");
    moveToSecondLine();
    centerText(compass, 8);
    centerText(degree, 8);

    resetCounter();
  }

}

void loopTrack() {
  if (millis() - millisCounter > 500) {
    moveToFirstLine();
    lcd.print("Track Altitude ");
    lcd.write(CLICK);
    moveToSecondLine();
    outputAltitudeLine(true, NULL);
    resetCounter();
  }
}

void loopMinMax() {
  if (millis() - millisCounter > 500) {
    moveToFirstLine();
    lcd.print("   Min    Max  ");
    lcd.write(CLICK);
    moveToSecondLine();
    centerText(altitudeWithUnit(minAltitude), 8, true);
    centerText(altitudeWithUnit(maxAltitude), 8, false);

    resetCounter();
  }
}

void loopCalibrate() {
  if (millis() - millisCounter > 250) {
    moveToFirstLine();
    lcd.print("  Set Altitude ");
    lcd.write(UP_ARROW);
    moveToSecondLine();

    if (calibrateAltitudeDisplay == NULL) {
      calibrateAltitudeDisplay = getAltitude();
    }
    outputAltitudeLine(false, calibrateAltitudeDisplay);
    lcd.setCursor(15,1);
    lcd.write(DOWN_ARROW);

    resetCounter();
  }
}

void loopTemperature() {
  if (millis() - millisCounter > 1000) {
    String output;
    moveToFirstLine();
    lcd.print("  Temperature  ");
    lcd.write(CLICK);
    moveToSecondLine();

    float temperature = getTemperature();

    if (unit == 'i') {
      output = floatToString(temperature*9/5+32, 1) + char(DEGREE) + 'F';
    } else {
      output = floatToString(temperature, 1) + char(DEGREE) + 'C';
    }
    centerText(output, 16);

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
}


void saveBrightness() {
  EEPROM.writeInt(LCD_BACKLIGHT_ADDRESS, brightness);
}



//////////////////////////////
// Incline functions
//////////////////////////////

// Pass the shortCircuit boolean if you want to return the roll/pitch without
// offset correction (used to set offset values)
void getIncline(int &pitch, int &roll, bool shortCircuit) {
  int x, y, z;
  int xTotal = 0;
  int yTotal = 0;
  int zTotal = 0;
  int samples = 10;

  // Take 10 samples and average them to help lower noise
  for (int i=0; i<samples; i++) {
    accel.getAcceleration(&x, &y, &z);
    xTotal += x;
    yTotal += y;
    zTotal += z;
  }
  int xAvg = xTotal / samples;
  int yAvg = yTotal / samples;
  int zAvg = zTotal / samples;

  // convert to range of -90 to +90 degrees
  int xAng = map(xAvg, X_MIN, X_MAX, -90, 90);
  int yAng = map(yAvg, Y_MIN, Y_MAX, -90, 90);
  int zAng = map(zAvg, Z_MIN, Z_MAX, -90, 90);

  // convert radians to degrees
  int pitchOut = -(RAD_TO_DEG * (atan2(-yAng, -zAng) + PI));
  int rollOut = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);

  // convert left roll and forward pitch to negative degrees
  if (pitchOut < -180) {
    pitchOut = pitchOut + 360;
  }
  if (rollOut > 180) {
    rollOut = rollOut - 360;
  }

  // Sometimes we want the actual output degree measurement, not the calibrated
  // version, so pass `true` to the shortCircuit param and return them
  if (shortCircuit) {
    pitch = pitchOut;
    roll = rollOut;
    return;
  }

  // Take into account any offset
  pitchOut -= pitchOffset;
  rollOut -= rollOffset;

  // Now we need to re-add the offset in case we try to roll around 180/-180 again
  if (pitchOut < -180) {
    pitchOut += 360;
  } else if (pitchOut > 180) {
    pitchOut -= 360;
  }
  if (rollOut < -180) {
    rollOut += 360;
  } else if (rollOut > 180) {
    rollOut -= 360;
  }

  pitch = pitchOut;
  roll = rollOut;
}

// Takes the current incline degrees and sets them to 0 by recording offsets
void zeroInclinometer() {
  getIncline(pitchOffset, rollOffset, true);
  EEPROM.writeInt(PITCH_OFFSET_ADDRESS, pitchOffset);
  EEPROM.writeInt(ROLL_OFFSET_ADDRESS, rollOffset);
}


// Writes the current pitch/roll to the screen
void displayIncline(int first, int second) {
  // convert int values to strings for output
  String firstString = String(first) + char(DEGREE);
  String secondString = String(second) + char(DEGREE);

  centerText(firstString, 8, true);
  centerText(secondString, 8, false);

}




//////////////////////////////
// Altitude functions
//////////////////////////////

// Zeros out the tracking altitude
void resetTrackingAltitude() {
  trackingAltitudeOffset = getAltitude() + calibrateAltitudeOffset;
}

// Take the current altitude and set that as both the min and max
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
    // EEPROM.write(UNIT_ADDRESS, 0);
  } else {
    unit = 'i';
    // EEPROM.write(UNIT_ADDRESS, 1);
  }
  EEPROM.writeByte(UNIT_ADDRESS, unit);
}


float getTemperature() {
  barometer.setControl(BMP085_MODE_TEMPERATURE);

  lastMicros = micros();
  while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

  return barometer.getTemperatureC();
}


// Gets the current altitude
float getAltitude() {
  float output;
  int samples = 10;
  float totalAltitude = 0.0;

  // Have to read temperature before getting pressure
  getTemperature();

  for (int i=0; i<samples; i++) {
    barometer.setControl(BMP085_MODE_PRESSURE_3);

    lastMicros = micros();
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());
    float pressure = barometer.getPressure();
    totalAltitude += barometer.getAltitude(pressure);
  }

  // To help reduce noise, only show altitude changing if the change is greater
  // than the average noise level (1m or so)
  // if (abs(altitude - lastAltitude) > 1.0) {
  //   output = altitude;
  //   lastAltitude = altitude;
  // } else {
  //   output = lastAltitude;
  // }

  // return output;
  return totalAltitude / (float)samples;
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

  centerText(altitudeWithUnit(altitude), 16);
}


// Write only the altitude to the screen
String altitudeWithUnit(float altitude) {
  float calibratedAltitude = altitude + calibrateAltitudeOffset;

  if (unit == 'i') {
    return String(round(calibratedAltitude * metersToFeet)) + char(FOOT);
  } else {
    return floatToString(calibratedAltitude, 1) + 'm';
  }

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
  calibrateAltitudeOffset = 0.0;
  calibrateAltitudeOffset = calibrateAltitudeDisplay - getAltitude();

  // Save to EEPROM
  EEPROM.writeFloat(CALIBRATE_ALTITUDE_OFFSET_ADDRESS, calibrateAltitudeOffset);
}


///////////////////////
// Compass functions
///////////////////////

String degreeToCardinal(float heading) {
  if (heading > 337.5 || heading <= 22.5)
    return "N";
  else if (heading > 22.5 && heading <= 67.5)
    return "NW";
  else if (heading > 67.5 && heading <= 112.5)
    return "W";
  else if (heading > 112.5 && heading <= 157.5)
    return "SW";
  else if (heading > 157.5 && heading <= 202.5)
    return "S";
  else if (heading > 202.5 && heading <= 247.5)
    return "SE";
  else if (heading > 247.5 && heading <= 292.5)
    return "E";
  else if (heading > 292.5 && heading <= 337.5)
    return "NE";
}



//////////////////////////////
// Text centering helpers
//////////////////////////////

// Centers an integer
void centerText(int num, int width) {
  String text = String(num);
  centerString(text, width, false);
}

// Centers a float
void centerText(float num, int decimals, int width) {
  String text = floatToString(num, decimals);
  centerString(text, width, false);
}

// Centers a string
void centerText(String text, int width) {
  centerString(text, width, false);
}

// Centers a string
void centerText(String text, int width, bool prependWhiteSpace) {
  centerString(text, width, prependWhiteSpace);
}

// Converts a floating point number to a string
String floatToString(float num, int decimals) {
  int wholePart = int(num);
  int precision = pow(10, decimals);
  int fractionalPart = abs(num - wholePart) * precision;
  String text = String(wholePart);
  text += ".";
  text += String(fractionalPart);

  return text;
}

// Takes a string and outputs it centered within the given width
void centerString(String text, int width, bool prependWhiteSpace) {
  int textLength = text.length();
  int totalWhiteSpace = width - textLength;
  int left = totalWhiteSpace / 2;
  int right = left;

  // Should we put any odd whitespace at the beginning or the endminmax
  if (prependWhiteSpace) {
    left += (totalWhiteSpace % 2);
  } else {
    right += (totalWhiteSpace % 2);
  }

  for (int i=0; i<left; i++) {
    lcd.print(" ");
  }

  lcd.print(text);

  for (int i=0; i<right; i++) {
    lcd.print(" ");
  }
}

