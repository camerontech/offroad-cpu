#include <SoftwareSerial.h>

SoftwareSerial lcd(3,2);

const bool DEBUG = true;

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
                              {"             Set","      Brightness"}};
const unsigned int MENU_LENGTH = 8;
const unsigned int INCLINE = 0;
const unsigned int ALTITUDE = 1;
const unsigned int TRACK = 2;
const unsigned int MINMAX = 3;
const unsigned int CALIBRATE = 4;
const unsigned int TEMPERATURE = 5;
const unsigned int BRIGHTNESS = 6;
const unsigned int MENU = 7;
// Start on the incline view
unsigned int mode = INCLINE;
int displayMenuItem = 0;

// Display in (m)etric or (i)mperial
char unit = 'i';

// Display character for brightness meter
const unsigned int BLOCK_CHAR = 255;
const unsigned int MAX_BRIGHTNESS = 157;
const unsigned int MIN_BRIGHTNESS = 128;
const unsigned int BRIGHTNESS_INCREMENT = 7;
int unsigned brightness;


/////////////////////
// Three-way button
/////////////////////
const unsigned int UP = 10;
const unsigned int PUSH = 11;
const unsigned int DOWN = 12;

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

// ASCII character for degree symbol
const unsigned int DEGREE_CHAR = 223;


//////////////////////
// Barometer
//////////////////////

#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

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
float altitude;

// User-defined offset based some something like GPS readings
float calibrateAltitudeOffset = 0;
float calibrateAltitudeDisplay = 0;

// When user wants to track altitude change we start with the current altitude
float trackingAltitudeOffset = 0;

// Keep track of minimum and maximum altitude
float minAltitude;
float maxAltitude;

// convert from meters to feet
float metersToFeet = 3.28084;


void setup() {
  Serial.begin(9600);
  
  analogReference(EXTERNAL);

  setupDisplay();
  setupButton();
  setupAccelerometer();
  setupBarometer();
}

void loop() {
  buttonCheck();

  updateMinMaxAltitude();

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

void setupDisplay() {
  lcd.begin(9600);

  // set brightness
  brightness = MAX_BRIGHTNESS;
  setBrightness();

  clearDisplay();
  moveToFirstLine();

  lcd.write(" CameronTech.io ");
  lcd.write("  Inclinometer  ");

  delay(1000);
}


void setupButton() {
  Serial.println("Setting up button...");

  pinMode(UP, INPUT);
  pinMode(PUSH, INPUT);
  pinMode(DOWN, INPUT);

  // Set pull-up resistors
  digitalWrite(UP, HIGH);
  digitalWrite(PUSH, HIGH);
  digitalWrite(DOWN, HIGH);
}


void setupAccelerometer() {
  Serial.println("Setting up altimeter...");

}


void setupBarometer() {
  Serial.println("Setting up barometer...");

  Wire.begin();
  bmp085Calibration();
  minAltitude = getAltitude(0);
  maxAltitude = minAltitude;
}


void buttonCheck() {
  // Serial.println("Checking for button press...");

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
      Serial.print("Button: ");
      Serial.println(buttonState);
    }
    lastState = buttonState;
  }

  // debounce delay
  delay(50);
}


void buttonClick() {
  if (mode == MENU) {
    if (buttonState == DOWN) {
      displayMenuItem++;
      if (displayMenuItem == MENU_LENGTH) {    // rollover to beginning of list
        displayMenuItem = 0;
      }
    } else if (buttonState == UP) {
      displayMenuItem--;
      if (displayMenuItem < 0) {              // rollover to end of list
        displayMenuItem = MENU_LENGTH - 1;
      }
    } else if (buttonState == PUSH) {
      mode = displayMenuItem;                 // select menu item
    }
  } else {
    if (mode == CALIBRATE) {  // when in calibrate mode, up/down change values, push goes back to menu
      if (buttonState == UP) {
        incrementAltimeterCalibration();
      } else if (buttonState == DOWN) {
        decrementAltimeterCalibration();
      } else if (buttonState == PUSH) {
        saveCalibration();
        displayMenuItem = ALTITUDE;
        mode = ALTITUDE;
      }
    } else if (mode == BRIGHTNESS) { // when in brightness mode, up/down changes brightness, push goes back to menu
      if (buttonState == UP) {
        increaseBrightness();
      } else if (buttonState == DOWN) {
        decreaseBrightness();
      } else {
        mode = MENU;
      }
    } else {                  // when not in calibrate mode, up/down goes to menu, push is optional function
      if (buttonState == UP || buttonState == DOWN) {        // Button pressed up or down to go into the menu
        mode = MENU;
      } else if (mode == TRACK && buttonState == PUSH) {     // Button clicked while tracking altitude
        // Reset tracking altitude
        resetTrackingAltitude();
      } else if (mode == ALTITUDE || mode == TEMPERATURE && buttonState == PUSH) { // Button clicked on altimeter
        // Switch between meters and feet
        switchUnit();
      }
    }
  }
}


////////////
// Loops  //
////////////

void loopMenu() {
  if (millis() - millisCounter > 250) {
    moveToFirstLine();
    lcd.print(menuText[displayMenuItem][0]);
    lcd.print(menuText[displayMenuItem][1]);

    Serial.println("------Menu------");
    Serial.println(menuText[displayMenuItem][0]);
    Serial.println(menuText[displayMenuItem][1]);

    resetCounter();
  }
}

void loopInclinometer() {
  if (millis() - millisCounter > 250) {
    Serial.println("Inclinometer loop...");

    // sample the voltages
    delay(10);
    int unsigned x = analogRead(X_PIN);
    delay(10);
    int unsigned y = analogRead(Y_PIN);
    delay(10);
    int unsigned z = analogRead(Z_PIN);

    if (DEBUG) {
      Serial.print(" x");
      Serial.print(x);
      Serial.print(" y");
      Serial.print(y);
      Serial.print(" z");
      Serial.println(z);
    }

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

    // write the pitch and roll to the second line
    displayIncline(roll, pitch);

    resetCounter();
  }
}

void loopAltimeter() {
  if (millis() - millisCounter > 1000) {
    Serial.println("Altimeter loop...");

    moveToFirstLine();
    lcd.print("    Altitude    ");
    outputAltitude(false);

    resetCounter();
  }
}

void loopTrack() {
  if (millis() - millisCounter > 1000) {
    moveToFirstLine();
    lcd.print(" Track Altitude ");
    outputAltitude(true);

    resetCounter();
  }
}

void loopMinMax() {
  if (millis() - millisCounter > 1000) {
    moveToFirstLine();
    lcd.print("   Min    Max   ");
    lcd.print("  ");
    displayAltitudeWithUnit(minAltitude);
    lcd.print("  ");
    displayAltitudeWithUnit(maxAltitude);
    lcd.print("  ");

    resetCounter();
  }
}

void loopCalibrate() {
  if (millis() - millisCounter > 250) {
    moveToFirstLine();
    lcd.print("Current Altitude");
    if (calibrateAltitudeDisplay == 0) {
      calibrateAltitudeDisplay = getAltitude(0);
    }
    lcd.print("    ");
    // Serial.println("Current Altitude");
    // Serial.print("      ");
    displayAltitudeWithUnit(calibrateAltitudeDisplay);
    lcd.print("    ");
    resetCounter();
  }
}

void loopTemperature() {
  if (millis() - millisCounter > 1000) {
    moveToFirstLine();
    lcd.print("  Temperature   ");
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
    Serial.println("Brightness loop...");

    moveToFirstLine();
    lcd.print("   Brightness   ");
    int unsigned dots = map(brightness, 128, 157, 1, 16);
    for (int i=0; i<dots; i++) {
      lcd.write(BLOCK_CHAR);
    }
    for (int i=0; i<16-dots; i++) {
      lcd.write(" ");
    }
    resetCounter();
  }
}










void clearDisplay() {
  moveToFirstLine();
  lcd.write("                ");
  lcd.write("                ");
}

void moveToFirstLine() {
  lcd.write(254);
  lcd.write(128);
}

void moveToSecondLine() {
  lcd.write(254);
  lcd.write(192);
}







//////////////////////////////
// Custom functions
//////////////////////////////

void updateMinMaxAltitude() {
  if (millis() - minMaxAltitudeMillsCounter > 1000) {
    float altitude = getAltitude(0);

    if (altitude > maxAltitude) {
      maxAltitude = altitude;
    }
    if (altitude < minAltitude) {
      minAltitude = altitude;
    }

    resetMinMaxCounter();
  }
}

void increaseBrightness() {
  // Only increase the brightness by one point if already completely dark
  if (brightness == MIN_BRIGHTNESS) {
    brightness += 1;
  } else {
    brightness += BRIGHTNESS_INCREMENT;
  }

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
  lcd.write(124);
  lcd.write(brightness);
  delay(100);
}

// Resets the millisecond counter
void resetCounter() {
  millisCounter = millis();
}

void resetMinMaxCounter() {
  minMaxAltitudeMillsCounter = millis();
}

// Zeros out the tracking altitude
void resetTrackingAltitude() {
  Serial.println("Resetting tracking altitude.");
  trackingAltitudeOffset = getAltitude(0);
  Serial.print("trackingAltitudeOffset: ");
  Serial.println(trackingAltitudeOffset);
}

// Swaps between metric/imperial measurements
void switchUnit() {
  Serial.println("Switching units.");
  if (unit == 'i') {
    unit = 'm';
  } else {
    unit = 'i';
  }
}

// Writes the current pitch/roll to the screen
void displayIncline(int first, int second) {
  // Move to the start of the second line
  moveToFirstLine();
  lcd.print("  Pitch   Roll  ");

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

// Gets the current altitude
float getAltitude(float offset) {
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));
  altitude = altitude + calibrateAltitudeOffset - offset;

  return altitude;
}

// Gets altitude with or without any offset
void outputAltitude(bool offset) {
  float altitude = getAltitude(offset ? trackingAltitudeOffset : 0);

  // pad with some white apce
  for (int i=0;i<4;i++) {
    lcd.print(" ");
  }

  displayAltitudeWithUnit(altitude);

  // end with white space
  for (int i=0;i<4;i++) {
    lcd.print(" ");
  }
}

// Write the current altitude to the screen
void displayAltitudeWithUnit(float altitude) {
  Serial.println("displayAltitudeWithUnit()");

  String altString = String(int(altitude));

  if (unit == 'i') {
    lcd.print(altitude * metersToFeet, 0);
    lcd.print("'");

    // Serial.print(altitude * 3.28084, 0);
    // Serial.println("'");
  } else {
    lcd.print(altitude, 1);
    lcd.print("m");

    // Serial.print(altitude, 1);
    // Serial.println("m");
  }
}

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

// Save the calibration offset
void saveCalibration() {
  // Reset to 0 so we get a raw altitude reading from scratch
  calibrateAltitudeOffset = 0;
  calibrateAltitudeOffset = calibrateAltitudeDisplay - getAltitude(0);
  Serial.print("Calibrate offset: ");
  Serial.println(calibrateAltitudeOffset);
}

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

