#include <Wire.h>
#include <SPI.h>
#include "Ard1863.h"
#include <SD.h>
#include <string.h>
#include <elapsedMillis.h> 

#define SerialConsole Serial

//Debug variable
boolean debug = false;
int debugPin = 9;

//Set up the ADC board and timing functions
Ard186x ard186xboard1;
int chipSelectPin = 3;
elapsedMillis sinceSample;
unsigned int sampleRate = 9; // 10ms - 1 for 100Hz sampling rate
unsigned int adcVal;

//SD board and variables
int SDChipSelect = 4;
String fileName = "lastcali.txt";
String theDelim = ",";
File caliFile;

//Setup input/output pins for calibration
int calibrateInPin = A0;
int highLowInPin = A1;
int caliLowLEDPin = 2;
int caliHighLEDPin = 5;
 
//Setup calibration variables
double lowAdc = 0;
double highAdc = 100.0;
boolean calibrated = false;
double linear_M;
double linear_B;
signed int convertedValue;
byte sendableValue[2];
elapsedMillis startedReading;
unsigned int onceCalibratedWait = 1000; //1 seconds after calibration dont allow recalibrate
boolean setLow = false;
boolean setHigh = false;
double sampledLow;
double sampledHigh;
unsigned int adcSampleSize = 100;

void sendAsByteArr(){
  if(Serial.availableForWrite() >= 3){
    Serial.write(0xFF);
    Serial.write(highByte(convertedValue));
    Serial.write(lowByte(convertedValue));
  }
  
  //uint8_t msbYte = (convertedValue >> 8) & 0xFF;
  //uint8_t lsbYte = (convertedValue >> 0) & 0xFF;
  //signed int reVal = (signed int) ((signed int)msbYte << 8) | ((signed int)lsbYte);
}

void initializeADC(){
  ard186xboard1.begin(DEVICE_LTC1867, ARD186X_EEP_ADDR_ZZ, chipSelectPin);
  ard186xboard1.setFastSPI(1);
  ard186xboard1.ltc186xChangeChannel(LTC186X_CHAN_DIFF_0P_1N, 1);
}

unsigned int readADC(){
  return ard186xboard1.ltc186xRead();
}

void setup() {
  // initialize serial communications:
  SerialConsole.begin(115200);
  while (!SerialConsole);

  // This is the initialization command for the ADC
  initializeADC();

  //Initialize the SD card
  if (!SD.begin(SDChipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }

  //Initialize calibrated pin and pull down
  pinMode(calibrateInPin, INPUT);
  digitalWrite(calibrateInPin, LOW);

  //Initialize high low pin
  pinMode(highLowInPin, INPUT);
  digitalWrite(highLowInPin, LOW);

  //Initialize the low calibrated pin
  pinMode(caliLowLEDPin, OUTPUT);
  digitalWrite(caliLowLEDPin, LOW);

  //Initialize the high calibrated pin
  pinMode(caliHighLEDPin, OUTPUT);
  digitalWrite(caliHighLEDPin, LOW);

  //Initialize the debug pin
  if(debug){
    pinMode(debugPin, OUTPUT);
    analogWrite(debugPin, 0);
  }

  //Check to see if configurations have already been saved
  if (SD.exists(fileName)) {
    //Open the file
    caliFile = SD.open(fileName);
    if(caliFile){
      int line = 0;
      while (caliFile.available() && line < 1) {  
        //Increment the line counter
        line++;
        
        // read from the file until end of line
        String theLine = caliFile.readStringUntil('\n');

        //Find the delimiter
        unsigned int theSplit = strcspn(theLine.c_str(), theDelim.c_str());

        //Extract the double values
        linear_M = atof(theLine.substring(0, theSplit).c_str());
        linear_B = atof(theLine.substring(theSplit + 1).c_str());

        if(linear_M != 0.0 && linear_B != 0.0){
          //If valid conversion then start doing conversions
          digitalWrite(caliLowLEDPin, HIGH);
          digitalWrite(caliHighLEDPin, HIGH);
          calibrated = true;
          startedReading = 0;
          setLow = true;
          setHigh = true;
        } else {
          //If any error then set calibration invalid
          digitalWrite(caliLowLEDPin, LOW);
          digitalWrite(caliHighLEDPin, LOW);
          calibrated = false;
          setLow = false;
          setHigh = false;
        }

        if(debug){
          Serial.print(linear_M, 8);
          Serial.print(",");
          Serial.println(linear_B, 8);
        }
      }
      
    // close the file:
    caliFile.close();
    }
  }
}

void loop() {
  if (calibrated){
    if(sinceSample > sampleRate) {
      if(digitalRead(calibrateInPin) == LOW || startedReading < onceCalibratedWait){
        //Read the adc values
        adcVal = readADC();
        
        if(!debug){
          //Reset the milisecond timer
          sinceSample = 0;
          
          //Convert from ADC to interpretable value
          convertedValue = (signed int) round(((linear_M * double(adcVal)) + linear_B) * 100);
          sendAsByteArr();
        } else {
          Serial.print(sinceSample);
          sinceSample = 0;
          Serial.print(",");
          Serial.print(adcVal);
          Serial.print(",");
          Serial.print((linear_M * double(adcVal)) + linear_B, 4);
          Serial.print(",");
          Serial.print((signed long) round((((linear_M * double(adcVal)) + linear_B) * 100)));
          Serial.println("");
          analogWrite(debugPin, (int) ((adcVal / (double) pow(2, 16)) * 255));
        }
      } else {
        //Catch the re-calibration sequence
        calibrated = false;
        digitalWrite(caliLowLEDPin, LOW);
        digitalWrite(caliHighLEDPin, LOW);
        calibrated = false;
        setLow = false;
        setHigh = false;
      }
    }
  } else {
    if (digitalRead(calibrateInPin) == HIGH) {
      //Get an averaged ADC value
      double adcSum = 0.0;
      for (unsigned int i = 0; i < adcSampleSize; i++) {
        adcSum += (double) readADC();
      }
      double avgAdc = adcSum / (double) adcSampleSize;

      //Check to see which calibration level
      if (!setLow && digitalRead(highLowInPin) == LOW) {
        sampledLow = avgAdc;
        setLow = true;
        digitalWrite(caliLowLEDPin, HIGH);
      } else if (!setHigh && digitalRead(highLowInPin) == HIGH) {
        sampledHigh = avgAdc;
        setHigh = true;
        digitalWrite(caliHighLEDPin, HIGH);
      }

      //If low and high are set, then calculate the linear function
      if (setLow && setHigh) {
        linear_M = (highAdc - lowAdc) / (sampledHigh - sampledLow);
        linear_B = -1 * linear_M * sampledLow;

        //If calibration already exists then delete it
        if(SD.exists(fileName)){
          SD.remove(fileName);
        }

        //Open the file for writing
        caliFile = SD.open(fileName, FILE_WRITE);
        if(caliFile){
          //Convert the linear M value to c-string
          char linMStr[11];
          dtostrf(linear_M, 1, 8, linMStr);
          caliFile.print(linMStr);

          caliFile.print(",");
          
          //Convert he linear B value to c-string
          char linBStr[11];
          dtostrf(linear_B, 1, 8, linBStr);
          caliFile.print(linBStr);
          caliFile.print("\n");
          
          //Close the file
          caliFile.close();
        }

        if(debug){
          Serial.print(sampledLow);
          Serial.print(",");
          Serial.print(sampledHigh);
          Serial.print(",");
          Serial.print(linear_M, 6);
          Serial.print(",");
          Serial.print(linear_B, 6);
          Serial.print(",");
          Serial.println(sampledHigh);
        }
        
        calibrated = true;
        startedReading = 0;
      }
    }
  }
}
