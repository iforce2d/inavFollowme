/*
Follow-me tag. Sends latitude, longitude, altitude, velocity.
Requires u-blox GPS module, 6 series or above.
Uses MS5611 barometer. Shield from wind and sunlight.
nRF24L01 library: https://github.com/gcopeland/RF24

GPS connections (hardware UART)
 1 - GND - GND
 2 - VCC - VCC
 3 - TX - Arduino pin 0 (disconnect to upload sketch)
 4 - RX - Arduino pin 1 (disconnect to upload sketch, maybe...)
 
MS5611 connections
 1 - GND - GND
 2 - VCC - VCC
 3 - SDA - Arduino pin A4
 4 - SCL - Arduino pin A5

nRF24L01 connections 
 1 - GND
 2 - VCC 3.3V !!! Ideally 3.0v, definitely not 5V
 3 - CE to Arduino pin 9
 4 - CSN to Arduino pin 10
 5 - SCK to Arduino pin 13
 6 - MOSI to Arduino pin 11
 7 - MISO to Arduino pin 12
 8 - UNUSED
 
*/
 
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
//#include <SoftwareSerial.h>
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "MS5611.h"
#include "nRF24.h"
#include "GPS.h"


void setup() {

  //Serial.begin(9600);

  gps_setup();
  baro_setup();
  radio_setup();
  
  delay(5000); // wait a few seconds to let me put the cover on :)

}

int count = 0;
boolean doneCalibration = false;

void loop() {

  unsigned long now = micros();
  baro_loop(now);
  radio_loop();
  gps_loop();

  if ( !doneCalibration ) {
    if ( count++ > 200 ) {
      //Serial.println("Calibrating");
      for (int i = 0; i < 200; i++) {
        performBaroCalibrationCycle();
        delay(10);
      }
      doneCalibration = true;
    }
  }
  else {
    data.altitude = baroCalculateAltitude();
    //Serial.println( data.altitude );
  }

  delay(20);
}


