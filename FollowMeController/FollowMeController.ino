/*
Follow-me controller for cleanflight iNav.
nRF24L01 library: https://github.com/gcopeland/RF24
I2C OLED screen library: https://github.com/olikraus/u8glib

nRF24L01 connections 
 1 - GND
 2 - VCC 3.3V !!! Ideally 3.0v, definitely not 5V
 3 - CE to Arduino pin 9
 4 - CSN to Arduino pin 10
 5 - SCK to Arduino pin 13
 6 - MOSI to Arduino pin 11
 7 - MISO to Arduino pin 12
 8 - UNUSED
 
OLED connections (optional but extremely useful)
 GND - GND
 VCC - VCC
 SDA - Arduino pin A4
 SCL - Arduino pin A5
 
Radio heartbeat LED on pin 2 (optional but extremely useful)
 
 */

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "printf.h"
#include <SoftwareSerial.h>
#include "U8glib.h"
#include <nRF24L01.h>
#include <RF24.h>
#include "cleanflight.h"
#include "nRF24.h"
#include "rc.h"
#include "gimbal.h"
#include "msp.h"
#include "screen.h"

void setup() { 
  msp_setup();
  gimbal_setup();  
  radio_setup();
  rc_setup();
  screen_setup(); 
}

void loop() {
  msp_loop(); // calls rc_loop, sets gimbal output
  radio_loop();
  screen_loop();
}








