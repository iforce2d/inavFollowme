/*
Simple receiver with display to check the data being sent by the follow-me tag.
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
 
OLED connections
 GND - GND
 VCC - VCC
 SDA - Arduino pin A4
 SCL - Arduino pin A5
 
*/

#include <SPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "U8glib.h"
#include <nRF24L01.h>
#include <RF24.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);	// Fast I2C / TWI

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct MyData {
  long lon;
  long lat;
  int32_t altitude;
  long velN;
  long velE;
};

MyData data;

/**************************************************/

int packetCounts[10];
int packetCountIndex = 0;
int packetCountTotal = 0;

void setup()
{
  // Set up radio module
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.openReadingPipe(1,pipeIn);
  radio.startListening();

  u8g.setColorIndex(1);
  u8g.setFont(u8g_font_fur11);

  memset(&data, 0, sizeof(MyData));
  data.altitude = 0;
  
  memset( packetCounts, 0, sizeof(packetCounts) );
}

unsigned long packetsRead = 0;
unsigned long lastScreenUpdate = 0;
unsigned long lastRecvTime = 0;
unsigned long drops = 0;

/**************************************************/

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    packetsRead++;
    lastRecvTime = millis();
  }

  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) 
    drops = 1;
  else
    drops = 0;
}

/**************************************************/

char ppsBuf[16];
char dropsBuf[16];
char altBuf[16];
char lonBuf[16];
char latBuf[16];

void draw() {
  u8g.drawStr( 2, 12, dropsBuf);
  u8g.drawStr( 2, 24, ppsBuf);
  u8g.drawStr( 2, 36, latBuf);
  u8g.drawStr( 2, 48, lonBuf);
  u8g.drawStr( 2, 60, altBuf);
}

boolean whichValues = false; // show either lat/lon/alt or speed
unsigned long lastValuesSwitchTime = 0;

void updateScreen()
{  
  unsigned long now = millis();
  if ( now - lastScreenUpdate < 100 )
    return;
    
  //lastScreenUpdate = now;
    
  packetCountTotal -= packetCounts[packetCountIndex];
  packetCounts[packetCountIndex] = packetsRead;
  packetCountTotal += packetsRead;

  packetsRead = 0;  
  packetCountIndex = (packetCountIndex + 1) % 10;
  
  if ( now - lastValuesSwitchTime > 5000 ) {
    whichValues = ! whichValues;
    lastValuesSwitchTime = now;
  }
  
  sprintf(dropsBuf, "Drops: %ld", drops);
  sprintf(ppsBuf, "PPS: %d", packetCountTotal);

  if ( whichValues ) {
    sprintf(latBuf, "Lat: %ld", data.lat);
    sprintf(lonBuf, "Lon: %ld", data.lon);
    sprintf(altBuf, "Alt: %ld", data.altitude);
  }
  else {
    sprintf(latBuf, "VelN: %ld", data.velN);
    sprintf(lonBuf, "VelE: %ld", data.velE);
    sprintf(altBuf, "");
  }

  u8g.firstPage();
  do {
    draw();
  } 
  while( u8g.nextPage() );
  
  lastScreenUpdate = millis();
  
}

/**************************************************/

void loop()
{
  recvData();
  updateScreen();
}







