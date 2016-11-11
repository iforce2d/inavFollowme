
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);

char ppsBuf[16];
char dstBuf[16];
char altBuf[16];
char yawBuf[16];
char modBuf[16];
char offBuf[16];

void draw() {
  u8g.drawStr( 2, 12, ppsBuf);
  u8g.drawStr( 2, 24, altBuf);
  u8g.drawStr( 2, 36, yawBuf);
  u8g.drawStr( 2, 48, modBuf);
  u8g.drawStr( 2, 60, offBuf);
}

unsigned long lastScreenUpdate = 0;

void screen_setup() {
  u8g.setColorIndex(1);
  u8g.setFont(u8g_font_fur11);
}

void screen_loop()
{  
  unsigned long now = millis();
  if ( now - lastScreenUpdate < 100 )
    return;
    
  packetCountTotal -= packetCounts[packetCountIndex];
  packetCounts[packetCountIndex] = packetsRead;
  packetCountTotal += packetsRead;

  packetsRead = 0;  
  packetCountIndex = (packetCountIndex + 1) % 10;
  
  /*if ( now - lastValuesSwitchTime > 5000 ) {
    whichValues = ! whichValues;
    lastValuesSwitchTime = now;
  }*/
  
  int chan0 = (rcChannels[RC_ROLL] - 1000) / 100;
  
  chan0 = constrain(chan0, 0, 10);
  
  sprintf(ppsBuf, "PPS: %d RC: %d", packetCountTotal, chan0);  
  sprintf(altBuf, "Alt: %ld, %ld", currentAlt, data.alt);
  sprintf(yawBuf, "Yaw: %d, %d", currentYaw, cameraYaw);
  sprintf(modBuf, "Mode: %d", followMeEnabled);
  sprintf(offBuf, "%ld, %ld, %ld", latOffset, lonOffset, altOffset);

  u8g.firstPage();
  do {
    draw();
  } 
  while( u8g.nextPage() );
  
  lastScreenUpdate = millis();  
}
