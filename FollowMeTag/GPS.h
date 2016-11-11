
//SoftwareSerial serial = SoftwareSerial(7,8); // too slow

#define GPSSERIAL Serial

const unsigned char UBLOX_INIT[] PROGMEM = {
  
  // Disable SBAS
  0xB5,0x62,0x06,0x16,0x08,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x25,0x90,
  
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x29, //NAV-VELNED off

  // Enable UBX
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //NAV-VELNED on

  // Rate
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };
const unsigned char NAV_POSLLH_HEADER[] = { 0x01, 0x02 };
const unsigned char NAV_VELNED_HEADER[] = { 0x01, 0x12 };

enum _ubxMsgType {
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_VELNED
};

struct NAV_POSLLH {
  unsigned char cls;      // 0x01
  unsigned char id;       // 0x02
  unsigned short len;     // 28
  unsigned long iTOW;     // millisecond time of week 
  long lon;               // longitude
  long lat;               // latitude
  long height;            // height above ellipsoid
  long hMSL;              // height above mean sea level
  unsigned long hAcc;     // horizontal accuracy
  unsigned long vAcc;     // vertical accuracy
};

struct NAV_VELNED {
  unsigned char cls;      // 0x01
  unsigned char id;       // 0x12
  unsigned short len;     // 36
  unsigned long iTOW;     // millisecond time of week
  long velN;              // north velocity (cm/s)
  long velE;              // east velocity (cm/s)
  long velD;              // down velocity (cm/s)
  unsigned long speed;    // 3d (cm/s)
  unsigned long gSpeed;   // ground speed (2d)
  long heading;           // heading of motion (2d)
  unsigned long sAcc;     // speed accuracy
  unsigned long cAcc;     // course/heading accuracy
};

union UBXMessage {
  NAV_POSLLH navPosllh;
  NAV_VELNED navVelned;
};

UBXMessage ubxMessage;

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}


// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}


// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  
  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while ( GPSSERIAL.available() ) {
    
    byte c = GPSSERIAL.read();    
    //Serial.write(c);
    
    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0; // Reset to beginning state.
    }
    else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&ubxMessage))[fpos-2] = c;

      fpos++;
      
      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if ( compareMsgHeader(NAV_POSLLH_HEADER) ) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(NAV_POSLLH);
        }
        else if ( compareMsgHeader(NAV_VELNED_HEADER) ) {
          currentMsgType = MT_NAV_VELNED;
          payloadSize = sizeof(NAV_VELNED);
        }
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0; 
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          // Checksum matches, we have a valid message.
          return currentMsgType; 
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  return MT_NONE;
}

void gps_setup()
{  
  GPSSERIAL.begin(9600);

  // send configuration data in UBX protocol
  for(int i = 0; i < (int)sizeof(UBLOX_INIT); i++) {                        
    GPSSERIAL.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

void gps_loop() {
  int msgType = processGPS();
  if ( msgType == MT_NAV_POSLLH ) {
    /*Serial.print("iTOW:");      Serial.print(posllh.iTOW);
    Serial.print(" lat/lon: "); Serial.print(posllh.lat/10000000.0f); Serial.print(","); Serial.print(posllh.lon/10000000.0f);
    Serial.print(" height: ");  Serial.print(posllh.height/1000.0f);
    Serial.print(" hMSL: ");    Serial.print(posllh.hMSL/1000.0f);
    Serial.print(" hAcc: ");    Serial.print(posllh.hAcc/1000.0f);
    Serial.print(" vAcc: ");    Serial.print(posllh.vAcc/1000.0f);
    Serial.println();*/
    data.lon = ubxMessage.navPosllh.lon;
    data.lat = ubxMessage.navPosllh.lat;
  }
  else if ( msgType == MT_NAV_VELNED ) {
    data.velN = ubxMessage.navVelned.velN;
    data.velE = ubxMessage.navVelned.velE;
  }
}



