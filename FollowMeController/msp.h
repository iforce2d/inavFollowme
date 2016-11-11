#define msp Serial

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

byte crc = 0;
boolean waitingForLocation = false;
unsigned long lastRequestTime = 0;

#define MSP_RC           105
#define MSP_RAW_GPS      106
#define MSP_ATTITUDE     108
#define MSP_ALTITUDE     109

#define MSP_SET_WP       209

int whichMsgToRequest = MSP_RAW_GPS;
unsigned long lastWaypointSetTime = 0;

#define FRAMELOC_UPDATE_INTERVAL  20  // ms. requests take turns, so each will be updated at four times this interval
#define MIN_WP_UPDATE_INTERVAL    100 // ms. Don't send WP updates any faster than this

float longitudeScale = 1;

struct geoloc {
  float lat;
  float lon;
  int16_t alt;
  geoloc(float x = 0, float y = 0, int32_t a = 0) { lat = x; lon = y; alt = a; }
};

// used for distance and bearing calculation between quad and target, for gimbal control
geoloc frameLoc(0,0,0);
geoloc tagLoc(0,0,0);

const unsigned char MSP_HEADER[] = { '$', 'M', '>' };

struct MSP_LOC {
  uint8_t size;          
  uint8_t type;          
  
  uint8_t fixType;       
  uint8_t numSat;        
  int32_t lat;          
  int32_t lon;          
  int16_t alt; // meters
  int16_t groundSpeed;  
  int16_t groundCourse; 
  uint16_t hdop;         
};

struct MSP_ATT {
  uint8_t size;
  uint8_t type;
  
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
};

struct MSP_RCI {
  uint8_t size;
  uint8_t type;
  
  uint16_t rcChan[12];
};

struct MSP_ALT {
  uint8_t size;
  uint8_t type;
  
  int32_t alt;
  int16_t vario;
};

struct MSP_WP {
  uint8_t wpNo;
  uint8_t action;
  int32_t lat;
  int32_t lon;
  int32_t alt;     // cm
  int16_t heading; // degrees 0 - 360 (aka p1)
  int16_t p2;      // unused
  int16_t p3;      // unused
  uint8_t flag;    // unused
};

union MSPMessage {
  MSP_LOC msploc;
  MSP_ATT mspatt;
  MSP_ALT mspalt;
  MSP_RCI msprci;
};

MSPMessage mspmsg;
MSP_WP mspwp;

void initWaypoint() {
  mspwp.wpNo = 255;
  mspwp.action = 1;
  mspwp.lat = 0;
  mspwp.lon = 0;
  mspwp.alt = 0;
  mspwp.heading = 0;
  mspwp.p2 = 0;
  mspwp.p3 = 0;
  mspwp.flag = 0;
}

void mspSerialize8(byte b) {
  msp.write(b);
  crc ^= b;
}
void mspSerialize16(int16_t a) {
  mspSerialize8((a   ) & 0xFF);
  mspSerialize8((a>>8) & 0xFF);
}
void mspSerialize32(uint32_t a) {
  mspSerialize8((a    ) & 0xFF);
  mspSerialize8((a>> 8) & 0xFF);
  mspSerialize8((a>>16) & 0xFF);
  mspSerialize8((a>>24) & 0xFF);
}

byte calcMSPChecksum(int payloadSize) {
  byte b = 0;
  //softSerial.print("calcing ");
  for (int i = 0; i < payloadSize; i++) {
    b ^= ((byte*)(&mspmsg))[i];
    //softSerial.print(((byte*)(&mspgps))[i]);
  }
  return b;
}

/********************************************************/

// http://www.movable-type.co.uk/scripts/latlong.html
float geoDistance(struct geoloc &a, struct geoloc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoBearing(struct geoloc &a, struct geoloc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

/********************************************************/

boolean processMSP() {
  static int fpos = 0;
  static byte checksum;
  
  static byte currentMsgType = 0;
  static int payloadSize = sizeof(MSPMessage);
  
  while (msp.available()) {
    byte c = msp.read();
    //softSerial.write(c);
    
    if ( fpos < 3 ) {
      if ( c == MSP_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      //softSerial.write(c);
      
      int payloadPos = fpos - 3;
      
      if ( payloadPos < payloadSize )
        ((byte*)(&mspmsg))[payloadPos] = c;
      
      if ( payloadPos == 1 ) {
        // We have just received the second byte of the payload, 
        // so now we can check to see what kind of message it is.
        if ( c == MSP_RAW_GPS ) {
          currentMsgType = MSP_RAW_GPS;
          payloadSize = sizeof(MSP_LOC);
        }
        else if ( c == MSP_ATTITUDE ) {
          currentMsgType = MSP_ATTITUDE;
          payloadSize = sizeof(MSP_ATT);
        }
        else if ( c == MSP_ALTITUDE ) {
          currentMsgType = MSP_ALTITUDE;
          payloadSize = sizeof(MSP_ALT);
        }
        else if ( c == MSP_RC ) {
          currentMsgType = MSP_RC;
          payloadSize = sizeof(MSP_RCI);
        }
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      fpos++;

      if ( payloadPos == payloadSize ) {
        checksum = calcMSPChecksum(payloadSize);
        /*softSerial.print("calced ");
        softSerial.println(checksum);
        softSerial.print("c is now: ");
        softSerial.println(c);*/
        if ( c == checksum ) {
          fpos = 0;
          return currentMsgType;
        }
      }
      else if ( payloadPos > payloadSize ) {
        fpos = 0;
      }
    }
    
  }
  return 0;
}

void msp_setup() {
  msp.begin(115200);
  initWaypoint();
}

void msp_loop() {
  
  int msgType = processMSP();
  if ( msgType ) {
    if ( msgType == MSP_RAW_GPS ) {
      currentLat = mspmsg.msploc.lat;
      currentLon = mspmsg.msploc.lon;
      longitudeScale = constrain(cos_approx((fabsf(currentLat) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
    }
    else if ( msgType == MSP_ATTITUDE ) {
      currentYaw = mspmsg.mspatt.yaw;
    }
    else if ( msgType == MSP_ALTITUDE ) {
      currentAlt = mspmsg.mspalt.alt;
    }
    else if ( msgType == MSP_RC ) {
      memcpy( rcChannels, mspmsg.msprci.rcChan, 12 * sizeof(uint16_t) );
      rc_loop();
    }
  }
  
  if ( waitingForLocation ) {
    if ( millis() - lastRequestTime > FRAMELOC_UPDATE_INTERVAL )
      waitingForLocation = false;
  }
  else {
    mspSerialize8('$');
    mspSerialize8('M');
    mspSerialize8('<');
    crc = 0;
    mspSerialize8(0);
    mspSerialize8(whichMsgToRequest);
    msp.write(crc);
    
    crc = 0;
    waitingForLocation = true;
    lastRequestTime = millis();
    
    switch ( whichMsgToRequest ) {
      case MSP_RAW_GPS: whichMsgToRequest = MSP_ATTITUDE; break;
      case MSP_ATTITUDE: whichMsgToRequest = MSP_ALTITUDE; break;
      case MSP_ALTITUDE: whichMsgToRequest = MSP_RC; break;
      case MSP_RC: whichMsgToRequest = MSP_RAW_GPS; break;
    }
  }
  
  if ( millis() - lastWaypointSetTime > MIN_WP_UPDATE_INTERVAL ) {
    
    if ( true || newRadioData ) {
      
      
      {
        // find offset in centimeters
        float dx = (currentLat - data.lat) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
        float dy = (currentLon - data.lon) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * longitudeScale;
        
        // add one second extrapolation
        dx += data.velN;
        dy += data.velE;
        
        if ( controlMode == CM_OVERHEAD ) {
          latOffset = 0;
          lonOffset = 0;
          if ( altOffset < 500 )
            altOffset = 500; // enforce at least 5m relative altitude
        }
        
        else if ( controlMode == CM_FIXED_RADIUS ) {
          float dist = sqrtf(dx*dx + dy*dy);
          
          if ( circleTick == 0 ) {
            // circleTick being zero means follow-me mode has just been switched on
            followDistance = dist; // set desired follow distance to current distance
            circleTick = 1; // just to prevent this from happening again
          }
          
          if ( dist > 0.1 ) {
            float mod = followDistance / dist;
            dx *= mod;
            dy *= mod;
            latOffset = dx / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
            lonOffset = dy / (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * longitudeScale);
          }
          else {
            latOffset = 0;
            lonOffset = 0;
            if ( altOffset < 500 )
              altOffset = 500; // enforce at least 5m relative altitude
          }
        }
        
        else if ( controlMode == CM_FIXED_OFFSET ) {
          // offsets will have been set in enterFollowMeMode
        }
        
        else if ( controlMode == CM_CIRCLE ) {
          
          if ( circleTick == 0 ) {
            // circleTick being zero means follow-me mode has just been switched on
            // record current distance as radius of circle to keep
            followDistance = sqrtf(dx*dx + dy*dy);
          }
          
          // we want movement speed of about 4m/s
          float circumference = 2 * M_PIf * followDistance; // cm
          float degreesPerSecond = (400 / circumference) * 360;
          // limit to a reasonable value when radius is small
          if (degreesPerSecond > 45)
            degreesPerSecond = 45; // 8 seconds per revolution
          
          circleTick += degreesPerSecond * 0.1; // this update is called 10 times per second
          float rads = circleTick * DEGTORAD;
          dx = followDistance * cos_approx(rads);
          dy = followDistance * sin_approx(rads);
          latOffset = dx / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
          lonOffset = dy / (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * longitudeScale);
        }
        
      }
      
      mspwp.lat = data.lat + latOffset;
      mspwp.lon = data.lon + lonOffset;
      mspwp.alt = data.alt + altOffset;        // zero will leave altitude unchanged
      mspwp.action = 0x01;  // NAV_WP_ACTION_WAYPOINT
      
      if ( controlMode == CM_TURN_ONLY ) {
        // just use the values as they were when mode was entered
        mspwp.lat = modeEnteredLat;
        mspwp.lon = modeEnteredLon;
        mspwp.alt = modeEnteredAlt;
      }
      
      {
        // calculate heading
        frameLoc.lat = currentLat / 10000000.0f;
        frameLoc.lon = currentLon / 10000000.0f;      
        tagLoc.lat = data.lat / 10000000.0f;
        tagLoc.lon = data.lon / 10000000.0f; 
        float distanceToTarget = geoDistance( frameLoc, tagLoc );
        float bearingToTarget = geoBearing( frameLoc, tagLoc );
        float altitudeToTarget = (currentAlt - data.alt) / 100.0f; // centimeters
        
        float localBearing = bearingToTarget - currentYaw;
        while (localBearing < -180) localBearing += 360;
        while (localBearing >  180) localBearing -= 360;
        
        if ( distanceToTarget < 2 ) // less than 2 meters away from tag horizontally
          localBearing = lastLocalBearing; // keep current camera yaw instead of turning all over the place
        
        setYaw(localBearing);
        lastLocalBearing = localBearing;
        
        float pitch = 0;
        if ( distanceToTarget != 0 ) {
          pitch = atan( altitudeToTarget / distanceToTarget );
          setPitch(-pitch * RADTODEG);
        }
        else // directly above target, just look straight down
          setPitch(-90);
        
        cameraYaw = localBearing; // for display
        
        // iNav expects angle in range 0-360 degrees
        if ( bearingToTarget < 0 )
          bearingToTarget += 360;

        if ( distanceToTarget < 2 ) // less than 2 meters away from tag horizontally
          bearingToTarget = mspwp.heading; // keep current heading instead of turning all over the place
          
        mspwp.heading = bearingToTarget; // float to int16_t
      }
      
      mspSerialize8('$');
      mspSerialize8('M');
      mspSerialize8('<');
      crc = 0;
      mspSerialize8(21);
      mspSerialize8(MSP_SET_WP);
      
      mspSerialize8(mspwp.wpNo);
      mspSerialize8(mspwp.action);
      mspSerialize32(mspwp.lat);
      mspSerialize32(mspwp.lon);
      mspSerialize32(mspwp.alt);
      mspSerialize16(mspwp.heading);
      mspSerialize16(mspwp.p2);
      mspSerialize16(mspwp.p3);
      mspSerialize8(mspwp.flag);
      
      msp.write(crc);

      lastWaypointSetTime = millis();
      newRadioData = false;
    }
    
  }
  
}
