uint16_t rcChannels[12];

#define RC_ROLL       0
#define RC_PITCH      1
#define RC_THROTTLE   2
#define RC_YAW        3
#define RC_AUX1       4
#define RC_AUX2       5
#define RC_AUX3       6
#define RC_AUX4       7

enum _controlMode {
  CM_OVERHEAD,        // move to position directly over follow-me tag (min 5m relative altitude), turn to face tag
  CM_FIXED_RADIUS,    // move toward follow-me tag if more than a certain distance away, turn to face tag
  CM_FIXED_OFFSET,    // maintain a fixed distance and bearing away from follow-me tag, turn to face tag
  CM_CIRCLE,          // rotate in a circle around the follow-me tag, turn to face tag
  CM_TURN_ONLY        // stay in fixed position, turn to face tag
};

float followDistance = 1000; // centimeters
float followAltitude = 800;  // centimeters 
_controlMode controlMode = CM_TURN_ONLY;

boolean followMeEnabled = false;

int32_t modeEnteredLat = 0;
int32_t modeEnteredLon = 0;
int32_t modeEnteredAlt = 0;

int32_t currentLat = 0;
int32_t currentLon = 0;
int32_t currentAlt = 0;
uint16_t currentYaw = 0;

int32_t latOffset = 0;
int32_t lonOffset = 0;
int32_t altOffset = followAltitude;

float lastLocalBearing = 0;

float circleTick = 0; // counter used to keep track of current position on circle for CM_CIRCLE mode

void enterFollowMeMode() {
  
  modeEnteredLat = currentLat;
  modeEnteredLon = currentLon;
  modeEnteredAlt = currentAlt;

  circleTick = 0;
  lastLocalBearing = 0;
  
  latOffset = currentLat - data.lat;
  lonOffset = currentLon - data.lon;
  altOffset = currentAlt - data.alt;
}

void exitFollowMeMode() {
}

void rc_setup() {
  memset( rcChannels, 0, 12 * sizeof(uint16_t) );
}

void rc_loop() {
  boolean followSwitchConditionsMet = 
    rcChannels[RC_AUX1] > 1300 && rcChannels[RC_AUX1] < 1700 && // switch 1 mid
    rcChannels[RC_AUX2] > 1700; // switch 2 high
  if ( ! followMeEnabled && followSwitchConditionsMet )
    enterFollowMeMode();
  else if ( followMeEnabled && ! followSwitchConditionsMet )
    exitFollowMeMode();
  followMeEnabled = followSwitchConditionsMet;
  
  /* // Example of setting control type from a switch
  _controlMode oldMode = controlMode;

  if ( rcChannels[RC_AUX3] < 1500 )
    controlMode = CM_CIRCLE;
  else 
    controlMode = CM_FIXED_OFFSET;
    
  if ( controlMode != oldMode )
    enterFollowMeMode();
  */
}
