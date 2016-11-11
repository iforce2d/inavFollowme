
#define LED_PIN 2
RF24 radio(9,10); // 7, 8


boolean ledOn = false;

void toggleLed() {
  ledOn = ! ledOn;
  digitalWrite(LED_PIN, ledOn);
}

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

// Data sent by the follow-me tag
// The sizeof this struct should not exceed 32 bytes
struct MyData {
  int32_t lon;  // deg * 10000000
  int32_t lat;  // deg * 10000000
  int32_t alt;  // cm
  int32_t velN; // cm/s
  int32_t velE; // cm/s
};

MyData data;

int ledCounter = 0;
int packetsRead = 0;

int packetCounts[10];
int packetCountIndex = 0;
int packetCountTotal = 0;

// This was originally used to skip giving instruction to cleanflight unless new info had 
// actually been received from the follow-me tag. But with CM_CIRCLE mode, instructions 
// need to be given continuously anyway, so this is not used. You could reinstate it to be
// used for modes other than CM_CIRCLE mode if you think it necessary to optimize.
boolean newRadioData = false; 

void radio_setup() {
  pinMode(LED_PIN, OUTPUT);

  memset( packetCounts, 0, sizeof(packetCounts) );
  data.lat = 0;
  data.lon = 0;
  
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
  
  toggleLed();
}

void radio_loop() {
    while ( radio.available() ) {
    int32_t oldLat = data.lat;
    int32_t oldLon = data.lon;
    int32_t oldAlt = data.alt;
    
    radio.read(&data, sizeof(MyData));
    packetsRead++;
    
    if ( ledCounter++ > 5 ) {
      toggleLed();
      ledCounter = 0;
    }
    
    if ( oldLat != data.lat ||
         oldLon != data.lon ||
         oldAlt != data.alt )
           newRadioData = true;
  }
}
