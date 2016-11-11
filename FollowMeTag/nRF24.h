
const uint64_t pipeOut =  0xE8E8F0F0E1LL;

struct PacketData {
  long lon;
  long lat;
  int32_t altitude;
  long velN;
  long velE;
};

PacketData data;

void resetData() {
  data.lon = 0;
  data.lat = 0;
  data.altitude = 0;
}

RF24 radio(9, 10);

void radio_setup() {
  //Serial.print("Radio...");
  
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  
  //Serial.println("done.");
}

void radio_loop() {
    radio.write(&data, sizeof(PacketData));
}
