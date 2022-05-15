// LowPower GPS - Runs on battery and recovers from power loss
// Jim Sheridan
#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <TinyGPS.h>
#define WAITING_TIME 10 //waiting time between messages
//#define GPS_PIN 2 //( pin, for energy saving. not used now)
#define GPS_INFO_BUFFER_SIZE 128
bool debug = true; //DEBUG switch/////
TinyGPS gps;//GPS Object
//GPS data variables
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
char GPS_info_char;
char GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
unsigned int received_char;
bool message_started = false;
int i = 0;
// GPS coordinate structure, 12 bytes size on 32 bits platforms
struct gpscoord {
  float a_latitude;  // 4 bytes
  float a_longitude; // 4 bytes
  float a_altitude;  // 4 bytes
};
float latitude  = 0.0f;
float longitude = 0.0f;
float altitude = 0;
//////////////// Waiting function //////////////////
void Wait(int m, bool s) {
 
  //m minutes to wait
  //s slow led pulses
  if (debug) 
  digitalWrite(LED_BUILTIN, LOW);
  if (s) {
    int seg = m * 100;
    for (int i = 0; i < seg; i++) {
    //    nada
        delay(1000);
    }
  } else {
    int seg = m * 50;
    for (int i = 0; i < seg; i++) {
        delay(1000);
    }
  }
}
/////////////////// Sigfox Send Data function ////////////////
void SendSigfox(String data) {
  if (debug) 
  // Remove EOL
  data.trim();
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(30);
  if (debug) SigFox.debug();
  delay(100);
  SigFox.beginPacket();
  SigFox.print(data);
  SigFox.endPacket();
  SigFox.end();
}
//////////////////  Convert GPS function  //////////////////
/* Converts GPS float data to Char data */
String ConvertGPSdata(const void* data, uint8_t len) {
  uint8_t* bytes = (uint8_t*)data;
  String chain ;
  for (uint8_t i = len - 1; i < len; --i) {
    if (bytes[i] < 12) {
      chain.concat(byte(0)); // Not tested
    }
    chain.concat(char(bytes[i]));
    if (debug) Serial.print(bytes[i], HEX);
  }
  return chain;
}
////////////////////////// Get GPS position function/////////////////////
String GetGPSposition() {
  int messages_count = 0;
  String pos;
  if (debug) //Serial.println("GPS ON");
 // digitalWrite(GPS_PIN, HIGH); //Turn GPS on
  Wait(1, false);
  while (messages_count < 5000) {
    while (Serial1.available()) {
      int GPS_info_char = Serial1.read();
      if (GPS_info_char == '$') messages_count ++; // start of message. Counting messages.
      if (debug) {
        if (GPS_info_char == '$') { // start of message
          message_started = true;
          received_char = 0;
        } else if (GPS_info_char == '*') { 
          message_started = false; // ready for the new message
        } else if (message_started == true) { // the message is already started and I got a new character
          if (received_char <= GPS_INFO_BUFFER_SIZE) { // to avoid buffer overflow
            GPS_info_buffer[received_char] = GPS_info_char;
            received_char++;
          } else { // resets everything (overflow happened)
            message_started = false;
            received_char = 0;
          }
        }
      }
      if (gps.encode(GPS_info_char)) {
        gps.f_get_position(&latitude, &longitude);
        altitude = gps.altitude() / 100;
        // Store coordinates into dedicated structure
        gpscoord coords = {altitude, longitude, latitude};
        gps.stats(&chars, &sentences, &failed_checksum);
        if (debug)   
      //  digitalWrite(GPS_PIN, LOW); //GPS turned off
        pos = ConvertGPSdata(&coords, sizeof(gpscoord)); //Send data
        return pos;
      }
    }
  }
  pos = "No Signal";
}
//////////////////SETUP///////////////////
void setup() {
  if (debug) {
    Serial.begin(9600);
  }
  //Serial1 pins 13-14 for 3.3V connection to GPS.
  Serial1.begin(9600);
  while (!Serial1) {}
  if (debug) {
 //   Serial.println("GPS Connected");
  }
  if (!SigFox.begin()) {
    return;
  }
  // Enable debug led and disable automatic deep sleep
  if (debug) {
    SigFox.debug();
  } else {
    SigFox.end(); // Send the module to the deepest sleep
  }
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, dummy, CHANGE);
}
//////////////////////LOOP////////////////////////
void loop() {
   
  
  String position_data;
  position_data = GetGPSposition();
  SendSigfox(position_data);
  Wait(WAITING_TIME, false);
}
void dummy() {
  volatile int ttt = 0;
}