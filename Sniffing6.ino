// by Ray Burnette 20161013 compiled on Linux 16.3 using Arduino 1.6.12

// #include <ESP8266WiFi.h>
#include <WiFi.h>
#include "./functions.h"

#define disable 0
#define enable  1
// uint8_t channel = 1;
unsigned int channel = 1;
//#include <Wire.h>


// #include <SoftwareSerial.h>
// SoftwareSerial mySerial(13, 15); // RX, TX nodemcu

void setup() {
  //  mySerial.begin(9600);
  //pinMode(13, OUTPUT);
  //Wire.begin();


  Serial.begin(115200);
  Serial.printf("\n\nSDK version:%s\n\r", system_get_sdk_version());
  Serial.println(F("ESP8266 mini-sniff by Ray Burnette http://www.hackster.io/rayburne/projects"));
  Serial.println(F("Type:   /-------MAC------/-----WiFi Access Point SSID-----/  /----MAC---/  Chnl  RSSI"));

  wifi_set_opmode(STATION_MODE);            // Promiscuous works only with station mode
  wifi_set_channel(channel);
  wifi_promiscuous_enable(disable);
  wifi_set_promiscuous_rx_cb(promisc_cb);   // Set up promiscuous callback
  wifi_promiscuous_enable(enable);
}

uint8_t flag = false;
void loop() {
  channel = 1;
  // digitalWrite(13, (flag ^= flag));
  wifi_set_channel(channel);
  while (true) {
    nothing_new++;                          // Array is not finite, check bounds and adjust if required
    if (nothing_new > 200) {
      nothing_new = 0;
      channel++;
      if (channel == 15) break;             // Only scan channels 1 to 14
      wifi_set_channel(channel);
    }
    delay(5);  // critical processing timeslice for NONOS SDK! No delay(0) yield()
    // Press keyboard ENTER in console with NL active to repaint the screen
    if ((Serial.available() > 0) && (Serial.read() == '\n')) {
      Serial.println("\n-------------------------------------------------------------------------------------\n");
      for (int u = 0; u < clients_known_count; u++) {
        Serial.print(u); Serial.print(" - ");
        print_client(clients_known[u]);
        delay(1);
      }
      delay(100);
      for (int u = 0; u < aps_known_count; u++) {
        Serial.print(u); Serial.print(" - ");
        print_beacon(aps_known[u]);
        delay(1);
      }
      Serial.println("\n-------------------------------------------------------------------------------------\n");
      // delay(1000);  // critical processing timeslice for NONOS SDK! No delay(0) yield()

    }
  }
}
