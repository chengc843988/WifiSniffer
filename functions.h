// This-->tab == "functions.h"

// Expose Espressif SDK functionality
extern "C" {
//#include "user_interface.h"
  typedef void (*freedom_outside_cb_t)(uint8 status);
  int  wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
  void wifi_unregister_send_pkt_freedom_cb(void);
  int  wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
  //  SoftwareSerial mySerial;
}

//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include "./structures.h"
// #include <Wire.h>

#define MAX_APS_TRACKED 150
#define MAX_CLIENTS_TRACKED 200

beaconinfo aps_known[MAX_APS_TRACKED];                    // Array to save MACs of known APs
int aps_known_count = 0;                                  // Number of known APs
int nothing_new = 0;
clientinfo clients_known[MAX_CLIENTS_TRACKED];            // Array to save MACs of known CLIENTs
int clients_known_count = 0;                              // Number of known CLIENTs

#define register_client(ci) register_clientTTL(ci)
#define register_beacon(beacon) register_beaconTTL(beacon)

char *AP_MAC[] = {"\xdc\xfb\x02\x76\x19\x18", NULL};
char *CL_MAC[] = {"\x30\x75\x12\xf5\x7a\x81", NULL};

int register_beaconTTL(beaconinfo beacon)
{
  // if(beacon.err) return 0;
  int u, x, c;
  int known = 0;
  uint32_t timestamp = 0xffff0000;
  for (u = 0; u < aps_known_count; u++) {
    if ((c = memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) == 0) {
      known = 1;
      timestamp = aps_known[u].timestamp;
      break;
    }
  }
  if (u == MAX_APS_TRACKED) u--;
  for (int x = u; x > 0; x--)
    memcpy(&aps_known[x], &aps_known[x - 1], sizeof(beacon));
  memcpy(&aps_known[0], &beacon, sizeof(beacon));

  if (!known && aps_known_count < MAX_APS_TRACKED) {
    aps_known_count++;
  }
  //  for (x = 0; AP_MAC[x] != NULL; x++) {
  //    if (memcmp(AP_MAC[x], beacon.bssid, ETH_MAC_LEN) == 0) {
  //      known = 0;
  //    }
  //  }
  // if ( timestamp > 20) known = 0;
  if ( (timestamp + 3000) < millis()) known = 0;
  return known;
  // return aps_known_count-u;
}

int register_clientTTL(clientinfo ci)
{
  int u, x, c;
  int known = 0;
  uint32_t timestamp = 0xffff0000;
  for (u = 0; u < clients_known_count ; u++) {
    if ((c = memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) == 0) {
      known = 1;
      timestamp = clients_known[u].timestamp;
      break;
    }
  }
  if (u == clients_known_count) u--;
  for (x = u; x > 0; x--) {
    memcpy(&clients_known[x], &clients_known[x - 1], sizeof(ci));
  }
  memcpy(&clients_known[0], &ci, sizeof(ci));

  if (!known && clients_known_count < MAX_CLIENTS_TRACKED) {
    clients_known_count++;
  }
  //  for (x = 0; CL_MAC[x] != NULL; x++) {
  //    if (memcmp(CL_MAC[x], ci.station, ETH_MAC_LEN) == 0) {
  //      known = 0;
  //    }
  //  }
  // if ( u > 20) known = 0;
  if ( (timestamp + 3000) < millis()) known = 0;
  return known;
  // return u + 1;
}



int register_beaconSorted(beaconinfo beacon)
{
  int u, x;
  int known = 0;
  signed int c;
  for (u = 0; u < aps_known_count; u++) {
    c = memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN);
    if (c == 0) {
      known = 1;
      memcpy(&aps_known[u], &beacon, sizeof(beacon));
      break;
    }
    if (c > 0) {
      break;
    }
  }
  if (aps_known_count < MAX_APS_TRACKED && known == 0 ) {
    for (x = aps_known_count; x > u; x--)
      memcpy(&aps_known[x], &aps_known[x - 1], sizeof(beacon));
    memcpy(&aps_known[u], &beacon, sizeof(beacon));
    aps_known_count++;
  }
  return known;
}

int register_clientSort(clientinfo ci)
{
  int u;
  int known = 0;
  for (u = 0; u < clients_known_count; u++) {
    int c;
    c = memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN);
    if (c == 0) {
      known = 1;
      memcpy(&clients_known[u], &ci, sizeof(ci));
      break;
    }
    if (c > 0) {
      known = 0;
      break;
    }
  }
  if (clients_known_count < MAX_CLIENTS_TRACKED && known == 0 ) {
    for (int x = clients_known_count; x > u; x--)
      memcpy(&clients_known[x], &clients_known[x - 1], sizeof(ci));
    memcpy(&clients_known[u], &ci, sizeof(ci));
    clients_known_count++;
  }
  return known;
}

int register_beaconSerial(beaconinfo beacon)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < aps_known_count; u++)
  {
    if (! memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) {
      known = 1;
      break;
    }   // AP known => Set known flag
  }
  if (! known)  // AP is NEW, copy MAC to array and return it
  {
    memcpy(&aps_known[aps_known_count], &beacon, sizeof(beacon));
    aps_known_count++;

    if ((unsigned int) aps_known_count >=
        sizeof (aps_known) / sizeof (aps_known[0]) ) {
      Serial.printf("exceeded max aps_known\n");
      aps_known_count = 0;
    }
  }
  return known;
}



int register_clientSerial(clientinfo ci)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < clients_known_count; u++)
  {
    if (! memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) {
      known = 1;
      break;
    }
  }
  if (! known)
  {
    memcpy(&clients_known[clients_known_count], &ci, sizeof(ci));
    clients_known_count++;

    if ((unsigned int) clients_known_count >=
        sizeof (clients_known) / sizeof (clients_known[0]) ) {
      Serial.printf("exceeded max clients_known\n");
      clients_known_count = 0;
    }
  }
  return known;
}

void print_beacon(beaconinfo beacon)
{
  if (beacon.err != 0) {
    Serial.printf("BEACON: <=============== [BEACON ERR: %20d]  ", beacon.err);
    for (int i = 0; i < 6; i++) Serial.printf("%02x", beacon.bssid[i]);
    Serial.printf("   %2d", beacon.channel);
    Serial.printf("   %4d\r\n", beacon.rssi);
    // Serial.printf("BEACON ERR: (%d)  ", beacon.err);
  } else {
    Serial.printf("BEACON: <=============== [%32s]  ", beacon.ssid);
    for (int i = 0; i < 6; i++) Serial.printf("%02x", beacon.bssid[i]);
    Serial.printf("   %2d", beacon.channel);
    Serial.printf("   %4d\r\n", beacon.rssi);
  }
}

void print_client(clientinfo ci)
{
  int u = 0;
  int known = 0;   // Clear known flag
  if (ci.err != 0) {
    // nothing
  } else {
    Serial.printf("DEVICE: ");
    for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.station[i]);
    Serial.printf(" ==> ");

    for (u = 0; u < aps_known_count; u++)
    {
      if (! memcmp(aps_known[u].bssid, ci.bssid, ETH_MAC_LEN)) {
        Serial.printf("[%32s]", aps_known[u].ssid);
        known = 1;     // AP known => Set known flag
        break;
      }
    }

    if (! known)  {
      Serial.printf("   Unknown/Malformed packet \r\n");
      //  for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.bssid[i]);
    } else {
      Serial.printf("%2s", " ");
      for (int i = 0; i < 6; i++) Serial.printf("%02x", ci.ap[i]);
      Serial.printf("  %3d", aps_known[u].channel);
      Serial.printf("   %4d\r\n", ci.rssi);

      // Wire.beginTransmission(8);
      // Wire.write(ci.rssi);
      // Wire.endTransmission();
    }
  }
}

void promisc_cb(uint8_t *buf, uint16_t len)
{
  int i = 0;
  uint16_t seq_n_new = 0;
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len == 128) {
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2*) buf;
    struct beaconinfo beacon = parse_beacon(sniffer->buf, 112, sniffer->rx_ctrl.rssi);
    if (register_beacon(beacon) == 0) {
      print_beacon(beacon);
      nothing_new = 0;
    }
  } else {
    struct sniffer_buf *sniffer = (struct sniffer_buf*) buf;
    //Is data or QOS?
    if ((sniffer->buf[0] == 0x08) || (sniffer->buf[0] == 0x88)) {
      struct clientinfo ci = parse_data(sniffer->buf, 36, sniffer->rx_ctrl.rssi, sniffer->rx_ctrl.channel);
      if (memcmp(ci.bssid, ci.station, ETH_MAC_LEN)) {
        if (register_client(ci) == 0) {
          print_client(ci);
          nothing_new = 0;
        }
      }
    }
  }
}

