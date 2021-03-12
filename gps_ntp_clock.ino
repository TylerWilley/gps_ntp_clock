/*
 * GPS and NTP based clock
 * 24 MAX7219 8x8 LED panels in series, CS and CLK pins terminated to ground with a 100 ohm resistor each to eliminate reflection.
 * outputs on pins GPIO10 (sd3), MOSI (d7), and SCK (d5) (and 5v/Vin+gnd)
 * 
 * NEO-6M GPS module inputs on pins D1 (rx) and D2 (tx) (and 3.3v+gnd)
 * 
 * NTP source is set to time.apple.com. (if GPS is unavailable)
 * 
 * Requires:
 * Adafruit GFX library
 * Max72xxPanel library
 * ESP8266 wifi library
 * TimeLib library
 * TinyGPSPlus library
 */

#include <sys/time.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include "espconn.h"
#include <TimeLib.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

/********************************************************************** GPS **********************************************************************/
static const int RXPin = D1, TXPin = D2;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup_gps()
{
  ss.begin(GPSBaud);
}

unsigned long last_gps_status = 0;
void loop_update_gps(unsigned long currentMillis, unsigned long *gps_sync_millis, uint32_t *gps_time, uint32_t *gps_time_ms)
{
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      // Periodically print out GPS status for debugging
      if(currentMillis - last_gps_status > 60000)
      {
        last_gps_status = currentMillis;
        Serial.print(F("DIAGS      Chars="));
        Serial.print(gps.charsProcessed());
        Serial.print(F(" Sentences-with-Fix="));
        Serial.print(gps.sentencesWithFix());
        Serial.print(F(" Failed-checksum="));
        Serial.print(gps.failedChecksum());
        Serial.print(F(" Passed-checksum="));
        Serial.println(gps.passedChecksum());
        Serial.println();
        if (gps.satellites.isUpdated())
        {
          Serial.print(F("SATELLITES Fix Age="));
          Serial.print(gps.satellites.age());
          Serial.print(F("ms Value="));
          Serial.println(gps.satellites.value());
        }
      }
    }
  }

  // Only sync system clock to GPS once every minute.
  if(*gps_sync_millis == 0 || currentMillis - *gps_sync_millis > 60000)
  if (gps.time.isValid() && gps.date.isValid())
  {
    struct tm gps_tm;
    gps_tm.tm_year = gps.date.year() - 1900; // gps is actual year, tm_year is years since 1900
    gps_tm.tm_mon = gps.date.month() - 1; // gps is 1-12, tm_mon is 0-11
    gps_tm.tm_mday = gps.date.day();
    gps_tm.tm_hour = gps.time.hour();
    gps_tm.tm_min = gps.time.minute();
    gps_tm.tm_sec = gps.time.second();
    gps_tm.tm_isdst = -1; // Don't know if it should be DST, and don't care... will handle that later
    time_t epoch = mktime(&gps_tm);

    // Store out the time this date was pulled from the GPS module, and the time
    // epoch in ms takes more than 32 bits, so have to use 2 vars to store seconds + milliseconds
    *gps_sync_millis = millis() - gps.time.age();
    *gps_time = epoch;
    *gps_time_ms = gps.time.centisecond() * 10;
  }
}

/********************************************************************** WIFI **********************************************************************/
ESP8266WiFiMulti wifiMulti;

WiFiUDP UDP;

const char *SSID = "";
const char *PASS = "";

void setup_wifi() { // Try to connect to some given access points. Then wait for a connection
  wifiMulti.addAP(SSID, PASS);   // add Wi-Fi networks you want to connect to

  Serial.println("Connecting");
  while (wifiMulti.run() != WL_CONNECTED) {  // Wait for the Wi-Fi to connect
    delay(250);
    Serial.print('.');
  }
  Serial.println("\r\n");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());             // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
  Serial.println("\r\n");
}


/********************************************************************** NTP **********************************************************************/
IPAddress timeServerIP;
const char* NTPServerName = "time.apple.com";
const int NTP_PACKET_SIZE = 48;
byte NTPBuffer[NTP_PACKET_SIZE];
ip_addr_t dnsserver;

void setup_ntp() {
  Serial.println("Listening for NTP on UDP://127.0.0.1:123");
  UDP.begin(123);
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
  Serial.println();
  
  IP4_ADDR(&dnsserver, 8,8,8,8);
  espconn_dns_setserver(0, &dnsserver);

  if(!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.reset();
  }
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);
}

void sendNTPpacket(IPAddress& address) {
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

unsigned long intervalNTP = 60000 * 30; // Request NTP time every minute
unsigned long prevNTP = 0;
void loop_request_ntp_update(unsigned long currentMillis)
{
  if (prevNTP == 0 or currentMillis - prevNTP > intervalNTP) { // If a 30 minutes has passed since last NTP request
    prevNTP = currentMillis;
    Serial.println("\r\nSending NTP request ...");
    sendNTPpacket(timeServerIP);
  }
}

void loop_read_ntp_packet(unsigned long currentMillis, unsigned long *lastNTPResponse, uint32_t *timeUNIX, uint32_t *timeUNIXms)
{
  /**** Read NTP ****/
  uint32_t UNIXTime = 0;
  uint32_t UNIXms = 0;
  if (UDP.parsePacket() != 0) {
    UDP.read(NTPBuffer, NTP_PACKET_SIZE);
    uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];

    uint32_t fraction = (NTPBuffer[44] << 24) | (NTPBuffer[45] << 16) | (NTPBuffer[46] << 8) | NTPBuffer[47];
    UNIXms = (uint32_t)(((double)fraction / 0xFFFFFFFF) * 1000);
    
    const uint32_t seventyYears = 2208988800UL;
    UNIXTime = NTPTime - seventyYears;
    Serial.printf("Unix Time: %d\r\n", UNIXTime);
    Serial.printf("MS: %d\r\n", UNIXms);
  }

  if (UNIXTime) {
    // Store out the time this date was pulled from the NTP module, and the time
    // epoch in ms takes more than 32 bits, so have to use 2 vars to store seconds + milliseconds
    *lastNTPResponse = currentMillis;
    *timeUNIX = UNIXTime;
    *timeUNIXms = UNIXms;
    Serial.print("NTP response:\t");
    Serial.println(*timeUNIX);
  } else if ((currentMillis - *lastNTPResponse) > 3600000) {
    // If it's been an hour since the last response, restart the ESP (maybe network got borked)
    Serial.println("More than 1 hour since last NTP response. Rebooting.");
    Serial.flush();
    ESP.reset();
  }

}

/********************************************************************** DISPLAY **********************************************************************/
int pinCS = 10; // Attach CS to this pin, DIN to MOSI and CLK to SCK (cf http://arduino.cc/en/Reference/SPI )
int numberOfHorizontalDisplays = 24;
int numberOfVerticalDisplays = 1;

Max72xxPanel matrix = Max72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);

#define OP_DECODEMODE   9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

void reset_spi()
{
  // This fixes display corruption
  matrix.spiTransfer(OP_DISPLAYTEST, 0);
  matrix.spiTransfer(OP_SCANLIMIT, 7);
  matrix.spiTransfer(OP_DECODEMODE, 0);
  matrix.shutdown(false);
  matrix.setIntensity(0);
}

void draw_loading()
{
  matrix.fillScreen(LOW);
  matrix.drawChar(0, 0, 'L', HIGH, LOW, 1);
  matrix.drawChar(6, 0, 'o', HIGH, LOW, 1);
  matrix.drawChar(12, 0, 'a', HIGH, LOW, 1);
  matrix.drawChar(18, 0, 'd', HIGH, LOW, 1);
  matrix.drawChar(24, 0, 'i', HIGH, LOW, 1);
  matrix.drawChar(30, 0, 'n', HIGH, LOW, 1);
  matrix.drawChar(36, 0, 'g', HIGH, LOW, 1);
  matrix.drawChar(42, 0, '.', HIGH, LOW, 1);
  matrix.drawChar(48, 0, '.', HIGH, LOW, 1);
  matrix.drawChar(54, 0, '.', HIGH, LOW, 1);
  matrix.write();
}

void setup_matrix()
{
  reset_spi();

  for(int i = 0; i < numberOfHorizontalDisplays; i++)
  {
    matrix.setPosition(i, numberOfHorizontalDisplays - 1 - i, 0);
    matrix.setRotation(i, 3);
  }
  draw_loading();
}


/**** Rotate display pixel offset y=0, x=0-4, then y=1, x=4-0, repeat ****/
int last_pixel_offset = 0;
int pixel_offset_x = 0;
int pixel_offset_y = 0;
void loop_rotate_offset(unsigned long currentMillis)
{
  if(last_pixel_offset == 0)
    last_pixel_offset = currentMillis;
  if(currentMillis - last_pixel_offset > 60000 * 30)
  {
    last_pixel_offset = currentMillis;
    if(pixel_offset_y == 0)
    {
      if(pixel_offset_x < 4)
        pixel_offset_x += 1;
      else
        pixel_offset_y = 1;
    }
    else
    {
      if(pixel_offset_x > 0)
        pixel_offset_x -= 1;
      else
        pixel_offset_y = 0;
    }
  }
}

int last_reset = 0;
void loop_reset_spi(unsigned long currentMillis)
{
  /**** Reset SPI periodically in case we get corruption ****/
  if(last_reset == 0)
    last_reset = currentMillis;
  if(currentMillis - last_reset > 60000)
  {
    Serial.println("Resetting SPI\r\n");
    reset_spi();
    last_reset = currentMillis;
  }
}


int last_offset_change = 0;
int current_index = 0;
const char *current_label = "UTC";
int current_offset = 0;
const char* labels[] = {"UTC", "IST", "MST", "PST"};
const int offsets[] = {0, 2, -7, -8};

const char* dst_labels[] = {"UTC", "IDT", "MDT", "PDT"};
const int dst_offset[] = {0, 3, -6, -7};
const int dst_start_month[] = {-1, 3, 3, 3};
const int dst_start_day[] = {-1, 26, 14, 14};
const int dst_end_month[] = {-1, 10, 11, 11};
const int dst_end_day[] = {-1, 31, 7, 7};

char sign = '+';
void loop_set_time_zone(unsigned long currentMillis)
{
  /**** Switch Timezones ****/
  if(last_offset_change == 0)
    last_offset_change = currentMillis;

  if(currentMillis - last_offset_change > 60)
  {
    Serial.print("Millis: ");
    Serial.print(currentMillis);
    Serial.print(" ");
    Serial.print(year(currentMillis));
    Serial.print("-");
    Serial.print(month(currentMillis));
    Serial.print("-");
    Serial.println(day(currentMillis));

    current_index += 1;
    if(current_index >= 4)
      current_index = 0;
    
    bool isdst = false;
    int cMonth = month(currentMillis);
    int cDay = day(currentMillis);
    if(dst_start_month[current_index] != -1 && dst_start_month[current_index] <= cMonth && cMonth <= dst_end_month[current_index])
    {
      Serial.print("DST: ");
      Serial.print(dst_start_month[current_index]);
      Serial.print("<=");
      Serial.print(cMonth);
      Serial.print("<=");
      Serial.println(dst_end_month[current_index]);

      if(cMonth == dst_start_month[current_index] && cDay >= dst_start_day[current_index])
      {
        Serial.print("DST2: ");
        Serial.print(cDay);
        Serial.print(">=");
        Serial.println(dst_start_day[current_index]);
        isdst = true;
      }
      else if(cMonth == dst_end_month[current_index] && cDay <= dst_end_day[current_index]) 
      {
        Serial.print("DST3: ");
        Serial.print(cDay);
        Serial.print("<=");
        Serial.println(dst_start_day[current_index]);
        isdst = true;
      }
      else if(dst_start_month[current_index] < cMonth && cMonth < dst_end_month[current_index])
      {
        Serial.print("DST: ");
        Serial.print(dst_start_month[current_index]);
        Serial.print("<");
        Serial.print(cMonth);
        Serial.print("<");
        Serial.println(dst_end_month[current_index]);
       isdst = true;
      }
    }

    last_offset_change = currentMillis;
    Serial.printf("Switching to %s %d\r\n", labels[current_index], offsets[current_index]);
    current_offset = offsets[current_index];
    current_label = labels[current_index];
    if(isdst)
    {
      current_label = dst_labels[current_index];
      current_offset = dst_offset[current_index];
    }
    if(current_offset < 0)
    {
      sign = '-';
      Serial.println("Set sign to negative");
    }
    else
    {
      sign = '+';
      Serial.println("Set sign to positive");
    }
  }

}

void loop_draw_time(uint32_t actualTime, uint32_t actualMs)
{
  /**** Generate Text and fill screen ****/
  char text[60];
  sprintf(text, "%04d-%02d-%02dT%02d:%02d:%02d.%03d%c%02d:00 %s",
          year(actualTime), 
          month(actualTime),
          day(actualTime),
          hour(actualTime),
          minute(actualTime), 
          second(actualTime),
          actualMs,
          sign,
          abs(current_offset),
          current_label
         );

  matrix.fillScreen(LOW);
  
  int x = 0;
  for(int i = 0; i < strlen(text); i++)
  {
    if(text[i] == ':')
      x -= 1;
    matrix.drawChar(x+pixel_offset_x, pixel_offset_y, text[i], HIGH, LOW, 1);
    if(text[i] == ':')
      x += 4;
    else
      x += 6;
  }
  matrix.write(); 
}

unsigned long last_print = 0;
void loop_print(unsigned long currentMillis, uint32_t gpsTime, uint32_t gpsMs, uint32_t ntpTime, uint32_t ntpMs)
{
  if(currentMillis - last_print > 10000)
  {
    last_print = currentMillis;
    Serial.print("GPS: ");
    Serial.print(gpsTime);
    Serial.print(".");
    Serial.println(gpsMs);
    
    Serial.print("NTP: ");
    Serial.print(ntpTime);
    Serial.print(".");
    Serial.println(ntpMs);
    
    Serial.print("Offset ");
    if(gpsTime > ntpTime || (gpsTime == ntpTime && gpsMs > ntpMs))
      Serial.print("(GPS ahead by):");
    else
      Serial.print("(NTP ahead by):");
    Serial.print(gpsTime - ntpTime);
    Serial.print(".");
    Serial.println(gpsMs - ntpMs);
  }
}

/********************************************************************** SETUP AND LOOP **********************************************************************/
void setup() {
  Serial.begin(115200);
  delay(10); // Allow serial some time to initialize
  Serial.println("\r\n");
  setup_matrix();

  for(int i = 0; i < 10; i++)
  {
    reset_spi();
    draw_loading();
    delay(50); // Allow in-rush current to stabilize, so that we can clear corruption it can cause
  }
  
  setup_gps();
  setup_wifi();
  setup_ntp();
}


// Used to calculate accurate time based on the ESP8266's millis()
// system clock and either ntp or gps source
unsigned long last_time_sync = 0;
uint32_t last_time = 0;
uint32_t last_time_ms = 0;
unsigned long gps_time_sync = 0;
uint32_t gps_time = 0;
uint32_t gps_time_ms = 0;

void loop() {
  unsigned long currentMillis = millis();

  // Moves the print around periodically so that LEDS burn out less (once / 10 min)
  loop_rotate_offset(currentMillis);

  // Resets the display periodically in case of SPI corruption (once / min)
  loop_reset_spi(currentMillis);

  // Requests an NTP update (once / 30 min)
  loop_request_ntp_update(currentMillis);

  // Reads any NTP packets off UDP and updates NTP time (on demand)
  loop_read_ntp_packet(currentMillis, &last_time_sync, &last_time, &last_time_ms);
  
  // Reads any GPS serial comms and updates GPS time (once / min)
  loop_update_gps(currentMillis, &gps_time_sync, &gps_time, &gps_time_ms);

  // Update ntp based clock with millis elapsed since last NTP update
  uint32_t ntpMs = last_time_ms + (currentMillis - last_time_sync) % 1000;
  uint32_t ntpTime = last_time + (currentMillis - last_time_sync) / 1000 + last_time_ms / 1000;
  ntpMs = ntpMs % 1000;

  // Update gps based clock with millis elapsed since last GPS update
  uint32_t gpsMs = gps_time_ms + (currentMillis - gps_time_sync) % 1000;
  uint32_t gpsTime = gps_time + (currentMillis - gps_time_sync) / 1000 + gps_time_ms / 1000;
  gpsMs = gpsMs % 1000;

  // Print the time to the console so we can compare GPS vs NTP for debugging (once / min)
  loop_print(currentMillis, gpsTime, gpsMs, ntpTime, ntpMs);

  // Changes the timezone every few minutes
  loop_set_time_zone(gpsTime);

  // Add in the time zone offset
  ntpTime += current_offset * 60 * 60;
  gpsTime += current_offset * 60 * 60;

  // Draw the time to the display
  loop_draw_time(gpsTime, gpsMs);
}
