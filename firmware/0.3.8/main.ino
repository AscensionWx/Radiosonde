/*

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Unless otherwise stated, all other files under this Arduino program
fall under the GPL license.

This code requires the MCCI LoRaWAN LMIC library
by IBM, Matthis Kooijman, Terry Moore, ChaeHee Won, Frank Rose
https://github.com/mcci-catena/arduino-lmic

*/

#pragma once

#include "config_hardware.h"

#include <Arduino.h>
#include <lmic.h>
//void lorawan_register(void (*callback)(uint8_t message));

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "config_user.h"
#include "rom/rtc.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include <Math.h>
#include <axp20x.h>

#include <LoraMessage.h>
#include <LoraEncoder.h>

#include <WiFi.h>

const char * APP_VERSION = "0.3.8";
String APP_VERSION_STR = "0.3.8";

// Set web server port number to 80
WiFiServer SERVER(80);
WiFiClient CLIENT;
int WIFI_DEFAULT_WINDOW = 2*60*1000;
bool WIFI_OPEN;

// Used in power management
//  Will be used to turn off GPS, check if charging, battery etc
AXP20X_Class axp;

bool SSD1306_FOUND = false;
bool BME280_FOUND = false;
bool SHT21_FOUND = false;
bool SPL06_FOUND = false;
bool AXP192_FOUND = false;
bool RELAIS_ON = false;

// Message _counter, stored in RTC memory, survives deep sleep
RTC_DATA_ATTR uint32_t COUNT = 0;

// Calculated at each step based on pressure
int MSG_INTERVAL;
bool GATEWAY_IN_RANGE;
String DOWNLINK_RESPONSE = "";
bool AUTHENTICATED = false;
bool BATT_CHARGING = false;
int8_t APPROX_UTC_HR_OFFSET = 0;
bool MESSAGE_PENDING = false;

// Launch-specific values
float SFC_PRESSURE = 0.0;
bool IF_WEB_OPENED = false;
bool IF_LAUNCHED = false;
uint8_t LAUNCH_ID[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } ; // For launch_id downlink
int LAUNCH_CHECK_INTERVAL = 5000;

// BME 280 environment
unsigned int TIMESTAMP;
float LATITUDE;
float LONGITUDE;
float ELEVATION_GPS;
float TEMPERATURE; // celcius
float HUMIDITY; // percent (range 0 to 100)
float PRESSURE; // hpa
float V_TEMPERATURE; // kelvin
float ELEVATION; // meters
float LUX_AVG;


// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------



//////////////////////////////////////////
///////////// ARDUINO SETUP //////////////
//////////////////////////////////////////
void setup() {
  
  // Debug
  #ifdef DEBUG_PORT
  DEBUG_PORT.begin(SERIAL_BAUD);
  #endif

  while ( millis() < 500 ); // Brief wait for serial baud

  Serial.begin(115200);

  // Buttons & LED
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2Cdevice();

  // Display
  screen_setup();
  screen_clear();
  
  #ifndef IGNORE_TPRH_CHECK // If want to check for P,T, RH sensor
  if (!if_T_and_RH_sensor() || !if_P_sensor())
  {
    screen_print("[ERR]\n");
    screen_print("No weather sensor \n");
    screen_print("connected! Check wiring.");
    Serial.println("[ERR] No weather sensor connected!");
    delay(MESSAGE_TO_SLEEP_DELAY);
    screen_off();
    sleep_forever();
  }
  
  #endif // 
  
  // Init BME280
  bme_setup();

  // Init BH1750 and Lux variable
  light_setup();

  // Init GPS
  Serial.println("Setting up GPS (uses serial)");
  Serial.println("Note that Serial monitor may not display properly after this point.");
  delay(50); // Allow serial to catch up
  gps_setup();
  //GPS_SetHAB();

  // TTN setup
  if (!lorawan_setup()) {
    screen_print("[ERR] Radio module not found!\n");
    Serial.println("[ERR] Radio module not found!");
    delay(MESSAGE_TO_SLEEP_DELAY);
    screen_off();
    sleep_forever();
  }

   // Print initial header at top
  char screen_buff[40];
  strncpy( screen_buff, APP_VERSION, 5 );
  strncat( screen_buff, "   ", 3 );
  strncat(screen_buff, get_devname(), 12);
  strncat( screen_buff, "   ", 3);
  strncat( screen_buff, get_freq(), 3);
  screen_print(screen_buff, 0, 0);

  lorawan_register(callback);
  lorawan_sf(LORAWAN_SF);
  lorawan_adr(LORAWAN_ADR);

  Serial.println("Beginning Join attempt.");
  lorawan_join();
  if(!LORAWAN_ADR){
    LMIC_setLinkCheckMode(0); // Link check problematic if not using ADR. Must be set after join
  }

  // Show logo on first boot (if OLED attached)
  //screen_print(APP_NAME " " APP_VERSION " " DEVNAME " " get_freq(), 0, 0 ); // print firmware version in upper left corner

  Serial.println("Waiting in Join window.");
  screen_show_logo();
  screen_update();
  time_loop_wait( LOGO_DELAY ); // If Join_accept message received during this time, it will trigger LMIC callback function

  // Wait a bit longer in case OTAA response miss or poor connection
  if ( !GATEWAY_IN_RANGE )
  {
    // Listen for LoRaWAN response
    Serial.println("Waiting for OTAA Joined response.");
    screen_print("Please wait!\n\n");
    screen_print("Listening for LoRaWAN\n");
    screen_print("network coverage...\n");

    int time_check_start = millis();
    time_loop_wait( 8*1000 ); 
    while( !GATEWAY_IN_RANGE && !time_loop_check( time_check_start, 30*1000 ) )
    {
      send_status(true); // Receive possible missed join message or downlinks
      screen_update();
      time_loop_wait( 8*1000 );// Wait 5 seconds
    }
  }


  // Check that at least one confirmed message was received
  #if !defined(IGNORE_GATEWAY_CHECK)
  if (!GATEWAY_IN_RANGE) 
  { 
    screen_print("[ERR]\n");
    screen_print("No LoRaWAN coverage!\n\n");
    screen_print("Please find a better\n");
    screen_print("place outdoors.\n");
    Serial.println("[ERR] LoRaWAN network not connected!");
    delay(MESSAGE_TO_SLEEP_DELAY);
    screen_off();
    sleep_forever();
  }
  #endif

  //screen_show_qrcode();
  //screen_update();
  screen_print("LoRaWAN Connected!\n");
  screen_print("WiFi AP:  AscensionWx\n");
  screen_print("  password:  balloons\n");
  screen_print("\n");
  screen_print("Then go to  192.168.4.1\n");

  // Init wifi access point
  WiFi.softAP(SSID, PASSWORD);
  WIFI_OPEN = true;

  // Defines how async server handles GET requests
  //initServer();
  SERVER.begin();

  MSG_INTERVAL = 30000; // Status message this many millis
  uint32_t last_msg = 0;

  // Check if surface pressure has changed over 3 seconds
  uint32_t last_sfc = 0;

  while ( true ) {

    // Frequently check pressure change to see if launched
    if ( millis() - last_sfc > LAUNCH_CHECK_INTERVAL) {
      IF_LAUNCHED = checkIfLaunched();
      update_lux_avg( get_light() );
      last_sfc = millis();
    }

    if ( IF_LAUNCHED )
      break;
  
    if ( millis() - last_msg > MSG_INTERVAL) {

      // Send status message every MSG_INTERVALnumber of seconds
      //   Note that no downlinks will be able to be received, because
      //     we prioritize having checkServer() be reactive instead of
      //     just doing os_runloop_once to listen for downlinks
      send_status();
      last_msg = millis(); // store for next iteration

      wait_for_message_txcomplete();
      
    } // End status message interval loop

    if ( WIFI_OPEN ) {
      checkServer(); // Should be called frequently to check if smartphone connected
      gps_read(); // Populates time header and gets us fix progress
      screen_loop();
      os_runloop_once();
      
      if ( !IF_WEB_OPENED && millis() > WIFI_DEFAULT_WINDOW ) {
        SERVER.end();
        WiFi.softAPdisconnect(true);
        WIFI_OPEN=false;

        screen_print("\n");
        screen_print("WiFi Window Closed\n");
        screen_print("Screen will turn off soon\n");
        screen_print("\n");
        screen_print("\n");
        
      }
    }
    else // if WiFi window closed
    { 
      // Allow sleep between BME280 check
      int slp_time = LAUNCH_CHECK_INTERVAL - ( millis() - last_sfc );
      if ( slp_time > 100 ) {
        sleep_light_millis( slp_time - 50 );
        screen_off();
      }

      os_runloop_once();
      sysTimeoutCheck(); // make sure device hasn't been kept on for over 1 hour
    }

  }// End IF_LAUNCHED check

  // Reset send interval for observations
  set_msg_interval();
  
  // Turn off wifi
  if ( WIFI_OPEN )
  {
    SERVER.end(); // close access to the server
    WiFi.softAPdisconnect(true); // Ends wifi connection completely
    WIFI_OPEN = false;
  }

  Serial.println("Finished closing wifi");
  
}// End Arduino setup() method

/////////////////////////////////////////
///////////// ARDUINO LOOP //////////////
/////////////////////////////////////////
void loop() {

  int time_start = millis();
  
  // Wait a few sec for GPS to communicate NMEA sentences over Serial
  gps_time_loop_wait( GPS_WAIT );

  set_obs_timestamp();
  save_gps_data();
  read_all_sensors();

  // Note that this call sends the data sampled from the prior sleep
  print_observation();
  //Serial.println("TRANSMITTING");
  send_observation();

  //wait_for_message_txcomplete();
  time_loop_wait( LMIC_WAIT );

  int loop_time = millis() - time_start;
  int sleep_ms = MSG_INTERVAL - loop_time;
  if ( sleep_ms > 50 ) {
    //debug_print( "sleep millis", sleep_ms );
    sleep_light_millis( sleep_ms );
  }

  // Reset send interval if needed
  set_msg_interval();

  // Check that unphysical time limit isn't reached
  sysTimeoutCheck(); 

}
