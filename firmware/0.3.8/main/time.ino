

#include "time.h"

int last_rtc_sync_gps = 0;


bool time_loop_check( int time_loop_start , int wait_millis )
{
  if ( millis() > time_loop_start + wait_millis )
  {
    return true;
  } else {
    return false;
  }
}

void time_loop_wait( int milliseconds )
{
  int start = millis();
  while ( !time_loop_check( start, milliseconds ) )
  {
    os_runloop_once();
  }
}

void wait_for_message_txcomplete()
{
  // Relies on callback function to set MESSAGE_PENDING to false
  //   once EV_TXCOMPLETE event has occurred
  while( MESSAGE_PENDING )
  {
    os_runloop_once();
  }
  os_runloop_once();
}

void sysTimeoutCheck()
{
    // If device has been in setup mode for over an hour, give warning
    //    and stop broadcasting messages.    
    int timeout;
    if ( IF_LAUNCHED )
      timeout = 1000*60*60*3; // 3 hours
    else
      timeout = 1000*60*60; // 1 hour

    if ( millis() > timeout ) {
      screen_on();
      screen_clear();
      screen_print( "\n" );
      screen_print( "Setup time limit reached.\n" );
      screen_print( "Please restart.\n" );
      screen_print( "\n" );
      screen_print( "\n" );
      sleep_forever();
    }
}

struct tm tm_construct( int yr, int month, int mday, int hr, int minute, int sec, int isDst )
{
  struct tm tm;

  tm.tm_year = yr - 1900;   // Set date
  tm.tm_mon = month-1;
  tm.tm_mday = mday;
  tm.tm_hour = hr;      // Set time
  tm.tm_min = minute;
  tm.tm_sec = sec;
  tm.tm_isdst = isDst;  // 1 or 0

  return tm;
}

int seconds_since( struct tm tm )
{
  time_t now;
  time(&now);

  uint64_t num_seconds = difftime(now, mktime(&tm));
  return num_seconds;
}

void setTime(struct tm tm){
  // Sets the ESP32 time using struct tm
  time_t t = mktime(&tm);
  
  Serial.printf("Setting time on ESP32: %s", asctime(&tm));
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
}

void set_obs_timestamp() {
  
  if ( gps_time_age() < 1500 ) {
    sync_rtc_with_gps(); // update esp32's clock time
  }

  if (last_rtc_sync_gps == 0) {
    TIMESTAMP = 0;
    return;
  }
  
  if ( last_rtc_sync_gps > millis() - 25000 )
  {
    TIMESTAMP = get_unix_time();
  } 
  else if ( last_rtc_sync_gps > millis() - 60000 &&
            TEMPERATURE > 0 ) // cold temperatures lead to rtc clock drift
  {
    TIMESTAMP = get_unix_time();
  }
  else // too much uncertainty
    TIMESTAMP = 0;
  
}

void calc_utc_offset() {

  if( !if_gps_on() ) {
    Serial.println("Error. Cannot calculate UTC offset while GPS is off.");
  }
  
  // Local timezone is changes roughly every 15 degrees
  // UTC (offset=0) is also prime meridian, so this is relatively easy.
  //   - offset is negative value for degrees West , positive for degrees east
  float longitude = gps_longitude();
  APPROX_UTC_HR_OFFSET = int( longitude/15.0 );
  
}

int8_t get_approx_local_hour() {

  struct tm tm;
  int hour;

  // Gets the ESP32 local time
  getLocalTime(&tm);

  hour = tm.tm_hour + APPROX_UTC_HR_OFFSET;
  
  if (hour < 0) {// Prevent negative value for hour
    hour = hour + 24;
  }
  else if (hour > 23) { // Prevent hour going over 23
    hour = hour - 24;
  }

  return hour;
  // return APPROX_UTC_HR_OFFSET; // for testing

}

void sync_rtc_with_gps()
{
  if( !if_gps_on() ) {
    Serial.println("Error. Attempting to do RTC sync while GPS is off.");
  }
  
  // WARNING: This function should only be called if the GPS is on

  // Initialize tm struct
  struct tm gps_time = tm_construct( gps_year(), gps_month(), gps_day(), gps_hour(), gps_minute(), gps_second(), 0 );
 
  // Set ESP 32 clock to match GPS's UTC time
  setTime(gps_time);

  last_rtc_sync_gps = millis();

}

// NOTE that this function requires a previous call to setTime() or it will result in 3+ second hangup
int get_unix_time() {
  // Uses last time_refresh, esp32's time, and time_drift to get unix_time.

  time_t now;
  struct tm timeinfo;

  if( !getLocalTime(&timeinfo) ) {
    Serial.println("getLocalTime failed.");
    return 0;
  };

  time(&now);

  return now;

  // Approx time based on last_time_refresh and time_drift
  //uint32_t millis_since_refresh = millis() - last_time_refresh;
  //uint64_t unix_time = ( now + (millis_since_refresh*time_drift) );

  //return unix_time;

}
