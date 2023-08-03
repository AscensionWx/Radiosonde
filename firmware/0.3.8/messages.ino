
void send_status( bool want_confirmed )
{
  MESSAGE_PENDING = true;
  uint8_t txBuffer[3];
  LoraEncoder encoder(txBuffer);

  encoder.writeBitmap( if_T_and_RH_sensor(),
                       if_P_sensor(),
                       SSD1306_FOUND,
                       gps_available(),
                       if_good_gps_quality(),
                       IF_WEB_OPENED,
                       if_received_downlink(),
                       IF_LAUNCHED );
  encoder.writeUint16( (uint16_t)(PRESSURE*10.0) ); // Convert hpa to deci-paschals

  // Battery / solar voltage
  //encoder.writeUint8(0);

  lorawan_cnt(COUNT);

  Serial.println("");
  Serial.print("If T and RH sensor found: ");
  if (if_T_and_RH_sensor()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If P sensor found: ");
  if (if_P_sensor()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If OLED found: ");
  if (SSD1306_FOUND) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If GPS currently available: ");
  if (gps_available()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If Good GPS quality currently: ");
  if (if_good_gps_quality()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If Web App opened: ");
  if (IF_WEB_OPENED) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If received downlink: ");
  if (if_received_downlink()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If has been launched: ");
  if (IF_LAUNCHED) Serial.println("TRUE");
  else Serial.println("FALSE");
  
  Serial.println("Sending balloon status message.");
  lorawan_send(txBuffer, sizeof(txBuffer), STATUS_PORT, want_confirmed);
  COUNT++;
}

void send_status() {
  send_status( false );
}

void send_authentication( String account ) {

  MESSAGE_PENDING = true;
  uint8_t txBuffer[13];  // 8 for uint64   ... 13 for string (12 characters plus \n)

  boolean confirmed = false;
  lorawan_cnt(COUNT);
  Serial.println("Sending balloon authentication message.");

  account.getBytes(txBuffer, 13);
  
  lorawan_send(txBuffer, sizeof(txBuffer), AUTHENTICATION_PORT, confirmed);

}

void print_observation() {
  
  char buffer[40];

  snprintf(buffer, sizeof(buffer), "Unixtime: %u\n", TIMESTAMP);
  Serial.println(buffer);
  screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Latitude: %10.6f\n", LATITUDE);
  Serial.println(buffer);
  screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Longitude: %10.6f\n", LONGITUDE);
  Serial.println(buffer);
  screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Altitude: %10.1f\n",ELEVATION_GPS);
  Serial.println(buffer);
  //screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "HDOP: %10.2f\n", gps_hdop());
  Serial.println(buffer);
  //screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Elevation_2: %10.1f\n", ELEVATION);
  Serial.println(buffer);
  //screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Temperature: %10.2f\n", TEMPERATURE);
  Serial.println(buffer);
  screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Humidity: %10.1f\n", HUMIDITY);
  Serial.println(buffer);  
  //screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Pressure: %10.3f\n", PRESSURE);
  Serial.println(buffer);
  //screen_print(buffer);
  snprintf(buffer, sizeof(buffer), "Lux Avg: %10.1f\n", LUX_AVG);
  Serial.println(buffer);
  screen_print(buffer);
}

void send_observation() {
  //LoraMessage message;

  MESSAGE_PENDING = true;

  uint8_t txBuffer[26];
  LoraEncoder encoder(txBuffer);

  encoder.writeUnixtime( TIMESTAMP );
  encoder.writeLatLng( LATITUDE, LONGITUDE );
  encoder.writeUint16( ELEVATION_GPS );
  //encoder.writeRawFloat( gps_hdop() ); // possibly remove this field
  encoder.writeUint16( (uint16_t)( (ELEVATION < 0) ? 0 : ELEVATION) ); // In meters (send 0 if negative)
  encoder.writeUint16( (uint16_t)(PRESSURE*10.0) ); // Convert hpa to deci-paschals
  encoder.writeTemperature(TEMPERATURE);
  encoder.writeHumidity(HUMIDITY);
  encoder.writeUint16( (uint16_t)(LUX_AVG / 10.0) ); // divide by 10 to meet physical range 0 to 200,000 lux
  //encoder.writeBytes( LAUNCH_ID, 8 ); // Add 8 bytes for the launch_id
  encoder.writeBitmap( if_T_and_RH_sensor(),
                       if_P_sensor(),
                       SSD1306_FOUND,
                       gps_available(),
                       if_good_gps_quality(),
                       IF_WEB_OPENED,
                       if_received_downlink(),
                       IF_LAUNCHED );

 Serial.println("Built message successfully");

#if LORAWAN_CONFIRMED_EVERY > 0
  bool confirmed = (COUNT % LORAWAN_CONFIRMED_EVERY == 0);
#else
  bool confirmed = false;
#endif

  lorawan_cnt(COUNT);

/*
  Serial.println("");
  Serial.print("If T and RH sensor found: ");
  if (if_T_and_RH_sensor()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If P sensor found: ");
  if (if_P_sensor()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If OLED found: ");
  if (SSD1306_FOUND) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If GPS currently available: ");
  if (gps_available()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If Good GPS quality currently: ");
  if (if_good_gps_quality()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If Web App opened: ");
  if (IF_WEB_OPENED) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If received downlink: ");
  if (if_received_downlink()) Serial.println("TRUE");
  else Serial.println("FALSE");
  Serial.print("If has been launched: ");
  if (IF_LAUNCHED) Serial.println("TRUE");
  else Serial.println("FALSE");
  */
  
  Serial.println("Sending balloon observation message.");
  lorawan_send(txBuffer, sizeof(txBuffer), OBSERVATION_PORT, confirmed);
  //lorawan_send(message.getBytes(),message.getLength(), OBSERVATION_PORT, confirmed);

  COUNT++;
}

uint32_t get_count() {
  return COUNT;
}

void set_msg_interval() {

  if ( ELEVATION < 150 && PRESSURE > 800 )
    MSG_INTERVAL = MSG_INTERVAL_150M;
  else if ( PRESSURE > 950 )
    MSG_INTERVAL = MSG_INTERVAL_950;
  else if ( PRESSURE > 900 )
    MSG_INTERVAL = MSG_INTERVAL_900;
  else if ( PRESSURE > 800 )
    MSG_INTERVAL = MSG_INTERVAL_800;
  else if ( PRESSURE > 700 )
    MSG_INTERVAL = MSG_INTERVAL_700;
  else if ( PRESSURE > 600 )
    MSG_INTERVAL = MSG_INTERVAL_600;
  else if ( PRESSURE > 500 )
    MSG_INTERVAL = MSG_INTERVAL_500;
  else if ( PRESSURE > 400 )
    MSG_INTERVAL = MSG_INTERVAL_400;
  else if ( PRESSURE > 300 )
    MSG_INTERVAL = MSG_INTERVAL_300;
  else
    MSG_INTERVAL = MSG_INTERVAL_200;
  
}

void callback(uint8_t message) {
  /* To be removed
  if (EV_JOINING == message) screen_print("Joining TTN...\n");
  if (EV_JOINED == message) screen_print("TTN joined!\n");
  if (EV_JOIN_FAILED == message) screen_print("TTN join failed\n");
  if (EV_REJOIN_FAILED == message) screen_print("TTN rejoin failed\n");
  if (EV_RESET == message) screen_print("Reset TTN connection\n");
  if (EV_LINK_DEAD == message) screen_print("TTN link dead\n");
  if (EV_ACK == message) screen_print("ACK received\n");
  if (EV_PENDING == message) screen_print("Message discarded\n");
  if (EV_QUEUED == message) screen_print("Message queued\n");
  */

  if (EV_JOINED == message && if_otaa()) {
    // OTAA Join request successful
    GATEWAY_IN_RANGE = true;
  }

  if (EV_TXCOMPLETE == message) {
    //screen_print("Message sent\n");
    
    MESSAGE_PENDING = false;
    if (LMIC.txrxFlags & TXRX_ACK)
    {
        GATEWAY_IN_RANGE = true;
    }

  }

  if (EV_RESPONSE == message) {

    if (LMIC.dataLen) 
    {
      GATEWAY_IN_RANGE = true; // if we receive downlink, we know this is true

      int auth_size = 1;
      char* msg_start = (char*)(LMIC.frame+LMIC.dataBeg);

      char auth_status[ auth_size ];
      strncpy( auth_status, msg_start, 1 ); // Get first byte
      auth_status[1] = '\0'; // null terminate to make it a string
      if ( strcmp( auth_status , "1" ) == 0 ) // strcmp equals 0 if the two strings are the same
      {
        AUTHENTICATED=true;
      } else {
        AUTHENTICATED=false;
      }
        
      char tmp[25]; // set to size of smartphone screen, etc
      int data_len = LMIC.dataLen - 2;
      strncpy( tmp, msg_start + 2, data_len );
      tmp[ data_len ] = '\0'; // null terminate to make it a string

      DOWNLINK_RESPONSE = tmp; // Store the response as a string

      String respText = "Response:\n" + DOWNLINK_RESPONSE + "\n";
      screen_print(respText.c_str());
      Serial.println();
      Serial.println("Authenticated: ");
      Serial.println(AUTHENTICATED);
      Serial.println("LoRaWAN response: " + DOWNLINK_RESPONSE);   
    }
/*
    if (!gps_available()) {
      screen_print("Warning:\n");
      screen_print("  GPS not yet locked.\n");
    } else 
    {
      screen_print(" \n \n");
    }
*/
    screen_print(" \n \n");
  } // end of check for response
}
