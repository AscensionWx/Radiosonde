/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  First written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.

  Regarding calculation of elevation:
    - It's only possible with a rising sensor
    - The approximation is made under the assumption of 
         hydrostatic balance (almost always) in meteorology
    - Written by Nicolas Lopez - Kanda Weather Group LLC
    - kandaweather@gmail.com
  
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SHT21.h>
#include <SPL06-007.h>
#include <BH1750.h>

char bme_char[32]; // used to sprintf for Serial output

Adafruit_BME280 bme; // I2C
SHT21 sht;
BH1750 lightMeter;
  
float v_temperature_last;
float elevation_last;
float pressure_last;

const uint8_t psfc_len = 4;
float psfc_data[ psfc_len ];

// Keeps last 60 seconds
const int lux_increment = 5; // 5 seconds per datapoint
const int lux_window = 60*5; // save last 5 minutes
const uint8_t lux_len = lux_window/lux_increment;
float lux_data[ lux_len ];
uint32_t last_lux_update = 0;

void bme_setup() {

    bool status;
    
    if (BME280_FOUND) {
       status = bme.begin( BME280_ADDRESS_0x76 );
       if (!status) // if still not found, try 0x77
          status = bme.begin( BME280_ADDRESS_0x77 );
    }
    else if (SPL06_FOUND) {
      SPL_init( ); // Has hardcoded 0x77 into the library function
      status = true;
    }
    
    if (!status ) {
        Serial.println("Could not find a valid weather sensor, check wiring!");
    }

    // Wake up sensor
    delay(20);
    get_pressure_hpa();
    delay(500);
    
    // Take first guess at surface pressure
    while (SFC_PRESSURE < 150.0 || SFC_PRESSURE > 1020.0)
    {
      // Note THAT SPL06_001 sensor currently does not work properly.
      SFC_PRESSURE = get_pressure_hpa();
      delay(500);
    }

    int i = 0;
    while( i < 4 )
    {
      psfc_data[i] = 0;
      i++;
    }

    // Compute prior variables
    pressure_last = SFC_PRESSURE;
    v_temperature_last = calc_v_temperature( get_raw_temperature(), 
                                             get_humidity(), 
                                             pressure_last );
    elevation_last = 0;

}

void light_setup()
{
  lightMeter.begin();

  // Set all lux values to 0
  for (int i = 0; i < lux_len; i++) {
    lux_data[i] = 0.0;
  }
  
}

void read_all_sensors() {
  
      // Store the last variables
      pressure_last = PRESSURE;
      v_temperature_last = V_TEMPERATURE;
      elevation_last = ELEVATION;
  
      // Update environment state
      time_loop_wait(20); // BME280 has sample rate of 100Hz
      TEMPERATURE = get_raw_temperature();
      time_loop_wait(20); 
      HUMIDITY = get_humidity();
      time_loop_wait(20);
      PRESSURE = get_pressure_hpa(); // Store as hpa

      // get the lux value and calibrate the temperature
      LUX_AVG = update_lux_avg( get_light() );
      calibrateTemperature();

      // Calculate virtual temperature given new values
      V_TEMPERATURE = calc_v_temperature( TEMPERATURE, 
                                              HUMIDITY, 
                                              PRESSURE );

      // Finally we use new virtual temperature in 
      ELEVATION = calc_elevation();
}

float update_lux_avg( float lux )
{

  if ( last_lux_update == 0 ) {
    last_lux_update = millis();
    return 0.0;
  }

  // Convert ms to seconds
  float interval = (millis() - last_lux_update)/1000;
  int increment = round(interval / lux_increment);
  
  // Shift all the data back by interval
  for (int i = lux_len-1; i > increment-1; i--){        
    lux_data[i]=lux_data[i-increment];
  }

  // Populate initial parts of the array
  for (int i=0; i < increment; i++)
    lux_data[i] = lux;

  // Finally get the average of the list
  float total = 0;
  for (int i = 0; i < lux_len; i++) {
    total = total + lux_data[i];
  }

  last_lux_update = millis();

  float avg = total/lux_len;
  return avg;
}

void calibrateTemperature() {
  
  float internal_offset = 0;
  if ( BME280_FOUND ) internal_offset = 1.2; // bme280 is known to self-heat

  // Daytime/sunny enclosure heating plus ambient light in aperture
  // This offset is enclosure-specific, and 
  //    should be physically tested in several daylight/cloudy conditions.
  // At night might give small non-zero results if there is blinking light somewhere inside the enclosure
  float sunlight_offset = LUX_AVG/7000.0; 

  //debug_print("sunlght offset", sunlight_offset);

  TEMPERATURE = TEMPERATURE - (internal_offset + sunlight_offset);
}

float get_humidity() {
    float rh = MISSING_FLOAT;
    if ( BME280_FOUND ) rh = bme.readHumidity();
    else if ( SHT21_FOUND ) rh = sht.getHumidity();
    
    return rh;
}

float get_pressure_hpa() {
    float p = MISSING_FLOAT;
    if ( BME280_FOUND ) p = bme.readPressure() / 100.0; // convert to hpa
    // be careful... get_pressure referenced below is spl06 library function
    else if ( SPL06_FOUND ) p = get_pressure(); // returns already in hpa

    return p;
}

float get_raw_temperature() {
    float t = MISSING_FLOAT;
    if ( BME280_FOUND ) t = bme.readTemperature();
    else if ( SHT21_FOUND ) t = sht.getTemperature();
    
    return t;
}

float get_light()
{
  float lux = lightMeter.readLightLevel();
  //debug_print("Light", lux);
  return lux;
}

float calc_v_temperature(float t, float rh, float p) {
  
  // calc saturation vapor pressure es(T)
  // https://journals.ametsoc.org/jamc/article/57/6/1265/68235/A-Simple-Accurate-Formula-for-Calculating
  float es;
  if ( t>0 )
  {
    es = exp( 34.494 - (4924.99/(t+237.1)) ) /  pow(t+105, 1.57);
  } else {
    es = exp( 43.494 - (6545.8/(t+278)) ) /  pow(t+868, 2);
  }
  
  // calc actual vapor pressure using  rh = e/es
  float e = (rh/100)*es;
  e = e/100.0; // Convert to hpa
  
  // calc mixing ratio
  float w = 0.62197 * (e / (p-e));

  // calc virtual temperature in Kelvin
  float tv = (1+(0.61*w))*(t+273.15);

  return tv;
  
}

// Also called geopotential height
float calc_elevation() {

  float p1 = pressure_last;
  float tv1 = v_temperature_last;
  float z1 = elevation_last;

  float p2 = PRESSURE;
  float tv2 = V_TEMPERATURE;

  
  // Uses COUNT and last values to get the elevation
  //   p2 and tv2 are the most recent pressure and virtual temp values
  if (COUNT == 0)
  {
     // Turning on device at about 1 meter above surface
     return 1;
  }
  
  float R = 287.058; // specific gas constant for dry air
  float g = calc_gravity(); // gravitational acceleration

  // Approx average virtual temperature of layer
  float tv_avg = tv1 + (tv2 - tv1)/2.0;

  // Use natural logarithm
  float elevation = (R*tv_avg/g)*log(p1/p2) + z1;

  if (elevation < 0 || elevation > 20000)
    elevation = 0;

  return elevation;

}

float calc_gravity(){

  // Do this to offset gravitational changes. For example,
  //   in West Africa, gravity is a little less because being closer to the equator
  //   means centrifigal force is stronger
  // difference can be as much as 0.5% ... which adds up when integrating up the column

  float g = 978031.85; // gravity at elev=0 and equator
  
  if ( gps_available() )
  {
    float lat = (gps_latitude() * 71.0) / 4068.0; // Get lat and convert to radians
    float elev = gps_altitude(); // in meters
    // Latitude correction
    g = g * (1.0 + 0.005278895*sin(lat)*sin(lat));
    // Altitude correction
    g = g + 0.3086*elev;
  }

  return g / 100000.0; // convert to m/s^2 units
}

bool checkIfLaunched()
{ 

  // Get pressure directly to lower latency overhead for esp server
  PRESSURE = get_pressure_hpa();

  #ifdef DEBUG_MODE
  if (millis() > 120000) {  // In debug mode, we start broadcasting observations after 2 minutes
      Serial.println("One minute reached. Setting IF_LAUNCHED to true");
      return true;
  }
  #endif

  if( PRESSURE < FORCE_IF_LAUNCHED_HPA )
    return true;

  // Shift all the data back by 1
  for (int i = psfc_len-1; i > 0; i--){        
    psfc_data[i]=psfc_data[i-1];
  }
  psfc_data[0] = PRESSURE; // Place newest pressure in front

  // Pressure change per second multiplied by seconds
  float pchange_threshold = 0.06*(LAUNCH_CHECK_INTERVAL/1000.0);

  char buffer[40];
  snprintf(buffer, sizeof(buffer), "Setting pchange thresh: %10.6f\n", pchange_threshold);
  Serial.println(buffer);

  return if_p_changed( pchange_threshold );

}

bool if_p_changed( float threshold ) {

  // NOTE that BME280 has relative accuracy of +-0.125hpa which is why we check for
  //   two consecutive pressure decreases

  int index = 0;
  int seq_threshold = 2;
  int count = 0;

  while (index < psfc_len - 1) {
    if ( psfc_data[index+1] - psfc_data[index] > threshold  ) {
      count++;
    } else
      count=0;

    if ( count >= seq_threshold )
    {
      SFC_PRESSURE = psfc_data[psfc_len-1];
      return true;
    }

    index++;
  }

  return false;
  
}

bool if_T_and_RH_sensor()
{
  if (BME280_FOUND) return true;
  else if (SHT21_FOUND) return true;
  else return false;
}

bool if_P_sensor()
{
  if (BME280_FOUND) return true;
  else if (SPL06_FOUND) return true;
  else return false;
}

void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == SSD1306_ADDRESS) {
                SSD1306_FOUND = true;
                Serial.println("ssd1306 display found");
            }
            if (addr == AXP192_SLAVE_ADDRESS) {
                AXP192_FOUND = true;
                Serial.println("axp192 PMU found");
            }
            if ( addr == SHT21_ADDRESS ) {
                SHT21_FOUND = true;
                Serial.println("SHT21 sensor found");
            }
            // This is a slight hack because SPL06 and some BME280 share 0x76 and 0x77 address
            //   Warning:  Only use if SPL06 SHT21 are guaranteed on the same board and working!
            if ( addr == SPL06_ADDRESS_0x76 && SHT21_FOUND ) {
                SPL06_FOUND = true;
                Serial.println("SPL06 sensor found");
            }
            if ( addr == SPL06_ADDRESS_0x77 && SHT21_FOUND ) {
                SPL06_FOUND = true;
                Serial.println("SPL06 sensor found");
            }
            if ( addr == BME280_ADDRESS_0x76 && !SHT21_FOUND  ) {
                BME280_FOUND = true;
                Serial.println("BME280 sensor found");
            }
            if ( addr == BME280_ADDRESS_0x77 && !SHT21_FOUND ) {
                BME280_FOUND = true;
                Serial.println("BME280 sensor found");
            }
     
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}
