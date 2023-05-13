#include <eosio/eosio.hpp>
//#include <eosio/time.hpp>
#include <math.h> // exponent pow() function
//#include <time.h>

using namespace std;
using namespace eosio;

CONTRACT wxlaunches : public contract {
  public:
    using contract::contract;

    // Should be called by device

    ACTION addobs(name launch_id,
                  uint64_t unix_time_s, 
                  float pressure_hpa, 
                  float temperature_c, 
                  float humidity_percent,
                  float latitude_deg,
                  float longitude_deg,
                  uint16_t gps_elevation_m,
                  uint16_t elevation2_m,
                  uint8_t flags);

    ACTION removeobs( uint64_t unix_time_start,
                      uint64_t unix_time_end);

    ACTION setstation(string owner, 
                      float latitude, 
                      float longitude, 
                      float elevation,
                      float missing,
                      uint8_t launch_window_hrs,
                      uint8_t launch_freq_hrs);

    ACTION websetlaunch(name launch_id, 
                        uint64_t unix_time,
                        float surf_pressure, 
                        name miner, 
                        string device_type,
                        string wxcondition);

    ACTION setflag(uint64_t bit_number,
                    string description);

    ACTION settimeslots(uint64_t first_start_time);

    ACTION settwelvehr(name launch_id, 
                       name miner,
                       string wx12hrcondition);

  private:

    // Local functions (not actions)
    void sendReward( name miner, float last_level, float pressure);
    void updatetimes( uint64_t first_start_time );
    bool checkIfReleased( float surf_pressure, float pressure_hpa );

    TABLE observations {
      uint64_t unix_time;
      name launch_id;
      float pressure_hpa;
      float temperature_c;
      float humidity_percent;
      float latitude_deg;
      float longitude_deg;
      uint16_t elevation_gps_m;
      uint16_t elevation2_m;
      uint8_t flags;

      auto  primary_key() const { return unix_time; }
      uint64_t  get_secondary() const { return launch_id.value;}
    };
    //using observation_index = multi_index<"observations"_n, observations>;
    using observation_index = multi_index<"observations"_n, 
    observations,
    indexed_by<"bylaunch"_n, const_mem_fun<observations, uint64_t, &observations::get_secondary>>
    >;

    TABLE upperweather {
      name devname;
      double station_lat;
      double station_lon;
      uint16_t station_elevation_m;
      name miner;
      uint64_t launch_time_s;
      float station_pressure_hpa;
      uint16_t num_obs_850_500;
      uint16_t max_height_m;
      uint16_t flight_duration_s;
      float avg_rh_850_500;
      float pressure_1000_hpa;
      float pressure_925_hpa;
      float pressure_850_hpa;
      float pressure_700_hpa;
      float pressure_500_hpa;
      float pressure_400_hpa;
      float pressure_300_hpa;
      float pressure_250_hpa;
      float pressure_200_hpa;
      float temperature_1000_c;
      float temperature_925_c;
      float temperature_850_c;
      float temperature_700_c;
      float temperature_500_c;
      float temperature_400_c;
      float temperature_300_c;
      float temperature_250_c;
      float temperature_200_c;
      float dewpt_1000_c;
      float dewpt_925_c;
      float dewpt_850_c;
      float dewpt_700_c;
      float dewpt_500_c;
      float dewpt_400_c;
      float dewpt_300_c;
      float dewpt_250_c;
      float dewpt_200_c;
      uint16_t wnddir_1000_deg;
      uint16_t wnddir_925_deg;
      uint16_t wnddir_850_deg;
      uint16_t wnddir_700_deg;
      uint16_t wnddir_500_deg;
      uint16_t wnddir_400_deg;
      uint16_t wnddir_300_deg;
      uint16_t wnddir_250_deg;
      float wndspd_1000_mps;
      float wndspd_925_mps;
      float wndspd_850_mps;
      float wndspd_700_mps;
      float wndspd_500_mps;
      float wndspd_400_mps;
      float wndspd_300_mps;
      float wndspd_250_mps;

      auto primary_key() const { return devname.value; }
      uint64_t by_launchtime() const { return launch_time_s; }
      double by_latitude() const { return station_lat; }
      double by_longitude() const { return station_lon; }
      uint32_t by_height_reached() const { return max_height_m; }

      // TODO: custom index based on having / not having specific flag
    };
    //using observations_index = multi_index<"observations"_n, observations>;
    typedef multi_index<name("upperweather"), 
                        upperweather,
                        indexed_by<"launchtime"_n, const_mem_fun<upperweather, uint64_t, &upperweather::by_launchtime>>,
                        indexed_by<"latitude"_n, const_mem_fun<upperweather, double, &upperweather::by_latitude>>,
                        indexed_by<"longitude"_n, const_mem_fun<upperweather, double, &upperweather::by_longitude>>,
                        indexed_by<"maxheight"_n, const_mem_fun<upperweather, uint32_t, &upperweather::by_height_reached>>
    > upperweather_table_t;

    TABLE launches {
      name launch_id;
      uint64_t unix_time;
      name miner;
      name assistant;
      float surf_pressure;
      float level_reached;
      string device_type;
      string wxcondition;
      bool if_released;
      float last_known_lat;
      float last_known_lon;
      float last_known_elev;
      string wx12hrcondition;

      auto  primary_key() const { return launch_id.value; }
    };
    typedef multi_index<name("launches"), launches> launches_table_t;

    TABLE station {
       name id;
       string owner;
       float latitude;
       float longitude;
       float elevation;
       float missing;
       uint8_t launch_window_hrs;
       uint8_t launch_freq_hrs;

       auto  primary_key() const { return id.value; }
    };
    typedef multi_index<name("station"), station> station_table_t;

    TABLE flags {
      uint64_t bit_number;
      string description;

      auto primary_key() const { return bit_number; }
    };
    typedef multi_index<name("flags"), flags> flags_table_t;

    TABLE timeslots {
      uint64_t start_time;
      uint64_t end_time;
      string human_window;

      auto primary_key() const { return start_time; }
    };
    typedef multi_index<name("timeslots"), timeslots> timeslots_table_t;

    // List all rewards to be sent out
    TABLE rewards {
      name token_contract;
      bool usd_denominated;
      float amount_per_mb;
      string symbol_letters;
      uint8_t precision; // e.g. 4 for Telos , 8 for Kanda

      auto primary_key() const { return token_contract.value; }
    };
    typedef multi_index<name("rewards"), rewards> rewards_table_t;

    TABLE delphi_data {
      uint64_t id;
      name owner; 
      uint64_t value;
      uint64_t median;
      time_point timestamp;

      uint64_t primary_key() const {return id;}
      uint64_t by_timestamp() const {return timestamp.elapsed.to_seconds();}
      uint64_t by_value() const {return value;}

    };
    typedef eosio::multi_index<"datapoints"_n, delphi_data,
      indexed_by<"value"_n, const_mem_fun<delphi_data, uint64_t, &delphi_data::by_value>>, 
      indexed_by<"timestamp"_n, const_mem_fun<delphi_data, uint64_t, &delphi_data::by_timestamp>>> datapointstable;


};
