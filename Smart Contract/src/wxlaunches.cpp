#include <wxlaunches.hpp>
#include <eosio/asset.hpp>

using namespace std;
using namespace eosio;

ACTION wxlaunches::websetlaunch(name launch_id, 
                                uint64_t unix_time, 
                                float surf_pressure,
                                name miner,
                                string device_type,
                                string wxcondition) {

  // Only self can run this Action
  require_auth(get_self());

  // Check miner account
  check( is_account( miner ), "Miner account invalid.");

  // Check that pressure is physical
  check( surf_pressure > 50.0 && surf_pressure < 1050.0, "Surface pressure is invalid." );

  launches_table_t _launches(get_self(), get_self().value);
  timeslots_table_t _times(get_self(), get_self().value);

  // For testing we approve all launch times
  bool time_approved = true;

  /*
  bool time_approved = false;

  // Loop through launch windows and check for valid launch time
  for ( auto itr = _times.begin(); itr != _times.end(); itr++ )
  {  
    if ( unix_time > itr->start_time && unix_time < itr->end_time )
    {
      time_approved = true;
      //itr++; // Commenting this out allows current window to remain
      updatetimes( itr->start_time ); // Removes old windows
      break;
    }
  }
  */

  if ( time_approved || miner == get_self()  )
  {
    // Add the currently stored variables to the launches table
    _launches.emplace(get_self(), [&](auto& launch) {

      launch.launch_id = launch_id;
      launch.unix_time = unix_time; // Comes as function input
      launch.miner = miner;
      //launch.assistant = assistant; // not implemented
      launch.device_type = device_type;
      launch.wxcondition = wxcondition;
      launch.surf_pressure = surf_pressure;
      launch.level_reached = surf_pressure;
      launch.if_released = false;
      //launch.wx12hrcondition = "N/A"; // not implemented

    });
  } else {
    check( false, "Launch time not valid. Next window");
  }

}

ACTION wxlaunches::setstation(string owner, 
                              float latitude, 
                              float longitude, 
                              float elevation,
                              float missing,
                              uint8_t launch_window_hrs,
                              uint8_t launch_freq_hrs) {

  // Only self can run this Action
  require_auth(get_self());

  station_table_t _station(get_self(), get_self().value);

  // Get field with same id as contract name, if it's there
  auto station_itr = _station.cbegin();

  if ( station_itr != _station.cend()) // if row was found
  {
    _station.erase( station_itr ); // remove the row
  }

  // Update new station info
  _station.emplace(get_self(), [&](auto& station) {

    station.id = get_self();
    station.owner = owner;
    station.latitude = latitude;
    station.longitude = longitude;
    station.elevation = elevation;
    station.missing = missing;
    station.launch_window_hrs = launch_window_hrs;
    station.launch_freq_hrs = launch_freq_hrs;

  });
}

ACTION wxlaunches::settimeslots( uint64_t first_start_time )
{
  require_auth(get_self());

  updatetimes( first_start_time );
}

ACTION wxlaunches::setflag( uint64_t bit_number,
                            string description )
{
  // Only self can run this Action
  require_auth(get_self());

  flags_table_t _flags(get_self(), get_self().value);

  auto flags_itr = _flags.find(bit_number);

  if ( flags_itr == _flags.cend() )
  {
    // If not present then add it
    _flags.emplace(get_self(), [&](auto& flags) {
      flags.bit_number = bit_number;
      flags.description = description;
    });
  } else {
    // modify the bit description
    _flags.modify(flags_itr, get_self(), [&](auto& flags) {
      flags.bit_number = bit_number;
      flags.description = description;
    });
  }
}

ACTION wxlaunches::addobs(name launch_id,
                  uint64_t unix_time, 
                  float pressure_hpa, 
                  float temperature_c, 
                  float humidity_percent,
                  float latitude_deg,
                  float longitude_deg,
                  uint16_t elevation_gps_m,
                  uint16_t elevation2_m,
                  uint8_t flags) 
{
  
  // Make sure eosio linkauth action assigns iot permission to add obs action
  require_auth(get_self());

  // Caution use the following line. It forces iot permission to be passed
  //   Tools like NODE-RED currently pass @active permission by default. You have
  //   to update telos-node-red-contrib to allow use of custom permissions first
  //require_auth(permission_level( get_self(), "iot"_n));

  // Check that pressure is reasonable
  check( pressure_hpa > 0.0, "Pressure is invalid." );

  // Get our own tables
  launches_table_t _launches(get_self(), get_self().value);
  auto launch_itr = _launches.find(launch_id.value);

  float level_reached = launch_itr->level_reached;
  name miner = launch_itr->miner;

  // Update last known position
  _launches.modify(launch_itr, get_self(), [&](auto& launch) {
      launch.last_known_lat = latitude_deg;
      launch.last_known_lon = longitude_deg;
      launch.last_known_elev = elevation_gps_m;
  });

  if ( launch_itr->if_released == false ) 
  {
    if ( checkIfReleased( launch_itr->surf_pressure, pressure_hpa) == false)
    { return; } // still on the ground
    else
    {
      // Mark the launch as "released"
      _launches.modify( launch_itr, get_self(), [&](auto& launch)
      { launch.if_released = true; });

      // Change the timeslots table 
    }
  }
  
  if( pressure_hpa < level_reached ) 
  {
    if ( miner != get_self() ) 
      sendReward( miner, level_reached, pressure_hpa );

    _launches.modify(launch_itr, get_self(), [&](auto& launch) {
      launch.level_reached = pressure_hpa;
    });
  }

  // Get observations table
  observation_index observations(get_self(), get_first_receiver().value);

  // Add the row to the observation set
  observations.emplace(get_self(), [&](auto& obs) {
    obs.launch_id= launch_id;
    obs.unix_time = unix_time;
    obs.pressure_hpa = pressure_hpa;
    obs.temperature_c = temperature_c;
    obs.humidity_percent = humidity_percent;
    obs.latitude_deg = latitude_deg;
    obs.longitude_deg = longitude_deg;
    obs.elevation_gps_m = elevation_gps_m;
    obs.elevation2_m = elevation2_m;
    obs.flags = flags;
  });

}

ACTION wxlaunches::removeobs( uint64_t unix_time_start,
                              uint64_t unix_time_end )
{
    // Only self can run this action
  require_auth(get_self());

  // Get our own two tables
  observation_index observations(get_self(), get_first_receiver().value);

  auto observations_itr = observations.cbegin();

  while ( observations_itr != observations.end() )
  {
    uint64_t time = observations_itr->unix_time;
    if ( time >= unix_time_start && time <= unix_time_end )
    {
      observations_itr = observations.erase( observations_itr );
      // since the iterator has been erased, no need to increment with ++
    }
    else
    {
      observations_itr++;
    }
  }
}

bool wxlaunches::checkIfReleased( float surf_pressure, float pressure )
{
  if ( pressure < surf_pressure - 3.0 ) 
  { return true; } 
  else 
  { return false; } 
}

void wxlaunches::updatetimes( uint64_t first_start_time )
{
  timeslots_table_t _times(get_self(), get_self().value);
  station_table_t _station(get_self(), get_self().value);
  auto station_itr = _station.begin();

  uint8_t launch_freq_hrs = station_itr->launch_freq_hrs;
  uint8_t launch_window_hrs = station_itr->launch_window_hrs;

  // Erase any contents of the table
  //for ( auto itr = _times.begin(); itr != _times.end(); itr++ )
  auto itr = _times.begin();
  while( itr != _times.end() )
  {
    itr = _times.erase( itr );
  }
  for ( int i=0; i<10; i++ )
  {
    uint64_t stime = first_start_time + i*launch_freq_hrs*3600;
    _times.emplace( get_self(), [&](auto& times) {
      times.start_time = stime;
      times.end_time = stime + launch_window_hrs*3600;
      
      // When EOSIO.CDT v1.8 (or v2.0) becomes available on Docker, the following line can be used
      //times.human_window = time_point_sec( (uint32_t)first_start_time ).to_string();
      times.human_window = "N/A";
    });
  }
}

void wxlaunches::sendReward( name miner,
                             float last_level,
                             float pressure ) 
{
  station_table_t _station(get_self(), get_self().value);
  auto station_itr = _station.find(get_self().value);
  //auto station_itr = _station.find("wxstationdat"_n.value);
  
  rewards_table_t _rewards(get_self(), get_self().value);
  auto rewards_itr = _rewards.begin();

  datapointstable _delphi_prices( name("delphioracle"), get_first_receiver().value );
  auto delphi_itr = _delphi_prices.begin(); // gets the last price

  float pressure_change = last_level - pressure;

  // Set reward asset. A value of 10000 is equivalent to 1 Telos
  /*
  eosio::asset reward = eosio::asset( reward_amt, symbol(symbol_code("TLOS"),4));
  string memo = "Reached " + to_string(pressure) + " hpa";

  // Use an inline action to send reward to miner
  // TODO: Permission should be made to somewhere else
  action(
      permission_level{ get_self(), "active"_n },
      "eosio.token"_n, "transfer"_n,
      std::make_tuple( get_self(), miner, reward, memo )
  ).send();

  // Do the same for kanda tokens, but to telokandaone contract
  reward = eosio::asset( reward_amt, symbol(symbol_code("KANDA"),8));
  memo = "";
  action(
    permission_level{ get_self(), "active"_n },
    "telokandaone"_n, "transfer"_n,
    std::make_tuple( get_self(), miner, reward, memo )
  ).send();
  */

  string memo;
  float token_amount;

  // Loop over all rewards
  while( rewards_itr != _rewards.end() )
  {
    // if !reward_itr->usd_denominated
    if ( rewards_itr->usd_denominated ) {
      float usd_price = delphi_itr->value / 1000.0;
      token_amount = rewards_itr->amount_per_mb / usd_price;
      memo = "Reached " + to_string(pressure) + " hpa";
    } else {
      token_amount = rewards_itr->amount_per_mb;
      memo = "";
    }

    uint32_t amt_number = (uint32_t)(pow( 10,rewards_itr->precision ) * 
                                        pressure_change * 
                                        token_amount);
    
    eosio::asset reward = eosio::asset( 
                            amt_number,
                            symbol(symbol_code( rewards_itr->symbol_letters ), rewards_itr->precision));
    
    // Do inline token transfer
    action(
      permission_level{ get_self(), "active"_n },
      name( rewards_itr->token_contract ) , "transfer"_n,
      std::make_tuple( get_self(), miner, reward, memo )
    ).send();
  }


}

ACTION wxlaunches::settwelvehr(name launch_id,
                               name miner, 
                               string wx12hrcondition) {
  // Only web app can run this Action
  require_auth(get_self());

  launches_table_t _launches(get_self(), get_self().value);  
  auto launch_itr = _launches.find(launch_id.value);

  // Add the currently stored variables to the launches table
  if ( launch_itr->miner == miner )
  {
    _launches.modify(launch_itr, get_self(), [&](auto& launch) {
      launch.wx12hrcondition = wx12hrcondition;
    });
  }

}   

// Dispatch the actions to the blockchain
EOSIO_DISPATCH(wxlaunches, (addobs)(removeobs)(setstation)(settimeslots)(setflag)(websetlaunch)(settwelvehr))