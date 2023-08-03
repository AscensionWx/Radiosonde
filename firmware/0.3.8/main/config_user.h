
// -----------------------------------------------------------------------------
// User Configuration
// -----------------------------------------------------------------------------

//#define DEBUG_MODE 1  // Begins flight observations 2 minutes after WiFi  //NICK
//#define IGNORE_GATEWAY_CHECK 1 // NOTE: Helium and other OTAA should ignore gateway check regardless
//#define IGNORE_TPRH_CHECK 1 // Uncomment to allow no I2C solder for T,P, RH

// Choose an option
//#define HELIUM 1
//#define TTN 1

#define USE_OTAA 1

#define MSG_INTERVAL_150M          10000 // Observation interval for first 150 meters

#define MSG_INTERVAL_950           5000 // hpa
#define MSG_INTERVAL_900           5000      
#define MSG_INTERVAL_800           10000     
#define MSG_INTERVAL_700           10000     
#define MSG_INTERVAL_600           20000
#define MSG_INTERVAL_500           20000
#define MSG_INTERVAL_400           20000
#define MSG_INTERVAL_300           20000
#define MSG_INTERVAL_200           20000

// If using a single-channel gateway, uncomment this next option and set to your gateway's channel
//#define SINGLE_CHANNEL_GATEWAY  0

#define FORCE_IF_LAUNCHED_HPA   960   // Forcefully set if_launched to true if this level is reached

#define MESSAGE_TO_SLEEP_DELAY  5000        // Time after message before going to sleep
#define LOGO_DELAY              5000        // Time to show logo on first boot
#define QRCODE_DELAY            10000        // Time to show logo on first boot
#define OBSERVATION_PORT        81          // Port the obs messages will be sent to
#define STATUS_PORT             82          // Port the status messages will be sent to
#define AUTHENTICATION_PORT     83          // Port for authentication message
#define LORAWAN_CONFIRMED_EVERY 0           // Send confirmed message every these many messages (0 means never)
#define LORAWAN_SF              DR_SF10     // Spreading factor
#define LORAWAN_ADR             0           // Enable ADR

#define MISSING_UINT            0
#define MISSING_FLOAT           0

//Wifi defines
const char* SSID     = "AscensionWx";
const char* PASSWORD = "balloons";

// Define here for later implementation
void debug_print(String name, float val );
