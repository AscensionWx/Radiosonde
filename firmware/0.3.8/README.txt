Updates:
Send every 3 seconds for first 150 meters, 5 seconds to 850, 10 seconds to 700, 20 seconds afterwards
checkIfLaunch() function checks if pressure has fallen significantly in 2 consecutive readings
Call checkIfLaunch every 3 seconds while on the ground
Set weather variables as globals
Include -1.0 temperature offset
Apply additional offset based on lux (divide by 9000)
Store 2 minutes of lux
Improve sensor reading handling ... make all global and separate read() functions
Change GPS tx/rx pins to 1 and 3
Submit lat/lon elevation if data ONLY if it's not stale
Light sleep between messages
Set timezone on esp32 and use it when sending observations
We are firmware version 0.3.8, so use ports 81, 82, and 83
Include pressure with status messages that send every 30 seconds
Add unixtime in obs message (since we have GPS and need it for wind)
Add lux avg variable in obs message
Turn wifi off after 2 minutes if no smartphone connected
OLED displays "Turning off screen" for 10 seconds and then turns off
After wifi turn off, sleep between 3-second BME readings and 30-second status msgs
Add time limit check "Time limit reached. Please restart" and sleep



To Do:
Increase power and use SF7 with each transmit e.g. LMIC_setDrTxpow(EU868_DR_SF7, 21)


