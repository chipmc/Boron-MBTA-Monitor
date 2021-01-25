/*
* Project: Boron-MBTA-Montitor
* Description: Boron Temperature for MBTA Trains
* Author: Chip McClelland
* Date: 8-11-2020
*/

/*  This is a first pass minimally vialble code for a prototype device - iterations below
*/

//v1 -  Just seeing if I can talk to the temperature sensor and GPS sensor to work
//v2 -  First demo release - works and sends data to Ubidots via Webhook 
//v2.01 - Fix for doulbe send
//v2.02 - Update for WebHook and Moved to degrees F
//v2.03 - Shortened Webhook name and fixed temp labeling
//v2.04 - Updated to make it more clear what the sample interval is
//v3.00 - Added logic for low power battery mode
//v4.00 - Implemented fix for device not coming out of lowBattery mode
//v4.01 - Updated with the Boron not charging fix
//v5.00 - Fixed the timeout on the battery context and back to 4 hours
//v6.00 - Updated to the new deviceOS@2.0.0-rc2 - removed old PMIC fix
//v7.00 - Minor fixes
//v8.00 - updated to deviceOS@2.0.0-rc3 and fixed an ERROR_STATE issue
//v9.00 - Update to deviceOS@2.0.0
//v10.00 - Update to new carrier board v1.5 and deviceOS@2.0.1
//v11.00 - Fixed formatting of GPS and slowed our roll a bit to report occaisional double reporting
//v12.00 - Improved time keeping for battery power / vehicle power - added recovery from sleeping using user switch as long as battery is over 50% - added out of memory reset


// Particle Product definitions
PRODUCT_ID(11743);                                  // Boron Connected Counter Header
PRODUCT_VERSION (12);
#define DSTRULES isDSTusa
char currentPointRelease[8] ="12.03";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const byte FRAMversionNumber = 2;                    // Increment this number each time the memory map is changed

struct systemStatus_structure {                     // currently 14 bytes long
  uint8_t structuresVersion;                        // Version of the data structures (system and data)
  uint8_t placeholder;                              // available for future use
  uint8_t clockSet;                                 // Tells us if we need to connect and set the RTC
  uint8_t connectedStatus;
  uint8_t verboseMode;
  uint8_t lowBatteryMode;
  uint8_t sampleIntervalMin;
  int stateOfCharge;                                // Battery charge level
  uint8_t powerState;                               // Stores the current power state
  int resetCount;                                   // reset count of device (0-256)
  float timezone;                                   // Time zone value -12 to +12
  float dstOffset;                                  // How much does the DST value change?
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
  unsigned long lastTimePowered;                    // The last time we were on vehicle power (used to conserve battery during out of service events)
} sysStatus;

struct currentCounts_structure {                    // currently 10 bytes long
  double tempArray[3];
  int alertCount;
  double latitude;
  double longitude;
} current;

// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "PublishQueueAsyncRK.h"                    // Async Particle Publish
#include "DS18B20.h"                               // Include the DS18B20 Library
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "AssetTrackerRK.h" // only used for AssetTracker::antennaExternal

// Port of TinyGPS for the Particle AssetTracker
// https://github.com/mikalhart/TinyGPSPlus
#include "TinyGPS++.h"


// This is the maximum amount of time to allow for connecting to cloud. If this time is
// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
const std::chrono::milliseconds connectMaxTime = 11min;


// Pin Constants - Boron Carrier Board v1.2a
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor

// Pin Constants - Sensors
const int tempSensors =   A3;                      // PThree on-wire temp sensors on this pin (pulled up to VCC via 10k)

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));
DS18B20 ds18b20(tempSensors);
AssetTracker gps;
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library

// GPS Variables
const unsigned long PUBLISH_PERIOD = 120000;
const unsigned long SERIAL_PERIOD = 5000;
unsigned long startFix = 0;
bool gettingFix = false;
unsigned long lastPublish = 0;

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, SLEEPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Measuring", "Sleeping", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;


// Timing Variables
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
char currentOffsetStr[10];                          // What is our offset from UTC
int currentHourlyPeriod = 0;                        // Need to keep this separate from time so we know when to report
int currentMinutePeriod = 0;                        // Makes sure we don't go into reporting too many times


// Program Variables
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char cabinTempStr[12] = "Null";                     // Temperature in the cabin
char ventTempStr[12] = "Null";                      // Temperature of the air coming out of the AC vent
char outsideTempStr[12] = "Null";                   // Outdoor air temperature
char sampleIntervalStr[12] = "Null";                // Test for sample Interval
char lastPoweredStr[32]="Null";                     // When did we last see vehicle power (starts a 4 hour clock)
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentCountsWriteNeeded = false;
bool dataInFlight = false;

// These variables are associated with the watchdog timer and will need to be better integrated
int outOfMemory = -1;
time_t RTCTime;


// Sensor Variables
const int nSENSORS = 3;
float celsius[nSENSORS] = {NAN, NAN};
retained uint8_t sensorAddresses[nSENSORS][8];
const int MAXRETRY = 3;

void setup()                                        // Note: Disconnected Setup()
{
  /* Setup is run for three reasons once we deploy a sensor:
       1) When you deploy the sensor
       2) Each hour while the device is sleeping
       3) After a reset event
    All three of these have some common code - this will go first then we will set a conditional
    to determine which of the three we are in and finish the code
  */
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output

  digitalWrite(blueLED,HIGH);

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("Signal", SignalString);
  Particle.variable("SampleInterval",sampleIntervalStr);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("CabinTemp",cabinTempStr);
  Particle.variable("VentTemp",ventTempStr);
  Particle.variable("OutsideTemp",outsideTempStr);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("Alerts",current.alertCount);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextStr);
  Particle.variable("Last Powered",lastPoweredStr);

  Particle.function("resetFRAM", resetFRAM);                          // These are the functions exposed to the mobile app and console
  Particle.function("SendNow",sendNow);
  Particle.function("Verbose-Mode",setverboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("SampleInterval",setSampleInterval);

  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly

  connectToParticle();

  // Load FRAM and reset variables to their correct values
  fram.begin();                                                       // Initialize the FRAM module

  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                             // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                     // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                   // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                         // See if this worked
    if (tempVersion != FRAMversionNumber) state = ERROR_STATE;        // Device will not work without FRAM
    else loadSystemDefaults();                                        // Out of the box, we need the device to be awake and connected
  }
  else fram.get(FRAM::systemStatusAddr,sysStatus);                    // Loads the System Status array from FRAM

  checkSystemValues();                                                // Make sure System values are all in valid range

  // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.
  System.on(out_of_memory, outOfMemoryHandler);

  // The carrier board has D8 connected to FOUT for wake interrupts
  ab1805.withFOUT(D8).setup();

  // Note whether the RTC is set 
  sysStatus.clockSet =  ab1805.isRTCSet();

  // Enable watchdog
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);

  snprintf(sampleIntervalStr, sizeof(sampleIntervalStr),"%i minutes", sysStatus.sampleIntervalMin);

  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits
  
  if (!Time.isValid() && sysStatus.clockSet) {                         // We need to try to get the time so we can tell if we need DST
    ab1805.getRtcAsTime(RTCTime);
    Time.setTime(RTCTime);
  }
  else {                                                               // Special case, neither the RTC or the system clock is set, we need to connect and get the time
    connectToParticle();
    Particle.syncTime();                                               // Set the system clock here
  }

  DSTRULES() ? Time.beginDST() : Time.endDST();    // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                    // If so, store incremented number - watchdog must have done This
  }

  snprintf(lastPoweredStr, sizeof(lastPoweredStr), Time.timeStr(sysStatus.lastTimePowered));   // Load the last power event

  getBatteryContext();                                                // See if we have enough juice

  // Done with the System Stuff - now load the current counts
  fram.get(FRAM::currentCountsAddr,current);
  currentHourlyPeriod = Time.hour();                                   // The local time hourly period for reporting purposes

  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  if (!digitalRead(userSwitch)) loadSystemDefaults();                  // Make sure the device wakes up and connects

  ds18b20.resetsearch();                 // initialise for sensor search
  for (int i = 0; i < nSENSORS; i++) {   // try to read the sensor addresses
    ds18b20.search(sensorAddresses[i]); // and if available store
  }

	// Enable I2C mode
	gps.withI2C();

	// Run in threaded mode - this eliminates the need to read Serial1 from loop or updateGPS() and dramatically
	// lowers the risk of lost or corrupted GPS data caused by blocking loop for too long and overflowing the
	// 64-byte serial buffer.
	gps.startThreadedMode();

  // Here is where the code diverges based on why we are running Setup()
  publishQueue.publish("State","Startup Complete",PRIVATE);
  
  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code

  digitalWrite(blueLED,LOW);
}

void loop()
{
  switch(state) {
  case IDLE_STATE: {                                                   // Where we spend most time - note, the order of these conditionals is important
      static unsigned long lastPoll = 0;
      if (sysStatus.verboseMode && state != oldState) publishStateTransition();
      if (sysStatus.lowBatteryMode) state = SLEEPING_STATE;
      if (millis() - lastPoll > 1000) {                               // Slow down the polling a bit - less traffic on i2c bus and no need to check the clock more than once a second
        if ((Time.minute() % sysStatus.sampleIntervalMin == 0) && (Time.minute() != currentMinutePeriod)) state = MEASURING_STATE;   // sub hourly interval
        else if ((Time.minute() == 0) && (Time.minute() != currentMinutePeriod)) state = MEASURING_STATE;           //  on hourly interval
        lastPoll = millis();
      }
    } break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    getBatteryContext();                                              // Check to make sure we should still be in the low battery state
    if (!sysStatus.lowBatteryMode) {                                  // If not, we need to exit this state and go back to IDLE_STATE
      state = IDLE_STATE;
      break;
    }
    else if (Time.now() - sysStatus.lastHookResponse > 7200L) {       // Troubleshooting code
      connectToParticle();
      Particle.publish("State","Stuck in sleep state",PRIVATE);
      state = REPORTING_STATE;
      break;
    }
    if (Time.minute() > 1 && sysStatus.connectedStatus) disconnectFromParticle();          // Disconnect cleanly from Particle after the first minute
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      ab1805.stopWDT();                                                 // No watchdogs interrupting our slumber
      int wakeInSeconds = constrain((60 - Time.minute()) * 60, 1, 60 * 60);   // Sleep till the top of the hour
      config.mode(SystemSleepMode::ULTRA_LOW_POWER)
        .gpio(userSwitch,CHANGE)
        .duration(wakeInSeconds * 1000);
      SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device reboots from here   
      ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
      if (result.wakeupPin() == userSwitch) sysStatus.lastTimePowered = Time.now(); // Gives you a chance to bring back a device (should reset lowBatteryMode in getBatteryConext() and put system back to IDLE_STATE)
    } break;

  case MEASURING_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    takeMeasurements();                                             // Update Temp, Battery and Signal Strength values
    state = REPORTING_STATE;
    break;

  case REPORTING_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!sysStatus.connectedStatus) connectToParticle();              // Only attempt to connect if not already New process to get connected
    if (!Particle.connected()) connectToParticle();                   // Just to make sure
    if (Particle.connected()) {
      if (Time.hour() == 0) dailyCleanup();                           // Once a day, clean house
      sendEvent();                                                    // Send data to Ubidots
      state = RESP_WAIT_STATE;                                        // Wait for Response
    }
    else {
      resetTimeStamp = millis();
      state = ERROR_STATE;
    }
    break;

  case RESP_WAIT_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight && Time.minute() != currentMinutePeriod)  {     // Response received back to IDLE state and wait till the minute has passed to prevent double reporting
      state = IDLE_STATE;
    }
    else if (dataInFlight && millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      publishQueue.publish("spark/device/session/end", "", PRIVATE);      // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {
      if (sysStatus.resetCount <= 3) {                                          // First try simple reset
        if (Particle.connected()) publishQueue.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - sysStatus.lastHookResponse > 7200L) { //It has been more than two hours since a sucessful hook response
        if (Particle.connected()) publishQueue.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                  // Zero the ResetCount
        fram.put(FRAM::systemStatusAddr,sysStatus);
        ab1805.deepPowerDown(10);
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) publishQueue.publish("State","Error State - Power Cycle", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        fram.put(FRAM::systemStatusAddr,sysStatus);
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }

  ab1805.loop();                                                      // Keeps the RTC synchronized with the Boron's clock

  if (systemStatusWriteNeeded) {
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentCountsWriteNeeded) {
    fram.put(FRAM::currentCountsAddr,current);
    currentCountsWriteNeeded = false;
  }
  if (outOfMemory >= 0) {
    char message[64];
    // An out of memory condition occurred - reset device.
    snprintf(message, sizeof(message), "Out of memory occurred size=%d",outOfMemory);
    Log.info(message);
    delay(100);
    publishQueue.publish("Memory",message,PRIVATE);
    delay(2000);
    System.reset();
  }
}


void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"cabinT\":%4.2f, \"ventT\":%4.2f, \"outsideT\":%4.2f, \"battery\":%i,  \"key1\":\"%s\", \"resets\":%i, \"alerts\":%i, \"timestamp\":%lu000, \"lat\":%f, \"lng\":%f}",current.tempArray[0], current.tempArray[1], current.tempArray[2],sysStatus.stateOfCharge, batteryContextStr, sysStatus.resetCount, current.alertCount, Time.now(), current.latitude, current.longitude);
  publishQueue.publish("Ubidots-MBTA-Hook-v2-Parse", data, PRIVATE);
  dataInFlight = true;                                                // set the data inflight flag
  webhookTimeStamp = millis();
  currentHourlyPeriod = Time.hour();
  currentMinutePeriod = Time.minute();
}

void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  if (sysStatus.verboseMode) publishQueue.publish("Ubidots Hook", responseString, PRIVATE);
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  displayInfo();

  if (Cellular.ready()) getSignalStrength();                        // Test signal strength if the cellular modem is on and ready
  for (int i = 0; i < nSENSORS; i++) {
    float temp = getTemp(sensorAddresses[i]);
    if (!isnan(temp)) current.tempArray[i] = temp;
  }
  snprintf(cabinTempStr, sizeof(cabinTempStr),"%4.2f F", current.tempArray[0]);
  snprintf(ventTempStr, sizeof(ventTempStr),"%4.2f F", current.tempArray[1]);
  snprintf(outsideTempStr, sizeof(outsideTempStr),"%4.2f F", current.tempArray[2]);
  getBatteryContext();                                               // What is the battery up to?
  systemStatusWriteNeeded=true;
  currentCountsWriteNeeded=true;
}

void displayInfo()
{
  static unsigned long lastSerial = 0;

	if (millis() - lastSerial >= 1000) {                                      // Don't read more than once a second
		lastSerial = millis();

		char buf[128];

		if (gps.gpsFix()) {
      current.latitude = gps.readLatDeg();
      current.longitude = gps.readLonDeg();
			snprintf(buf, sizeof(buf), "location: %f, %f altitude: %4.1fm %d sattelites", current.latitude, current.longitude, gps.getAltitude(), gps.getSatellites());
		}
		else {
			snprintf(buf, sizeof(buf), "no location satellites:%d", gps.getSatellites());
		}

		Log.info(buf);
		if (sysStatus.verboseMode) publishQueue.publish("gps", buf, PRIVATE);
	}
}

double getTemp(uint8_t addr[8]) {
  double _temp;
  int   i = 0;

  do {
    _temp = ds18b20.getTemperature(addr);
  } while (!ds18b20.crcCheck() && MAXRETRY > i++);

  if (i < MAXRETRY) {
    _temp = ds18b20.convertToFahrenheit(_temp);
  }
  else _temp = NAN;

  return _temp;
}

void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

void getBatteryContext() {
  static bool alreadyOnBattery = false;                                           // Wee need to watch how long we are on battery power

  sysStatus.stateOfCharge = int(System.batteryCharge());                          // Percentage of full charge

  const char* batteryContext[7] ={"Unknown","Not Charging","Charging","On Vehicle Pwr","Off Vehicle Pwr","Fault","Diconnected"};
  // Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-

  snprintf(batteryContextStr, sizeof(batteryContextStr),"%s", batteryContext[System.batteryState()]);

  if (!alreadyOnBattery && System.batteryState() == 4) {                          // Keep track how long we are on battery power
    alreadyOnBattery = true;
  }
  else if (System.batteryState() == 2 || System.batteryState() == 3) {            // If charged or charging
    alreadyOnBattery = false;
    sysStatus.lastTimePowered = Time.now();
    snprintf(lastPoweredStr, sizeof(lastPoweredStr), Time.timeStr());
  }

  if (Time.now() - sysStatus.lastTimePowered > 4 * 60 * 60 || sysStatus.stateOfCharge <= 50) {    // If we have been on battery for four hours, or the battery is less than 50%
    sysStatus.lowBatteryMode = true;
  } 
  else sysStatus.lowBatteryMode = false;

}


// Power Management function
int setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration
  conf.powerSourceMaxCurrent(900)                                   // default is 900mA 
      .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
      .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
      .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
      .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
  int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
  return res;
}

void loadSystemDefaults() {                                         // Default settings for the device - connected, not-low power and always on
  connectToParticle();                                              // Get connected to Particle - sets sysStatus.connectedStatus to true
  takeMeasurements();                                               // Need information to set value here - sets sysStatus.stateOfCharge
  if (Particle.connected()) publishQueue.publish("Mode","Loading System Defaults", PRIVATE);
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = true;
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;
  else sysStatus.lowBatteryMode = false;
  sysStatus.sampleIntervalMin = 10;                                 // Default reading every 10 minutes
  sysStatus.timezone = -5;                                          // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.lastHookResponse = 0;
  sysStatus.lastTimePowered = Time.now();                           // Make sure we do not go to sleep right away
  fram.put(FRAM::systemStatusAddr,sysStatus);                       // Write it now since this is a big deal and I don't want values over written
}

void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range
  takeMeasurements();                                               // Sets the sysStatus.stateOfCharge
  if (sysStatus.connectedStatus < 0 || sysStatus.connectedStatus > 1) {
    if (Particle.connected()) sysStatus.connectedStatus = true;
    else sysStatus.connectedStatus = false;
  }
  if (sysStatus.verboseMode < 0 || sysStatus.verboseMode > 1) sysStatus.verboseMode = false;
  if (sysStatus.lowBatteryMode < 0 || sysStatus.lowBatteryMode > 1) sysStatus.lowBatteryMode = 0;
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;
  else sysStatus.lowBatteryMode = false;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (Time.now() < sysStatus.lastTimePowered) sysStatus.lastTimePowered = 0;          // Can't go back in time
  sysStatus.sampleIntervalMin = 10;                                 // Default reading every 10 minutes
  // None for lastHookResponse

  systemStatusWriteNeeded = true;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {

    Particle.process();
  }
  if (Particle.connected()) {
    sysStatus.connectedStatus = true;
    systemStatusWriteNeeded = true;
    return 1;                               // Were able to connect successfully
  }
  else {
    return 0;                                                    // Failed to connect
  }
}

bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
{
  Particle.disconnect();
  waitFor(notConnected, 15000);                                   // make sure before turning off the cellular modem
  Cellular.off();
  sysStatus.connectedStatus = false;
  systemStatusWriteNeeded = true;
  delay(2000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                             // Companion function for disconnectFromParticle
  return !Particle.connected();
}

int resetFRAM(String command)                                     // Will reset the local counts
{
  if (command == "1")
  {
    fram.erase();
    return 1;
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = MEASURING_STATE;
    return 1;
  }
  else return 0;
}

void resetEverything() {                                            // The device is waking up in a new day or is a new install
  sysStatus.resetCount = current.alertCount = 0;           // Reset everything for the day
  currentCountsWriteNeeded=true;
  systemStatusWriteNeeded=true;
}

int setverboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) publishQueue.publish("Mode","Set Verbose Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) publishQueue.publish("Mode","Cleared Verbose Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  Particle.syncTime();                                                        // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                       // Wait for up to 30 seconds for the SyncTime to complete
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.timezone = (float)tempTimeZoneOffset;
  Time.zone(sysStatus.timezone);
  systemStatusWriteNeeded = true;                                             // Need to store to FRAM back in the main loop
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
    publishQueue.publish("Time",data, PRIVATE);
    publishQueue.publish("Time",Time.timeStr(Time.now()), PRIVATE);
  }

  return 1;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) publishQueue.publish("State Transition",stateTransitionString, PRIVATE);
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=15\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

void dailyCleanup() {                                                 // Called from Reporting State ONLY - clean house at the end of the day
  publishQueue.publish("Daily Cleanup","Running", PRIVATE);               // Make sure this is being run
  sysStatus.verboseMode = false;
  Particle.syncTime();                                                // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                               // Wait for up to 30 seconds for the SyncTime to complete
  sysStatus.resetCount = 0;
  systemStatusWriteNeeded=true;
}

int setDSTOffset(String command) {                                      // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempDSTOffset = strtol(command,&pEND,10);                      // Looks for the first integer and interprets it
  if ((tempDSTOffset < 0) | (tempDSTOffset > 2)) return 0;              // Make sure it falls in a valid range or send a "fail" result
  Time.setDSTOffset((float)tempDSTOffset);                              // Set the DST Offset
  sysStatus.dstOffset = (float)tempDSTOffset;
  systemStatusWriteNeeded = true;
  snprintf(data, sizeof(data), "DST offset %2.1f",sysStatus.dstOffset);
  if (Time.isValid()) isDSTusa() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) {
    publishQueue.publish("Time",data, PRIVATE);
    publishQueue.publish("Time",Time.timeStr(t), PRIVATE);
  }
  return 1;
}

int setSampleInterval(String command) {                                      // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2
  char * pEND;
  char data[256];
  int8_t tempSampleInterval = strtol(command,&pEND,10);                      // Looks for the first integer and interprets it
  if ((tempSampleInterval <= 0) | (tempSampleInterval > 60)) return 0;              // Make sure it falls in a valid range or send a "fail" result
  sysStatus.sampleIntervalMin = tempSampleInterval;
  systemStatusWriteNeeded = true;
  snprintf(sampleIntervalStr, sizeof(sampleIntervalStr),"%i minutes", sysStatus.sampleIntervalMin);
  snprintf(data, sizeof(data), "Sample Interval is now %i minutes",sysStatus.sampleIntervalMin);
  if (Particle.connected()) {
    publishQueue.publish("Interval", data, PRIVATE);
  }
  return 1;
}

// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
  outOfMemory = param;
}


bool isDSTusa() {
  // United States of America Summer Timer calculation (2am Local Time - 2nd Sunday in March/ 1st Sunday in November)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window
  if (month >= 4 && month <= 10)
  { // April to October definetly DST
    return true;
  }
  else if (month < 3 || month > 11)
  { // before March or after October is definetly standard time
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 0);
  boolean secondSundayOrAfter = (dayOfMonth - dayOfWeek > 7);

  if (beforeFirstSunday && !secondSundayOrAfter) return (month == 11);
  else if (!beforeFirstSunday && !secondSundayOrAfter) return false;
  else if (!beforeFirstSunday && secondSundayOrAfter) return (month == 3);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time
    return !dayStartedAs;
  }
  return dayStartedAs;
}
