/*
* Project Cellular-MMA8452Q - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:10 January 2018
*/

/*  The idea of this release is to use the new sensor model which should work with multiple sensors
    Both utility and solar implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge
*/

//v1    - Adapted from the Particle Electron version Cellular Pressure next
//v1.01 - Added code for daylight savings time
//v1.02 - Added the currentHourlyPeriod variable
//v1.03 - Need to fix reporting that is not aligning with the hour
//v2.00 - Updated to new deivceOS and implemented checking for sysStatus
//v2.01 - Minor fixes, removed deep sleep (need to switch to RTC), controlled sysStatus.connectedStatus
//v2.02 - Working on making sleep mode work.  24 hour operations require close time to 24.
//v2.03 - Rewrote the Ubidots handler, fixed some minor errors, added time offset string
//v2.04 - Updated the PMIC stuff
//v2.05 - Fixed bug in main loop - not resetting FRAM write needed flags
//v2.06 - Moved to deviceOS@1.5.0-rc2 and updated libraries
//v2.07 - Improved time keeping
//v2.08 - Sleep 2.0
//v3.01 - Added support for battery context and the Ubidots Webhook to report it.
//v3.02 - Issue which prevents charging.
//v3.03 - Trying to revert to PMIC for Solar
//v3.10 - Updated the power management function to move back to the new setPowerConfiguration Api
//v3.11 - Minor fix to charge current for the non-solar use case
//v3.12 - Trying a lower value for solar charge 4208
//v3.13 - Added instructions to explicitly tell the device to charge
//v3.14 - Added debugging code
//v4.00 - Fixed issue where counts are not zeroed if the device does not sleep at night (zero counts at mighnight skipped reset)
//v5.00 - Took out debugging code
//v5.01 - Added code to put timestamp on hourly events, Moved to 2nd Tire Counting
//v6.00 - Moved to the Async Particle Publish Library
//v7.00 - Added PMIC.enableBuck() to Setup: https://community.particle.io/t/battery-charge-doesnt-work-if-starting-when-en-grounded/56082/19
//v8.00 - Updated the Power Config settings to improve charging efficiency.

// Particle Product definitions
PRODUCT_ID(10864);                                  // Boron Connected Counter Header
PRODUCT_VERSION(8);
#define DSTRULES isDSTusa
char currentPointRelease[5] ="8";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 2;                    // Increment this number each time the memory map is changed

struct systemStatus_structure {                     // currently 14 bytes long
  uint8_t structuresVersion;                        // Version of the data structures (system and data)
  uint8_t placeholder;                              // available for future use
  uint8_t metricUnits;                              // Status of key system states
  uint8_t connectedStatus;
  uint8_t verboseMode;
  uint8_t solarPowerMode;
  uint8_t lowPowerMode;
  uint8_t lowBatteryMode;
  int stateOfCharge;                                // Battery charge level
  uint8_t powerState;                               // Stores the current power state
  int resetCount;                                   // reset count of device (0-256)
  float timezone;                                   // Time zone value -12 to +12
  float dstOffset;                                  // How much does the DST value change?
  int openTime;                                     // Hour the park opens (0-23)
  int closeTime;                                    // Hour the park closes (0-23)
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
} sysStatus;

struct currentCounts_structure {                    // currently 10 bytes long
  int hourlyCount;                                  // In period hourly count
  int hourlyCountInFlight;                          // In flight and waiting for Ubidots to confirm
  int dailyCount;                                   // In period daily count
  unsigned long lastCountTime;                      // When did we record our last count
  int temperature;                                  // Current Temperature
  int alertCount;                                   // What is the current alert count
  int maxMinValue;                                  // Highest count in one minute in the current period
} current;

// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "MCP79410RK.h"                             // Real Time Clock
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "UnitTestCode.h"                           // This code will exercise the device
#include "PublishQueueAsyncRK.h"                    // Async Particle Publish

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MCP79410 rtc;                                       // Rickkas MCP79410 libarary
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));
PMIC pmic;

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Pin Constants - Boron Carrier Board v1.2a
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int donePin =       D5;                       // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int intPin =        SCK;                      // Pressure Sensor inerrupt pin
const int disableModule = MOSI;                     // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      MISO;                     // Allows us to control the indicator LED on the sensor board

// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
char currentOffsetStr[10];                          // What is our offset from UTC
int currentHourlyPeriod = 0;                        // Need to keep this separate from time so we know when to report

// Program Variables
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char lowPowerModeStr[6];                            // In low power mode?
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentCountsWriteNeeded = false;

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Pressure Sensor Variables
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered

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
  pinMode(donePin,OUTPUT);                          // Allows us to pet the watchdog

  // Pressure / PIR Module Pin Setup
  pinMode(intPin,INPUT_PULLDOWN);                   // pressure sensor interrupt
  pinMode(disableModule,OUTPUT);                    // Turns on the module when pulled low
  pinResetFast(disableModule);                      // Turn on the module - send high to switch off board
  pinMode(ledPower,OUTPUT);                         // Turn on the lights
  pinSetFast(ledPower);                             // Turns on the LED on the pressure sensor board

  digitalWrite(blueLED,HIGH);

  petWatchdog();                                    // Pet the watchdog - This will reset the watchdog time period
  attachInterrupt(wakeUpPin, watchdogISR, RISING);  // The watchdog timer will signal us and we have to respond

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", current.hourlyCount);                // Define my Particle variables
  Particle.variable("DailyCount", current.dailyCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("OpenTime",sysStatus.openTime);
  Particle.variable("CloseTime",sysStatus.closeTime);
  Particle.variable("Alerts",current.alertCount);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextStr);


  Particle.function("resetFRAM", resetFRAM);                          // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setverboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);

  pmic.enableBuck();                                                  // To enable charging 

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

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                    // If so, store incremented number - watchdog must have done This
  }

  (sysStatus.lowPowerMode) ? strcpy(lowPowerModeStr,"True") : strcpy(lowPowerModeStr,"False");

  rtc.setup();                                                        // Start the real time clock
  rtc.clearAlarm();                                                   // Ensures alarm is still not set from last cycle

  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits
  if (!Time.isValid()) Time.setTime(rtc.getRTCTime());
  DSTRULES() ? Time.beginDST() : Time.endDST();    // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  // Done with the System Stuff - now load the current counts
  fram.get(FRAM::currentCountsAddr,current);
  if (current.hourlyCount) currentHourlyPeriod = Time.hour(current.lastCountTime);
  else currentHourlyPeriod = Time.hour();                              // The local time hourly period for reporting purposes

  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  if (!digitalRead(userSwitch)) loadSystemDefaults();                 // Make sure the device wakes up and connects

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  
  if (Time.day() != Time.day(current.lastCountTime)) {    // ******  - These are debug lines
    resetEverything();                                               // Zero the counts for the new day
    if (sysStatus.solarPowerMode && !sysStatus.lowPowerMode) {
      setLowPowerMode("1");                                           // If we are running on solar, we will reset to lowPowerMode at Midnight
    }
  }

  if ((Time.hour() >= sysStatus.closeTime || Time.hour() < sysStatus.openTime)) {} // The park is closed - don't connect
  else {                                                              // Park is open let's get ready for the day
    attachInterrupt(intPin, sensorISR, RISING);                       // Pressure Sensor interrupt from low to high
    if (sysStatus.connectedStatus && !Particle.connected()) connectToParticle(); // Only going to connect if we are in connectionMode
    takeMeasurements();                                               // Populates values so you can read them before the hour
    stayAwake = stayAwakeLong;                                        // Keeps Electron awake after reboot - helps with recovery
  }

  pinResetFast(ledPower);                                             // Turns off the LED on the sensor board

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code

  digitalWrite(blueLED,LOW);
}

void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (watchdogFlag) petWatchdog();                                  // Watchdog flag is raised - time to pet the watchdog
    if (sensorDetect) recordCount();                                  // The ISR had raised the sensor flag
    if (current.hourlyCountInFlight) {                                // Cleared here as there could be counts coming in while "in Flight"
      current.hourlyCount -= current.hourlyCountInFlight;             // Confirmed that count was recevied - clearing
      current.hourlyCountInFlight = current.maxMinValue = current.alertCount = 0; // Zero out the counts until next reporting period
      currentCountsWriteNeeded=true;
    }
    if (systemStatusWriteNeeded) {
      fram.put(FRAM::systemStatusAddr,sysStatus);
      systemStatusWriteNeeded = false;
    }
    if (currentCountsWriteNeeded) {
      fram.put(FRAM::currentCountsAddr,current);
      currentCountsWriteNeeded = false;
    }
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;  // When in low power mode, we can nap between taps
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;  // We want to report on the hour but not after bedtime
    if ((Time.hour() >= sysStatus.closeTime || Time.hour() < sysStatus.openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    detachInterrupt(intPin);                                          // Done sensing for the day
    pinSetFast(disableModule);                                        // Turn off the pressure module for the hour
    if (current.hourlyCount) {                                        // If this number is not zero then we need to send this last count
      state = REPORTING_STATE;
      break;
    }
    if (sysStatus.connectedStatus) disconnectFromParticle();          // Disconnect cleanly from Particle
    digitalWrite(blueLED,LOW);                                        // Turn off the LED
    petWatchdog();
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    rtc.setAlarm(wakeInSeconds);                                // The Real Time Clock will turn the Enable pin back on to wake the device
    } break;

  case NAPPING_STATE: {  // This state puts the device in low power mode quickly
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (sensorDetect) break;                                          // Don't nap until we are done with event
    if (sysStatus.connectedStatus) disconnectFromParticle();          // If we are in connected mode we need to Disconnect from Particle
    stayAwake = 1000;                                   // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    petWatchdog();                                                    // Reset the watchdog timer interval
    config.mode(SystemSleepMode::STOP).gpio(userSwitch,CHANGE).gpio(intPin,RISING).duration(wakeInSeconds * 1000).flag(SystemSleepFlag::WAIT_CLOUD);
    SystemSleepResult result = System.sleep(config);                    // Put the device to sleep
    if (result.wakeupPin() == intPin) {                                               // Executions starts here after sleep - time or sensor interrupt?
      stayAwakeTimeStamp = millis();
    }
    else if (result.wakeupPin() == userSwitch) setLowPowerMode("0");
    state = IDLE_STATE;                                               // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    } break;

  case REPORTING_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!sysStatus.connectedStatus) connectToParticle();              // Only attempt to connect if not already New process to get connected
    if (Particle.connected()) {
      if (Time.hour() == sysStatus.openTime) dailyCleanup();          // Once a day, clean house
      takeMeasurements();                                             // Update Temp, Battery and Signal Strength values
      sendEvent();                                                    // Send data to Ubidots
      state = RESP_WAIT_STATE;                                        // Wait for Response
    }
    else {
      resetTimeStamp = millis();
      state = ERROR_STATE;
    }
  
    if (sysStatus.lowPowerMode) Time.setTime(rtc.getRTCTime());
    break;

  case RESP_WAIT_STATE:
    if (sysStatus.verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)  {                                             // Response received back to IDLE state
      stayAwake = stayAwakeLong;                                      // Keeps Electron awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
      if (Time.hour() == 0) resetEverything();                        // It is a new day.  Zero everything so we can start fresh
      state = IDLE_STATE;
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
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
        systemStatusWriteNeeded=true;
        rtc.setAlarm(10);
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) publishQueue.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        systemStatusWriteNeeded=true;
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
  rtc.loop();                                                         // keeps the clock up to date
  //sensorDetect = steadyCountTest();                                     // Comment out to cause the device to run through a series of tests
  
}

void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
  static byte currentMinutePeriod;                                    // Current minute

  pinSetFast(blueLED);                                                // Turn on the blue LED

  if (currentMinutePeriod != Time.minute()) {                       // Done counting for the last minute
    currentMinutePeriod = Time.minute();                            // Reset period
    current.maxMinValue = 1;                                        // Reset for the new minute
  }
  current.maxMinValue++;

  current.lastCountTime = Time.now();
  current.hourlyCount++;                                                // Increment the PersonCount
  current.dailyCount++;                                                 // Increment the PersonCount
  if (sysStatus.verboseMode && Particle.connected()) {
    char data[256];                                                    // Store the date in this character array - not global
    snprintf(data, sizeof(data), "Count, hourly: %i, daily: %i",current.hourlyCount,current.dailyCount);
    publishQueue.publish("Count",data, PRIVATE);                           // Helpful for monitoring and calibration
  }

  currentCountsWriteNeeded = true;                                    // Write updated values to FRAM
  pinResetFast(blueLED);                                              // Turn off the blue LED
  sensorDetect = false;                                               // Reset the flag
}


void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  unsigned long timeStampValue = Time.now();                          // Going to start sending timestamps - and will modify for midnight to fix reporting issue
  int secondsPastHour = timeStampValue % 3600;
  timeStampValue = timeStampValue - (secondsPastHour + 1);            // This ensures that the midnight report is back-dated 5 minutes 
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i,  \"key1\":\"%s\", \"temp\":%i, \"resets\":%i, \"alerts\":%i, \"maxmin\":%i, \"timestamp\":%lu000}",current.hourlyCount, current.dailyCount, sysStatus.stateOfCharge, batteryContextStr, current.temperature, sysStatus.resetCount, current.alertCount, current.maxMinValue, timeStampValue);
  publishQueue.publish("Ubidots-Counter-Hook-v1", data, PRIVATE);
  dataInFlight = true;                                                // set the data inflight flag
  webhookTimeStamp = millis();
  currentHourlyPeriod = Time.hour();
  current.hourlyCountInFlight = current.hourlyCount;                  // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
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
  publishQueue.publish("Ubidots Hook", responseString, PRIVATE);
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready
  getTemperature();                                                   // Get Temperature at startup as well
  getBatteryContext();                                                // What is the battery up to?
  //sysStatus.stateOfCharge = int(batteryMonitor.getSoC());
  sysStatus.stateOfCharge = int(System.batteryCharge());             // Percentage of full charge
  systemStatusWriteNeeded=true;
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
  const char* batteryContext[7] ={"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};
  // Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-

  snprintf(batteryContextStr, sizeof(batteryContextStr),"%s", batteryContext[System.batteryState()]);

}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  currentCountsWriteNeeded=true;
  return current.temperature;
}

// Here are the various hardware and timer interrupt service routines
void sensorISR()
{
  static bool frontTireFlag = false;
  if (frontTireFlag) {
    sensorDetect = true;                              // sets the sensor flag for the main loop
    frontTireFlag = false;
  }
  else frontTireFlag = true;
}

void watchdogISR()
{
  watchdogFlag = true;
}

void petWatchdog()
{
  digitalWriteFast(donePin, HIGH);                                        // Pet the watchdog
  digitalWriteFast(donePin, LOW);
  watchdogFlag = false;
}


// Power Management function
int setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration
  if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(900) // Set maximum current the power source can provide (applies only when powered through VIN)
        .powerSourceMinVoltage(5080) // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(1024) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    enableCharging(true);
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA 
        .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
        .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    enableCharging(true);
    return res;
  }
}

bool enableCharging(bool enableCharge)
{
  PMIC pmic(true);
  if(enableCharge) {
    pmic.enableCharging();
    return TRUE;
  }
  else {
    pmic.disableCharging();
    return FALSE;
  }
}

void loadSystemDefaults() {                                         // Default settings for the device - connected, not-low power and always on
  connectToParticle();                                              // Get connected to Particle - sets sysStatus.connectedStatus to true
  takeMeasurements();                                               // Need information to set value here - sets sysStatus.stateOfCharge
  if (Particle.connected()) publishQueue.publish("Mode","Loading System Defaults", PRIVATE);
  sysStatus.structuresVersion = 1;
  sysStatus.metricUnits = false;
  sysStatus.verboseMode = true;
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;
  else sysStatus.lowBatteryMode = false;
  setLowPowerMode("0");
  sysStatus.timezone = -5;                                          // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.openTime = 0;
  sysStatus.closeTime = 24;

  fram.put(FRAM::systemStatusAddr,sysStatus);                       // Write it now since this is a big deal and I don't want values over written
}

void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range
  takeMeasurements();                                               // Sets the sysStatus.stateOfCharge
  if (sysStatus.metricUnits < 0 || sysStatus.metricUnits >1) sysStatus.metricUnits = 0;
  if (sysStatus.connectedStatus < 0 || sysStatus.connectedStatus > 1) {
    if (Particle.connected()) sysStatus.connectedStatus = true;
    else sysStatus.connectedStatus = false;
  }
  if (sysStatus.verboseMode < 0 || sysStatus.verboseMode > 1) sysStatus.verboseMode = false;
  if (sysStatus.solarPowerMode < 0 || sysStatus.solarPowerMode >1) sysStatus.solarPowerMode = 0;
  if (sysStatus.lowPowerMode < 0 || sysStatus.lowPowerMode > 1) setLowPowerMode("1");
  if (sysStatus.lowBatteryMode < 0 || sysStatus.lowBatteryMode > 1) sysStatus.lowBatteryMode = 0;
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;
  else sysStatus.lowBatteryMode = false;
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) sysStatus.openTime = 0;
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;
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
    if(sensorDetect) recordCount(); // service the interrupt every 10 seconds
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

int resetCounts(String command)                                       // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    current.dailyCount = 0;                                           // Reset Daily Count in memory
    current.hourlyCount = 0;                                          // Reset Hourly Count in memory
    sysStatus.resetCount = 0;                                            // If so, store incremented number - watchdog must have done This
    current.alertCount = 0;                                           // Reset count variables
    current.hourlyCountInFlight = 0;                                  // In the off-chance there is data in flight
    dataInFlight = false;
    currentCountsWriteNeeded = true;                                  // Make sure we write to FRAM back in the main loop
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    publishQueue.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
    rtc.setAlarm(10);
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

void resetEverything() {                                            // The device is waking up in a new day or is a new install
  current.dailyCount = 0;                              // Reset the counts in FRAM as well
  current.hourlyCount = 0;
  current.hourlyCountInFlight = 0;
  current.lastCountTime = Time.now();                      // Set the time context to the new day
  sysStatus.resetCount = current.alertCount = 0;           // Reset everything for the day
  currentCountsWriteNeeded=true;
  systemStatusWriteNeeded=true;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig();                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (Particle.connected()) publishQueue.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig();                                                // Change the power management settings
    if (Particle.connected()) publishQueue.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
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

int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                                    // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                            // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  systemStatusWriteNeeded = true;                                            // Need to store to FRAM back in the main loop
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
    publishQueue.publish("Time",data, PRIVATE);
  }
  return 1;
}

int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  if (Particle.connected()) publishQueue.publish("Time",data, PRIVATE);
  return 1;
}

int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (Particle.connected()) {
      publishQueue.publish("Mode","Low Power Mode", PRIVATE);
    }
    sysStatus.lowPowerMode = true;
    strcpy(lowPowerModeStr,"True");
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (!Particle.connected()) {                                      // In case we are not connected, we will do so now.
      connectToParticle();
      sysStatus.connectedStatus = true;
    }
    publishQueue.publish("Mode","Normal Operations", PRIVATE);
    delay(1000);                                                      // Need to make sure the message gets out.
    sysStatus.lowPowerMode = false;                                   // update the variable used for console status
    strcpy(lowPowerModeStr,"False");                                  // Use capitalization so we know that we set this.
  }
  systemStatusWriteNeeded = true;
  return 1;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) publishQueue.publish("State Transition",stateTransitionString, PRIVATE);
  Serial.println(stateTransitionString);
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
  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 70) {    // If Solar or if the battery is being discharged
    setLowPowerMode("1");
  }
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

bool isDSTnz() {
  // New Zealand Summer Timer calculation (2am Local Time - last Sunday in September/ 1st Sunday in April)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window - 10 out of 12 months with April and Septemper in question
  if (month >= 10 || month <= 3)
  { // October to March is definetly DST - 6 months
    return true;
  }
  else if (month < 9 && month > 4)
  { // before September and after April is definetly standard time - - 4 months
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 6);
  boolean lastSundayOrAfter = (dayOfMonth - dayOfWeek > 23);

  if (beforeFirstSunday && !lastSundayOrAfter) return (month == 4);
  else if (!beforeFirstSunday && !lastSundayOrAfter) return false;
  else if (!beforeFirstSunday && lastSundayOrAfter) return (month == 9);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time
    return !dayStartedAs;
  }
  return dayStartedAs;
}
