//Libraries
#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include "HX711.h"
#include <string.h>

//Defining pin arrays
const int floatMinPins[4] = {24, 26, 27, 8};
const int floatMaxPins[4] = {25, 28, 29, 9};
const int pumpPins[4] = {3, 4, 5, 6};

const int SD_CS_PIN = 53;
const int LOADCELL_DOUT_PIN = 22;
const int LOADCELL_SCK_PIN = 23;
const int ERROR_LED_PIN = 40;   //LED for flagging SD/RTC failures

//Relay polarity - board is active LOW
const int PUMP_ON  = LOW;
const int PUMP_OFF = HIGH;

const char* bucketNames[4] = {"Bucket 1", "Bucket 2", "Bucket 3", "Bucket 4"};

//Create objects
RTC_DS3231 rtc;
HX711 scale;

//Shared variables
bool systemBusy = false;          // true whenever ANY pump is running (serialization)
float weightBefore = 0;
unsigned long lastLogTime = 0;
unsigned long lastStatusLogTime = 0;

//SD health tracking - board keeps running (pumps/floats) even if this is false
bool sdHealthy = false;

//Defining interval holding arrays - continuous, 2hr, 4hr, 6hr
const unsigned long checkIntervals[4] = {0UL, 7200000UL, 14400000UL, 21600000UL};
unsigned long lastCheckTime[4] = {0, 0, 0, 0};

bool pumpRunning[4] = {false, false, false, false};   //individual pump flags
bool pumpPending[4] = {false, false, false, false};   //set when a check comes due but system is busy; held until free
unsigned long pumpTimeStart[4] = {0, 0, 0, 0};
DateTime pumpStartTimeRTC[4];

//Snapshot of last completed refill per bucket, for status.csv
char lastRefillTimestamp[4][20];

// Debouncing - MIN
int lastMinState[4]   = {HIGH, HIGH, HIGH, HIGH};
int stableMinState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastMinChangeTime[4] = {0, 0, 0, 0};

// Debouncing - MAX
int lastMaxState[4]   = {HIGH, HIGH, HIGH, HIGH};
int stableMaxState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastMaxChangeTime[4] = {0, 0, 0, 0};

// Previously-logged stable states, so we only write to floatlog.csv on an ACTUAL change,
// not every loop pass - compared against stableMinState/stableMaxState right after debounce settles
int prevStableMinState[4] = {HIGH, HIGH, HIGH, HIGH};
int prevStableMaxState[4] = {HIGH, HIGH, HIGH, HIGH};

//Constant
const unsigned long LOG_INTERVAL = 30000UL;         // 30 sec in ms - contlog refresh
const unsigned long STATUS_LOG_INTERVAL = 600000UL; // 10 min in ms - status.csv rewrite
const unsigned long MAX_PUMP_RUNTIME = 540000UL;    // 9 min - confirmed against real pilot fill time
const unsigned long DEBOUNCE_DELAY = 50UL;           // 50 ms
const int LOADCELL_SAMPLES = 4;                      // dropped from 10 to cut loop blocking time

float calibration_factor = 209.46;


//Formats an RTC DateTime into "YYYY-MM-DD HH:MM:SS" without any String allocation
void formatTimestamp(DateTime dt, char* buf) {
  snprintf(buf, 20, "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
}


//Attempts to (re)initialize the SD card, retrying up to 10 times.
//Returns true if it comes up healthy. Used at boot AND whenever a write fails mid-run.
bool trySdInit() {
  int sdRetries = 0;
  while (!SD.begin(SD_CS_PIN) && sdRetries < 10) {
    Serial.println("SD init failed, retrying...");
    delay(500);
    sdRetries++;
  }
  sdHealthy = (sdRetries < 10);
  return sdHealthy;
}


//Opens a file for writing, attempting SD recovery once if the open fails.
//Prints a Serial confirmation on success, updates the error LED either way.
File openWithRecovery(const char* filename) {
  File f = SD.open(filename, FILE_WRITE);

  if (!f) {
    Serial.print("Write failed opening ");
    Serial.print(filename);
    Serial.println(" - attempting SD recovery...");

    if (trySdInit()) {
      Serial.println("SD recovered - retrying open.");
      f = SD.open(filename, FILE_WRITE);
    }
  }

  if (f) {
    sdHealthy = true;
    digitalWrite(ERROR_LED_PIN, LOW);
  } else {
    sdHealthy = false;
    digitalWrite(ERROR_LED_PIN, HIGH);   // solid on - SD unreachable, but board keeps running
    Serial.print("SD still unreachable - could not open ");
    Serial.println(filename);
  }

  return f;
}


//Custom function to write errors to SD card
void logError(const char* message) {
  char errTimestamp[20];
  formatTimestamp(rtc.now(), errTimestamp);

  File errorLog = openWithRecovery("errorlog.csv");
  if (errorLog) {
    errorLog.print(errTimestamp);
    errorLog.print(",");
    errorLog.println(message);
    errorLog.close();
    Serial.println("Logged to errorlog.csv");
  }
  Serial.println(message);
}


//Writes one row to floatlog.csv the moment a float's debounced state actually changes,
//This is what lets overshoot be calculated as a timing gap:
void logFloatChange(int i, const char* floatType, int newState) {
  char timestamp[20];
  formatTimestamp(rtc.now(), timestamp);

  File floatLog = openWithRecovery("floatlog.csv");
  if (floatLog) {
    floatLog.print(timestamp);
    floatLog.print(",");
    floatLog.print(bucketNames[i]);
    floatLog.print(",");
    floatLog.print(floatType);
    floatLog.print(",");
    floatLog.println(newState);
    floatLog.close();
    Serial.println("Logged to floatlog.csv");
  }
}


//Writes one row to eventlog.csv - called on BOTH a normal max-float stop
//and a dry-run timeout stop
void logPumpEvent(int i, unsigned long runtime, float wBefore, float wAfter, const char* reason) {
  char eventTimestamp[20];
  formatTimestamp(pumpStartTimeRTC[i], eventTimestamp);

  File eventLog = openWithRecovery("eventlog.csv");
  if (eventLog) {
    eventLog.print(eventTimestamp);
    eventLog.print(",");
    eventLog.print(bucketNames[i]);
    eventLog.print(",");
    eventLog.print(runtime);
    eventLog.print(",");
    eventLog.print(wBefore);
    eventLog.print(",");
    eventLog.print(wAfter);
    eventLog.print(",");
    eventLog.print(wBefore - wAfter);
    eventLog.print(",");
    eventLog.println(reason);
    eventLog.close();
    Serial.println("Logged to eventlog.csv");
  }
}


//Overwrites status.csv - a throwaway snapshot
//Called once every 10 mins
void writeStatusFile(const char* timestampNow) {
  if (SD.exists("status.csv")) {
    SD.remove("status.csv");
  }
  File statusFile = openWithRecovery("status.csv");
  if (statusFile) {
    statusFile.print("LastUpdated,");
    statusFile.println(timestampNow);
    statusFile.println("Bucket,LastRefillTime");
    for (int i = 0; i < 4; i++) {
      statusFile.print(bucketNames[i]);
      statusFile.print(",");
      statusFile.println(lastRefillTimestamp[i]);
    }
    statusFile.close();
    Serial.println("Logged to status.csv");
  }
}


//Custom function to blink LED forever - only used for RTC failure, which IS unrecoverable
//for this board since every timestamp depends on it
void criticalErrorHalt() {
  while (true) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(300);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(300);
  }
}

//Every time the Arduino powers on
void setup() {
  Serial.begin(9600);

  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, LOW);

  //Configuring all 4 bucket pins in one loop
  for (int i = 0; i < 4; i++) {
    pinMode(floatMinPins[i], INPUT_PULLUP);
    pinMode(floatMaxPins[i], INPUT_PULLUP);
    pinMode(pumpPins[i], OUTPUT);
    digitalWrite(pumpPins[i], PUMP_OFF);   // active-low board: OFF = HIGH, not LOW
    strcpy(lastRefillTimestamp[i], "Never");
  }

  //Ensuring RTC initializes properly - this one DOES halt, since every log row needs a real timestamp
  if (!rtc.begin()) {
    Serial.println("RTC not found");
    criticalErrorHalt();
  }

  if (rtc.lostPower()) {
    // Only resyncs to compile time if the RTC actually lost power (dead coin cell, first boot, etc.)
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //SD card - retried, but does NOT halt the board. Pumps/floats keep working
  //even if logging is currently broken; recovery is retried on every write attempt.
  Serial.println("Initializing SD card...");
  if (!trySdInit()) {
    Serial.println("SD CARD FAILED after retries - continuing WITHOUT logging. Will keep retrying on writes.");
    digitalWrite(ERROR_LED_PIN, HIGH);
  } else {
    Serial.println("SD CARD OK");
  }

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  //File creation - only attempted if SD came up healthy; if not, files get created
  //later automatically the first time a write succeeds after recovery
  if (sdHealthy) {
    bool contLogExists = SD.exists("contlog.csv");
    File continuousLog = openWithRecovery("contlog.csv");
    if (continuousLog) {
      if (!contLogExists) {
        continuousLog.println("Timestamp,B1_MinState,B1_MaxState,B2_MinState,B2_MaxState,B3_MinState,B3_MaxState,B4_MinState,B4_MaxState,ReservoirWeight");
      }
      continuousLog.close();
    }

    bool eventLogExists = SD.exists("eventlog.csv");
    File eventLog = openWithRecovery("eventlog.csv");
    if (eventLog) {
      if (!eventLogExists) {
        eventLog.println("StartTime,Bucket,Duration,WeightBefore,WeightAfter,WeightDiff,StopReason");
      }
      eventLog.close();
    }

    bool errorLogExists = SD.exists("errorlog.csv");
    File errorLog = openWithRecovery("errorlog.csv");
    if (errorLog) {
      if (!errorLogExists) {
        errorLog.println("Timestamp,ErrorMessage");
      }
      errorLog.close();
    }

    bool floatLogExists = SD.exists("floatlog.csv");
    File floatLog = openWithRecovery("floatlog.csv");
    if (floatLog) {
      if (!floatLogExists) {
        floatLog.println("Timestamp,Bucket,FloatType,NewState");
      }
      floatLog.close();
    }
  }

  Serial.println("--------------------------------");
  Serial.println("Setup complete. Loop starting.");
  Serial.println("--------------------------------");
}


void loop() {
  unsigned long currentTime = millis();

  //DEBOUNCING - all 8 sensors, with immediate change-logging
  for (int i = 0; i < 4; i++) {
    int currentMinReading = digitalRead(floatMinPins[i]);
    if (currentMinReading != lastMinState[i]) {
      lastMinChangeTime[i] = millis();
    }
    if ((millis() - lastMinChangeTime[i]) > DEBOUNCE_DELAY) {
      stableMinState[i] = currentMinReading;
    }
    lastMinState[i] = currentMinReading;

    // Log the instant this float's STABLE state actually changes (not every loop)
    if (stableMinState[i] != prevStableMinState[i]) {
      logFloatChange(i, "MIN", stableMinState[i]);
      prevStableMinState[i] = stableMinState[i];
    }

    int currentMaxReading = digitalRead(floatMaxPins[i]);
    if (currentMaxReading != lastMaxState[i]) {
      lastMaxChangeTime[i] = millis();
    }
    if ((millis() - lastMaxChangeTime[i]) > DEBOUNCE_DELAY) {
      stableMaxState[i] = currentMaxReading;
    }
    lastMaxState[i] = currentMaxReading;

    if (stableMaxState[i] != prevStableMaxState[i]) {
      logFloatChange(i, "MAX", stableMaxState[i]);
      prevStableMaxState[i] = stableMaxState[i];
    }
  }


  //================== CONTINUOUS LOG (every 30 sec, all 4 buckets) ==================
  if (currentTime - lastLogTime >= LOG_INTERVAL) {

    char logtimestamp[20];
    formatTimestamp(rtc.now(), logtimestamp);

    File continuousLog = openWithRecovery("contlog.csv");
    if (continuousLog) {
      continuousLog.print(logtimestamp);

      for (int i = 0; i < 4; i++) {
        continuousLog.print(",");
        continuousLog.print(stableMinState[i]);
        continuousLog.print(",");
        continuousLog.print(stableMaxState[i]);
      }

      float currentReservoirWeight = scale.get_units(LOADCELL_SAMPLES);
      continuousLog.print(",");
      continuousLog.println(currentReservoirWeight);
      continuousLog.close();
      Serial.println("Logged to contlog.csv");
    }

    lastLogTime = currentTime;
  }


  //================== STATUS FILE (every 10 min) ==================
  if (currentTime - lastStatusLogTime >= STATUS_LOG_INTERVAL) {
    char statusTimestamp[20];
    formatTimestamp(rtc.now(), statusTimestamp);
    writeStatusFile(statusTimestamp);
    lastStatusLogTime = currentTime;
  }


  //================== PUMP CONTROL ==================
  for (int i = 0; i < 4; i++) {

    //Interval Gating - Resets interval clock regardless of last float check.
    //If dry but the system is busy, this no longer just gets dropped - it's held as pending.
    if (currentTime - lastCheckTime[i] >= checkIntervals[i]) {
      lastCheckTime[i] = currentTime;

      if (stableMinState[i] == LOW) {
        pumpPending[i] = true;
      }
    }

    // Start pump for whichever bucket is pending as soon as the system is free.
    // Runs every loop (not interval-gated), so a pending request is picked up
    // within one loop cycle of the pump freeing up, not on the next check interval.
    if (pumpPending[i] && !systemBusy && !pumpRunning[i]) {
      if (stableMinState[i] == LOW) {
        weightBefore = scale.get_units(LOADCELL_SAMPLES);

        digitalWrite(pumpPins[i], PUMP_ON);
        systemBusy = true;
        pumpRunning[i] = true;
        pumpTimeStart[i] = millis();
        pumpStartTimeRTC[i] = rtc.now();
      }
      // Cleared either way: if it's no longer dry when its turn comes up, there's nothing to do
      pumpPending[i] = false;
    }

    // Stop check using individual pump flag (normal refill completion)
    if (stableMaxState[i] == HIGH && pumpRunning[i]) {
      digitalWrite(pumpPins[i], PUMP_OFF);
      float weightAfter = scale.get_units(LOADCELL_SAMPLES);
      systemBusy = false;
      pumpRunning[i] = false;

      unsigned long pumpRuntime = millis() - pumpTimeStart[i];
      logPumpEvent(i, pumpRuntime, weightBefore, weightAfter, "Normal");

      // This was a completed refill (hit the max float) - update the status snapshot
      formatTimestamp(rtc.now(), lastRefillTimestamp[i]);
    }

    // Dry run protection using individual pump flag.
    // Writes the same event row a normal stop would, so the water it moved
    // still shows up in eventlog.csv instead of vanishing.
    if (pumpRunning[i] && (millis() - pumpTimeStart[i]) > MAX_PUMP_RUNTIME) {
      digitalWrite(pumpPins[i], PUMP_OFF);
      float weightAfter = scale.get_units(LOADCELL_SAMPLES);
      systemBusy = false;
      pumpRunning[i] = false;

      unsigned long pumpRuntime = millis() - pumpTimeStart[i];
      logPumpEvent(i, pumpRuntime, weightBefore, weightAfter, "Timeout(DryRun)");

      char errMsg[64];
      snprintf(errMsg, sizeof(errMsg), "DRY RUN PROTECTION TRIGGERED - %s", bucketNames[i]);
      logError(errMsg);
    }

  }
}