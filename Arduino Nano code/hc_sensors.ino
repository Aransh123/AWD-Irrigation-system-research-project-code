#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

RTC_DS3231 rtc;

// ================= SENSOR PINS =================
const int TRIG1 = 9;
const int ECHO1 = 6;
const int TRIG2 = 7;
const int ECHO2 = 8;
const int TRIG3 = 2;
const int ECHO3 = 3;
const int TRIG4 = A0;
const int ECHO4 = A1;

// ================= SD CARD =================
const int SD_CS = 4;

// ================= ERROR LED =================
const int ERROR_LED_PIN = 5;   // unused pin on this board's layout

// ================= TIMING =================
const unsigned long SENSOR_DELAY = 200;   // 200 ms between sensors
const unsigned long LOG_INTERVAL = 5000;  // 5 seconds
const unsigned long ECHO_TIMEOUT = 30000; // 30 ms

bool sdHealthy = false;


// =================================================
// READ ULTRASONIC DISTANCE
// =================================================
float readDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT);

  if (duration == 0)
    return -1.0;

  return duration * 0.0343 / 2.0;
}


// =================================================
// PRINT TWO DIGITS
// =================================================
void printTwoDigits(File &file, int number)
{
  if (number < 10)
    file.print("0");

  file.print(number);
}


// =================================================
// BLINK LED - short pulse means "just logged successfully"
// =================================================
void blinkSuccess()
{
  digitalWrite(ERROR_LED_PIN, HIGH);
  delay(50);
  digitalWrite(ERROR_LED_PIN, LOW);
}


// =================================================
// SOLID-ON/BLINK LED FOREVER - unrecoverable failure (RTC missing entirely)
// =================================================
void criticalErrorHalt()
{
  while (true) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(300);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(300);
  }
}


// =================================================
// ATTEMPT TO (RE)INITIALIZE THE SD CARD
// =================================================
bool trySdInit()
{
  int sdRetries = 0;
  while (!SD.begin(SD_CS) && sdRetries < 10) {
    Serial.println("SD init failed, retrying...");
    delay(500);
    sdRetries++;
  }
  sdHealthy = (sdRetries < 10);
  return sdHealthy;
}


// =================================================
// SETUP
// =================================================
void setup()
{
  Serial.begin(9600);

  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, LOW);

  // ================= SENSOR PINS =================
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  pinMode(TRIG4, OUTPUT);
  pinMode(ECHO4, INPUT);

  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);
  digitalWrite(TRIG3, LOW);
  digitalWrite(TRIG4, LOW);

  // ================= SPI =================
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // ================= RTC =================
  Wire.begin();

  if (!rtc.begin())
  {
    Serial.println("RTC not found");
    criticalErrorHalt();
  }

  if (rtc.lostPower()) {
    // Only resyncs to compile time if the RTC actually lost power (dead coin cell, first boot, etc.)
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime now = rtc.now();

  Serial.print("RTC Time: ");

  if (now.day() < 10) Serial.print("0");
  Serial.print(now.day());
  Serial.print("/");

  if (now.month() < 10) Serial.print("0");
  Serial.print(now.month());
  Serial.print("/");

  Serial.print(now.year());
  Serial.print(" ");

  if (now.hour() < 10) Serial.print("0");
  Serial.print(now.hour());
  Serial.print(":");

  if (now.minute() < 10) Serial.print("0");
  Serial.print(now.minute());
  Serial.print(":");

  if (now.second() < 10) Serial.print("0");
  Serial.println(now.second());


  // ================= SD CARD (with retry) =================
  Serial.println("Initializing SD card...");

  if (!trySdInit())
  {
    Serial.println("SD CARD FAILED after retries!");
    criticalErrorHalt();
  }

  Serial.println("SD CARD OK");


  // ================= CSV FILE =================
  if (!SD.exists("DATA.CSV"))
  {
    File dataFile = SD.open("DATA.CSV", FILE_WRITE);

    if (dataFile)
    {
      dataFile.println(
        "Date,Time,Bucket1_cm,Bucket2_cm,Bucket3_cm,Bucket4_cm"
      );

      dataFile.close();

      Serial.println("Created DATA.CSV");
    }
    else
    {
      Serial.println("ERROR CREATING DATA.CSV");
      criticalErrorHalt();
    }
  }


  Serial.println("--------------------------------");
  Serial.println("4-Bucket LOGGER STARTED");
  Serial.println("--------------------------------");
}


// =================================================
// LOOP
// =================================================
void loop()
{
  // ================= BUCKET 1 =================
  float distance1 = readDistance(TRIG1, ECHO1);
  Serial.print("Bucket 1: ");
  if (distance1 < 0) Serial.println("NO ECHO");
  else { Serial.print(distance1, 1); Serial.println(" cm"); }
  delay(SENSOR_DELAY);

  // ================= BUCKET 2 =================
  float distance2 = readDistance(TRIG2, ECHO2);
  Serial.print("Bucket 2: ");
  if (distance2 < 0) Serial.println("NO ECHO");
  else { Serial.print(distance2, 1); Serial.println(" cm"); }
  delay(SENSOR_DELAY);

  // ================= BUCKET 3 =================
  float distance3 = readDistance(TRIG3, ECHO3);
  Serial.print("Bucket 3: ");
  if (distance3 < 0) Serial.println("NO ECHO");
  else { Serial.print(distance3, 1); Serial.println(" cm"); }
  delay(SENSOR_DELAY);

  // ================= BUCKET 4 =================
  float distance4 = readDistance(TRIG4, ECHO4);
  Serial.print("Bucket 4: ");
  if (distance4 < 0) Serial.println("NO ECHO");
  else { Serial.print(distance4, 1); Serial.println(" cm"); }

  // ================= RTC =================
  DateTime now = rtc.now();

  // ================= SAVE TO SD (with recovery attempt) =================
  File dataFile = SD.open("DATA.CSV", FILE_WRITE);

  if (!dataFile)
  {
    Serial.println("ERROR opening DATA.CSV - attempting SD recovery...");
    if (trySdInit()) {
      Serial.println("SD recovered - retrying this write.");
      dataFile = SD.open("DATA.CSV", FILE_WRITE);
    } else {
      Serial.println("SD RECOVERY FAILED - card unreachable.");
      digitalWrite(ERROR_LED_PIN, HIGH);
    }
  }

  if (dataFile)
  {
    printTwoDigits(dataFile, now.day());
    dataFile.print("/");
    printTwoDigits(dataFile, now.month());
    dataFile.print("/");
    dataFile.print(now.year());
    dataFile.print(",");

    printTwoDigits(dataFile, now.hour());
    dataFile.print(":");
    printTwoDigits(dataFile, now.minute());
    dataFile.print(":");
    printTwoDigits(dataFile, now.second());
    dataFile.print(",");

    if (distance1 < 0) dataFile.print("NO ECHO"); else dataFile.print(distance1, 1);
    dataFile.print(",");
    if (distance2 < 0) dataFile.print("NO ECHO"); else dataFile.print(distance2, 1);
    dataFile.print(",");
    if (distance3 < 0) dataFile.print("NO ECHO"); else dataFile.print(distance3, 1);
    dataFile.print(",");
    if (distance4 < 0) dataFile.print("NO ECHO"); else dataFile.print(distance4, 1);

    dataFile.println();
    dataFile.close();

    Serial.println("Saved to SD card.");
    digitalWrite(ERROR_LED_PIN, LOW);
    blinkSuccess();
  }

  Serial.println("--------------------------------");

  delay(LOG_INTERVAL);
}