#include "HX711.h"

const int LOADCELL_DOUT_PIN = 22;
const int LOADCELL_SCK_PIN = 23;

HX711 scale;

void setup() {
  Serial.begin(9600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(209.46); // Calibrated for weight output in grams
  scale.tare();            // Zero out the scale on startup
}

void loop() {
  if (scale.is_ready()) {
    // get_units(5) applies your set_scale(209.46) and averages 5 readings
    float weight = scale.get_units(15); 
    
    Serial.print("Weight: ");
    Serial.print(weight, 1);
    Serial.println(" g");
  } else {
    Serial.println("HX711 timed out (Not Ready) - check wiring!");
  }
  delay(500);
}