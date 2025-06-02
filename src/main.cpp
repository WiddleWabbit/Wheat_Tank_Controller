#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Pin to measure voltage from (voltage from current to voltage converter)
const int analogPin = A2; // Analog input pin connected to SEN0262 OUT

// Reference Voltage measured from the Elegoo Mega 2560's reference port.
//const float VREF = 4.98;  // Reference voltage of Mega 2560 ADC (measure with voltmeter for accuracy)
const float VREF = 5.0;

// Sensors measure from 0 - 2 metres
const float MIN_DEPTH = 0.0; // Minimum depth (0m at 4mA)
const float MAX_DEPTH = 2.0; // Maximum depth (2m at 20mA)

// Out current of the sensor is 4 - 20mA
const float MIN_CURRENT = 4.0; // 4mA corresponds to 0m
const float MAX_CURRENT = 20.0; // 20mA corresponds to 2m

// Output voltage of the DFRobot Current to Voltage Converter is 0-3V
const float MIN_VOLTAGE = 0.96; // 0V corresponds to 0mA
const float MAX_VOLTAGE = 2.88; // 3V corresponds to 25mA

// Create an offset for each sensor
const float OFFSET_SENSOR0 = -0.13;

// Construct instance of ADS1115
Adafruit_ADS1115 ads;

void setup() {

  // Setup builtin LED so we can use it if we want.
  pinMode(LED_BUILTIN, OUTPUT);

  // Begin Serial on 115200 baudrate
  Serial.begin(115200);

  // Begin listening to the ADS1115 on the address 0x48
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  // Set ADC gain (allows for input range of 4.096V)
  ads.setGain(GAIN_ONE);

}

void loop() {

  int16_t adc0 = ads.readADC_SingleEnded(0);

  float voltage0 = adc0 * 0.125 / 1000.0;
  float current0 = (voltage0 / 3.0) * 25.0;

  current0 = current0 + OFFSET_SENSOR0;

  // Map current to depth (4-20mA to 0-5m)
  float depth0;
  if (current0 < MIN_CURRENT) {
    depth0 = -1.0; // Fault condition (<4mA)
  } else if (current0 > MAX_CURRENT) {
    depth0 = -2.0; // Overrun condition (>20mA)
  } else {
    depth0 = (current0 - MIN_CURRENT) * (MAX_DEPTH - MIN_DEPTH) / (MAX_CURRENT - MIN_CURRENT) + MIN_DEPTH;

  }

  // Output results
  Serial.print("Voltage: ");
  Serial.print(voltage0, 3);
  Serial.print("V, Current: ");
  Serial.print(current0, 2);
  Serial.print("mA, Depth: ");
  if (depth0 == -1.0) {
    Serial.println("Fault (<4mA)");
  } else if (depth0 == -2.0) {
    Serial.println("Overrun (>20mA)");
  } else {
    Serial.print(depth0, 2);
    Serial.println("m");
  }
  
  delay(1000); // Wait 1 second

}