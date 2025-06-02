#include <Arduino.h>

// Pin to measure voltage from (voltage from current to voltage converter)
const int analogPin = A2; // Analog input pin connected to SEN0262 OUT

// Reference Voltage measured from the Elegoo Mega 2560's reference port.
const float VREF = 4.98;  // Reference voltage of Mega 2560 ADC (measure with voltmeter for accuracy)

// Sensors measure from 0 - 2 metres
const float MIN_DEPTH = 0.0; // Minimum depth (0m at 4mA)
const float MAX_DEPTH = 2.0; // Maximum depth (2m at 20mA)

// Out current of the sensor is 4 - 20mA
const float MIN_CURRENT = 4.0; // 4mA corresponds to 0m
const float MAX_CURRENT = 20.0; // 20mA corresponds to 5m

// Output voltage of the DFRobot Current to Voltage Converter is 0-3V
const float MIN_VOLTAGE = 0.0; // 0V corresponds to 0mA
const float MAX_VOLTAGE = 3.0; // 3V corresponds to 25mA

unsigned int voltage; //unit:mV
float current;  //unit:mA

void setup() {

  // Setup builtin LED so we can use it if we want.
  pinMode(LED_BUILTIN, OUTPUT);

  // Begin Serial on 115200 baudrate
  Serial.begin(115200);

}

void loop() {

  // Read analog value (0-1023 for 10-bit ADC)
  int sensorValue = analogRead(analogPin);

  // Convert to voltage (0-5V range)
  float voltage = sensorValue * (VREF / 1023.0);

  // Convert voltage to current (0-25mA range)
  float current = (voltage / MAX_VOLTAGE) * 25.0;

  // Map current to depth (4-20mA to 0-5m)
  float depth;
  if (current < MIN_CURRENT) {
    depth = -1.0; // Fault condition (<4mA)
  } else if (current > MAX_CURRENT) {
    depth = -2.0; // Overrun condition (>20mA)
  } else {
    // Convert current to calculate the depth using linear interpolation
    depth = MIN_DEPTH + (MAX_DEPTH - MIN_DEPTH) * (current - MIN_CURRENT) / (MAX_CURRENT - MIN_CURRENT);
  }

  // Output results
  Serial.print("Voltage: ");
  Serial.print(voltage, 3);
  Serial.print("V, Current: ");
  Serial.print(current, 2);
  Serial.print("mA, Depth: ");
  if (depth == -1.0) {
    Serial.println("Fault (<4mA)");
  } else if (depth == -2.0) {
    Serial.println("Overrun (>20mA)");
  } else {
    Serial.print(depth, 2);
    Serial.println("m");
  }
  
  delay(1000); // Wait 1 second

}