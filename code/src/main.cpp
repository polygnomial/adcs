#include <Arduino.h>
#include <Wire.h>
#include "Rm3100.hpp"

// put function declarations here:
int myFunction(int, int);

char buf[128];
int addr = 0x20; //((Rm3100::RM3100_ADDR | Rm3100::RM3100_ADDR_SSN) << 1);
struct Rm3100::Status status = { 0 };
struct Rm3100::Sample sample = { 0 };

Rm3100 sensor(Wire, addr);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  sensor.Begin();
  delay(250);

  sensor.SetCycleCounts(200);
  delay(250);

  sensor.SetRate(100.0f);
  delay(250);

  sensor.SetContinuousMeasurementMode(true);
  delay(250);

}

// put function definitions here:
int read_sfh2400(uint8_t pin) {
  int val;
  val = analogRead(pin);
  return val;
}

// Photodiode Loop
// void loop() {
//   // put your main code here, to run repeatedly:
//   int val = read_sfh2400(A8);
//   Serial.print("analog A8 is: ");
//   Serial.println(val);
//   delay(250);
// }


// Mag Sensor Loop
void loop()
{
    sensor.GetStatus(&status);
    if (status.drdy) {
        sensor.GetSample(&sample);
        
        sprintf(buf, "x: %f, y: %f, z: %f\r\n", sample.x, sample.y, sample.z);
        Serial.print(buf);
    }
    delay(250);
}

