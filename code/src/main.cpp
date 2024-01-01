#include <Arduino.h>
#include <Wire.h>
#include "Rm3100.hpp"

#define HWSERIAL Serial2

// char buf[128];
// int addr = 0x20; //((Rm3100::RM3100_ADDR | Rm3100::RM3100_ADDR_SSN) << 1);
// struct Rm3100::Status status = { 0 };
// struct Rm3100::Sample sample = { 0 };

// Rm3100 sensor(Wire, addr);

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
  
//   sensor.Begin();
//   delay(250);

//   sensor.SetCycleCounts(200);
//   delay(250);

//   sensor.SetRate(100.0f);
//   delay(250);

//   sensor.SetContinuousMeasurementMode(true);
//   delay(250);

// }

// // put function definitions here:
// int read_sfh2400(uint8_t pin) {
//   int val;
//   val = analogRead(pin);
//   return val;
// }

// // Photodiode Loop
// void loop() {
//   // put your main code here, to run repeatedly:
//   int val0 = read_sfh2400(A0);
//   int val1 = read_sfh2400(A1);
//   int val2 = read_sfh2400(A2);
//   int val3 = read_sfh2400(A3);
//   Serial.print("analog readings: " + String(val0) + "  " + String(val1) + "  " + String(val2) + "  " + String(val3) + "\r\n");
//   delay(250);
// }


// // Mag Sensor Loop
// void loop()
// {
//     sensor.GetStatus(&status);
//     if (status.drdy) {
//         sensor.GetSample(&sample);
        
//         sprintf(buf, "x: %f, y: %f, z: %f\r\n", sample.x, sample.y, sample.z);
//         Serial.print(buf);
//     }
//     delay(250);
// }

#include <TinyGPS.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

TinyGPS gps;

void setup()
{
  Serial.begin(115200);
  HWSERIAL.begin(9600);
  
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (HWSERIAL.available())
    {
      char c = HWSERIAL.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing

      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}