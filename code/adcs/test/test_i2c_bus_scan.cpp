// --------------------------------------
// i2c_scanner
//
// From https://learn.adafruit.com/scanning-i2c-addresses/arduino
// 
// Make sure to run test with 
// pio test -f i2c_bus_scan -v
//
// --------------------------------------

#include <Wire.h>
#include <unity.h>

// I2C Scanner
// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

void setUp() {
  WIRE.begin();

  Serial.begin(9600);
  while (!Serial)
     delay(10);
  Serial.println("\nI2C Scanner");
}

void test_i2c_bus_scan() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}

int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_i2c_bus_scan);

    UNITY_END();
}