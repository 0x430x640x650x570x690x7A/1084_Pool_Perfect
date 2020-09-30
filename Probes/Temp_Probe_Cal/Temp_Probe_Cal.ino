/* 
 * Waterproof DS18B20 Digital temperature sensor + extras
 * Probe: https://www.adafruit.com/product/381
 * Similar Tutorial: https://www.instructables.com/id/Calibration-of-DS18B20-Sensor-With-Arduino-UNO/
 * Wiring: https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806?f=1
 */
 
//#include "Wire.h"
//#include "OneWire.h"
#include "DallasTemperature.h"


const byte pinTempProbe = A0;
float readTempProbe = 0;

//set data from sensor
OneWire dataWireTempProbe(pinTempProbe);
//create sensor object to use function calls
DallasTemperature tempSensor(&dataWireTempProbe);

void setup() {

  delay(1000);
  Serial.begin(9600);
  //init sensor
  tempSensor.begin();
  //change sensor accuracy
  tempSensor.setResolution(11);

}
void loop() {
  //init temp request
  tempSensor.requestTemperatures();
  //take reading using 1 wire library
  readTempProbe = tempSensor.getTempCByIndex(0);
  //convert to F from C
  readTempProbe = (readTempProbe * 9 / 5) + 32;
  //print out sensor data
  Serial.print(readTempProbe);
  Serial.println(" F");
  delay(500);
}

/* value correction
float RawHigh = 99.6;
float RawLow = 0.5;
float ReferenceHigh = 99.9;
float ReferenceLow = 0;
float RawRange = RawHigh - RawLow;
float ReferenceRange = ReferenceHigh - ReferenceLow;
float CorrectedValue = (((RawValue - RawLow) * ReferenceRange) / RawRange) + ReferenceLow;
*/
