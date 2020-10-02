/*
   Probe type: Gravity: Analog pH Sensor / Meter Pro Kit
   Probe link: https://store.arduino.cc/usa/analog-ph-sensor-meter-pro-kit
   Product Wiki: https://wiki.dfrobot.com/Industrial_pH_electrode_SKU_FIT0348_

*/

const byte probePin = A2;
//may need to make long if we have large sample size
//assumption is max reading is 414 and int only has ~32k so 77 * 414 > 32k
long rawReading = 0;
//need float for floating point arithmetic
float correctedReading = 0;
//in case we cant find correct gain for calibration
float myOffset = 0.00;

int sampleSize = 200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(probePin, INPUT);
}

void loop() {
  delay(2);
  //take reading and add it to
  for (int i = 0; i < sampleSize; ++i) {
    rawReading += analogRead(probePin);
    delay(2);
  }
  //may need to cast rawreading to float
  //scaling factor * ((raw * voltage / adc bit size / samples) + deviation) + correctionValue
  correctedReading = 3.5 * ((rawReading * 5.0 / 1024 / sampleSize) + 0.05) + myOffset; 
  //same as line before but broken up
  //correctedReading = (rawReading * 5.0 / 1024 / sampleSize) + 0.05;
  //correctedReading = 3.5 * correctedReading + myOffset;
  Serial.print("pH: ");
  Serial.println(correctedReading, 2);
  delay(500);
  rawReading = 0;
}

/*
float myMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/
