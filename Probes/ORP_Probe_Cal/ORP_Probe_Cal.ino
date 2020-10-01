/*
   Probe type: Gravity: Analog ORP Sensor Meter For Arduino
   Probe link: https://www.dfrobot.com/product-1071.html
   Product Wiki: https://wiki.dfrobot.com/Analog_ORP_Meter_SKU_SEN0165_

*/

/* ***UNTESTED***** */

const byte probePin = A2;
//may need to make long if we have large sample size
//assumption is max reading is 414 and int only has ~32k so 77 * 414 > 32k
int rawReading = 0;
//need float for floating point arithmetic
float correctedReading = 0;
//in case we cant find correct gain for calibration
float myOffset = 0;

int sampleSize = 20;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(probePin, INPUT);
}

void loop() {
  delay(2);
  //take reading and add it to buffer
  for (int i = 0; i < sampleSize; ++i) {
    rawReading += analogRead(probePin);
    Serial.print("Raw values: ");
    Serial.println(rawReading, 2);
    //sampling interval
    delay(2);
  }
  //may need to cast rawreading to float
  //not sure here 30 and 75 come from maybe scaling value or something to do with sample size
  //could be something with the hardware
  //((30 * voltage * 1k?) - 75? * avg reading * voltage * 1k? / 10 bit )) / 75?
  correctedReading = ((30 * 5.0 * 1000) - (75 * ((float)rawReading / sampleSize) * 5.0 * 1000 / 1024)) / 75; //- OFFSET;

  Serial.print("ORP: ");
  Serial.println(correctedReading, 2);
  delay(500);
}
