/*
   Clamp type: Gravity Analog AC Current Sensor (10A)
   Clamp link: https://www.robotshop.com/en/gravity-analog-ac-current-sensor-10a.html
   Clamp Wiki:

*/

/* ***UNTESTED*** */

const byte clampPin = A2;         //set arduino signal read pin
const byte ACTectionRange = 10; //set Non-invasive AC Current Sensor tection range (5A,10A,20A)

float rawVoltage = 0;
float correctedCurrentValue = 0;
float voltageRMS = 0;

byte sampleSize = 20;

void setup()
{
  Serial.begin(9600);
  pinMode(clampPin, INPUT);
}

void loop()
{
  for (int i = 0; i < sampleSize; ++i)
  {
    rawVoltage += analogRead(clampPin);   //read peak voltage
    delay(2);
  }
  //change the peak voltage to the RMS voltage
  //voltageRMS = (rawVoltage / sampleSize) * 0.707; 
     
  //The circuit is amplified by 2 so divided by 2
  //vRMS = (vRMS * ref voltage / adc 10 bit ) / amplification
  //voltageRMS = (voltageRMS * 5.0 / 1024 ) / 2;
  

  //correctedCurrentValue = vRMS * tection range
  correctedCurrentValue = ((((rawVoltage / sampleSize) * 0.707) * 5.0 / 1024 ) / 2) * ACTectionRange;
 
  Serial.print(correctedCurrentValue);
  Serial.println(" A");

}
