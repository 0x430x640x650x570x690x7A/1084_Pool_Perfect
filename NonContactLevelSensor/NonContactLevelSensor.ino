/*
 * Product Name: Non-Contact Water/Liquid Level Sensor
 * Level Sensor Link: https://www.cqrobot.com/index.php?route=product/product&product_id=1111&search=water+level&description=true
 * Level Sensor Wiki: http://www.cqrobot.wiki/index.php/Non-contact_Water_/_Liquid_Level_Sensor
 * 
 * 
 *Reads HIGH when there is adequate liquid
 *Reads LOW when no liquid is detected
 */

const byte levelSensorPin = 2;
bool Liquid_level = 0;

void setup() {
  Serial.begin(9600);
  pinMode(levelSensorPin, INPUT);
  //DDRD &= ~0x08;
}

void loop() {
  //Liquid_level = PIND & 0x08;
  Liquid_level = digitalRead(levelSensorPin);
  Serial.print("Liquid_level= ");
  Serial.println(Liquid_level);
  delay(500);
}
