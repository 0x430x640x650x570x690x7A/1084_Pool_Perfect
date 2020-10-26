/*
   Product Name: Non-Contact Water/Liquid Level Sensor
   Level Sensor Link: https://www.cqrobot.com/index.php?route=product/product&product_id=1111&search=water+level&description=true
   Level Sensor Wiki: http://www.cqrobot.wiki/index.php/Non-contact_Water_/_Liquid_Level_Sensor


  Reads HIGH when there is adequate liquid
  Reads LOW when no liquid is detected
*/

const byte levelSensorPin0 = 10; //chlorine
const byte levelSensorPin1 = 9; //soda ash
const byte levelSensorPin2 = 11; //muriatic acid
bool Liquid_level = 0;

void setup() {
  Serial.begin(9600);
  pinMode(levelSensorPin0, INPUT);
  pinMode(levelSensorPin1, INPUT);
  pinMode(levelSensorPin2, INPUT);
  //DDRD &= ~0x08;
}

void loop() {
  //Liquid_level = PIND & 0x08;
  bool chlorine = digitalRead(levelSensorPin0);
  bool sodaAsh = digitalRead(levelSensorPin1);
  bool muriatic = digitalRead(levelSensorPin2);
  Serial.print("Liquid_level= ");
  Serial.println(chlorine);
  Serial.println(sodaAsh);
  Serial.println(muriatic);
  delay(500);
}
