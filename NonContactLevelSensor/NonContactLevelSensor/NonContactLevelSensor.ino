bool Liquid_level=0;
void setup() {
 Serial.begin(9600);
 //pinMode(3,INPUT);
 DDRD &= ~0x08;
}

void loop() {
Liquid_level = PIND & 0x08;
//Liquid_level=digitalRead(3);
Serial.print("Liquid_level= ");
Serial.println(Liquid_level);
delay(500);
}
