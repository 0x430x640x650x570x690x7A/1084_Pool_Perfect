/*
 * Link to product: https://www.robotshop.com/en/4-channel-5v-relay-shield-module.html?utm_source=google&utm_medium=surfaces&utm_campaign=surfaces_across_google_usen
 * Link to pin mapping: https://icircuit.net/arduino-boards-pin-mapping/141
 * http://jim-st.blogspot.com/2014/05/arduino-mega-2650-pin-mapping.html
 * Details: 4 Channel 5V Relay Shield Module
 * relay is active low
 */
 // Port C mega pins 31(C6) 33(C4) 35(C2) 37(C0)
 //set these pins as outputs
//DDRC |= 0x55; // B0101 0101

byte relayPin0 = 31;
byte relayPin1 = 33;
byte relayPin2 = 35;
byte relayPin3 = 37;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
//DDRC |= 0x55; // B0101 0101
pinMode(relayPin0, OUTPUT);
pinMode(relayPin1, OUTPUT);
pinMode(relayPin2, OUTPUT);
pinMode(relayPin3, OUTPUT);

//PORTC &= ~0x55;
digitalWrite(relayPin0,HIGH);
digitalWrite(relayPin1,HIGH);
digitalWrite(relayPin2,HIGH);
digitalWrite(relayPin3,HIGH);
}

void loop() {
  delay(10);
  // put your main code here, to run repeatedly:

digitalWrite(relayPin0,LOW);
delay(1000);
digitalWrite(relayPin0,HIGH);
delay(1000);
digitalWrite(relayPin1,LOW);
delay(1000);
digitalWrite(relayPin1,HIGH);
delay(1000);
digitalWrite(relayPin2,LOW);
delay(1000);
digitalWrite(relayPin2,HIGH);
delay(1000);
/*
digitalWrite(relayPin3,LOW);
delay(1000);
digitalWrite(relayPin3,HIGH);
delay(1000);
*/
}
