/*
   Link to product: https://www.robotshop.com/en/4-channel-5v-relay-shield-module.html?utm_source=google&utm_medium=surfaces&utm_campaign=surfaces_across_google_usen
   Link to pin mapping: https://icircuit.net/arduino-boards-pin-mapping/141
   http://jim-st.blogspot.com/2014/05/arduino-mega-2650-pin-mapping.html
   Details: 4 Channel 5V Relay Shield Module
   relay is active low
*/
// Port C mega pins 31(C6) 33(C4) 35(C2) 37(C0)
//set these pins as outputs
//DDRC |= 0x55; // B0101 0101

byte relayPin0 = 35; //chlorine
byte relayPin1 = 37; //soda ash
byte relayPin2 = 39; //muriatic acid

int runningInterval = 3000; // 3sec

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //DDRC |= 0x55; // B0101 0101
  pinMode(relayPin0, OUTPUT); //chlorine
  pinMode(relayPin1, OUTPUT); //soda ash
  pinMode(relayPin2, OUTPUT); //muriatic acid


  //PORTC &= ~0x55;
  //digitalWrite(relayPin0, LOW);
  //digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin0, HIGH);
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);

}

void runPumps();
void stopPumps();
void runPumpChlorine();
void runPumpSodaAsh();
void runPumpMuriatic();
void runPumpsTimed();

void loop() {
  delay(10);
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char ch = Serial.read();
    switch (ch) {
      //run both pumps
      case 'r':
      case 'R':
        runPumps();
        break;
      //run high ground pump
      case 'c':
      case 'C':
        runPumpChlorine();
        break;
      //run low ground pump
      case 'a':
      case 'A':
        runPumpSodaAsh();
        break;
      case 'm':
      case 'M':
        runPumpMuriatic();
        break;
      //stop pumps
      case 's':
      case 'S':
        stopPumps();
        break;
      //run timed pumps
      case 't':
      case 'T':
        runPumpsTimed();
        break;
      default:
        break;
    }
  }
}

void runPumps() {
  digitalWrite(relayPin0, LOW);
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, LOW);
  Serial.println("Running");
}

void stopPumps() {
  digitalWrite(relayPin0, HIGH);
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  Serial.println("Stopped");
}

void runPumpChlorine() {
  digitalWrite(relayPin0, LOW);
  Serial.println("Running Chlorine");
}

void runPumpSodaAsh() {
  digitalWrite(relayPin1, LOW);
  Serial.println("Running SodaAsh");
}

void runPumpMuriatic() {
  digitalWrite(relayPin2, LOW);
  Serial.println("Running Muriatic Acid");
}

void runPumpsTimed() {
  digitalWrite(relayPin0, LOW);
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, LOW);
  Serial.println("Running Timed");
  Serial.print(runningInterval);
  Serial.println(" ms");
  delay(runningInterval);
  digitalWrite(relayPin0, HIGH);
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  Serial.println("Timed");

}
