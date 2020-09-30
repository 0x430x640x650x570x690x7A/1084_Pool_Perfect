/*
   Incorporates all aspects of design:
      I2C display
      Noncontact level sensors
      pH, ORP and Temp probes
      Relay Module
      Current clamp
      Arduino
*/
//include all necessary libraries
//#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // needed for I2C display


//include all pin connections and variables
//I2C display init variables
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

//level sensor init variables
const byte levelPinChlorine = 22;
const byte levelPinMuriatic = 1;
const byte levelPinSodaAsh = 2;
const byte levelPinCyanuric = 3;
bool readChlorineLevel = 0; // == F
bool readMuriaticLevel = 0;
bool readSodaAshLevel = 0;
bool readCyanuricLevel = 0;

//probe init variables
const byte probePinPH = 6;
const byte probePinORP = 6;
const byte probePinTemp = 6;
int readpH = 0;
int readORP = 0;
int readTemp = 0;
int chlorineReading = 0;

//relay init variables
const byte relayPin0Chlorine = 6;
const byte relayPin1Muriatic = 6;
const byte relayPin2SodaAsh = 6;
const byte relayPin3Cyanuric = 6;
int pumpDelay = 0;
const byte standbyTime = 5; // in min

//current clamp init variables
const byte pinCurrentClamp = 6;
int readCurrentClamp = 0;
const byte currentCutOff = 5; // amps

//function prototypes
void initDisplay();
void initLevelSensor();
void initProbe();
void initRelay();

bool checkMainPump();
bool checkChemicalLevels();
void checkProbes();
void updateDisplay();
void runPumps();
void doNothing();

void setup() {
  Serial.begin(9600); // interact with serial monitor
  initDisplay();
  initLevelSensor();
  initProbe();
  initRelay();
  pinMode(pinCurrentClamp, INPUT); // init current clamp input
}

void loop() {
  delay(2);
  //if my main motor is on/true run the while loop
  while (checkMainPump()) {
    //main pump is determined to be on
    //check if chemicals are adequate
    if (checkChemicalLevels()) {
      //if i have enough chemicals then i want to check probes and update display
      checkProbes();
      updateDisplay();
      //run my pumps
      runPumps();
    } else {
      //write to LED to refill chemicals in checking chemicals function
    }
    //wait after ive dispensed some chemicals
    //look more into sleep function from arduino library
    doNothing();
  }
  //dont want to continuously check main pump power
  //doNothing();
  //LowPower.deepSleep();
  //need to wakeup
}

void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("BootingSystem");
  display.display();
  delay(2000);
}

void initLevelSensor() {
  pinMode(levelPinChlorine, INPUT);
  pinMode(levelPinMuriatic, INPUT);
  pinMode(levelPinSodaAsh, INPUT);
  pinMode(levelPinCyanuric, INPUT);
}

void initProbe() {

}

void initRelay() {
  // set control line as outputs
  //DDRC |= 0x55; // B0101 0101
  pinMode(relayPin0Chlorine, OUTPUT);
  pinMode(relayPin1Muriatic, OUTPUT);
  pinMode(relayPin2SodaAsh, OUTPUT);
  pinMode(relayPin3Cyanuric, OUTPUT);

  //relay is active low, set all pins high
  //PORTC &= ~0x55;
  digitalWrite(relayPin0Chlorine, HIGH);
  digitalWrite(relayPin1Muriatic, HIGH);
  digitalWrite(relayPin2SodaAsh, HIGH);
  digitalWrite(relayPin3Cyanuric, HIGH);
}

bool checkMainPump() {
  //read raw analog value
  readCurrentClamp = analogRead(pinCurrentClamp);
  //check if it exceeds our threshold
  if (readCurrentClamp >= currentCutOff)
    return true;
  else
    return false;
}

bool checkChemicalLevels() {
  //read level of each level sensor
  //could do if between each read to not waste time reading all values if one is too low
  readChlorineLevel = digitalRead(levelPinCyanuric);
  readMuriaticLevel = digitalRead(levelPinCyanuric);
  readSodaAshLevel = digitalRead(levelPinCyanuric);
  readCyanuricLevel = digitalRead(levelPinCyanuric);
  //forget if active high or low
  //if levels are too low are true else if i have enough chemicals
  //may need to play around with text size
  if (readChlorineLevel) {
    display.clearDisplay();
    display.print("Low Chlorine");
    //
    display.display();
    return false;
  } else if (readMuriaticLevel) {
    display.clearDisplay();
    display.print("Low Muriatic Acid");
    display.display();
    return false;
  } else if (readSodaAshLevel) {
    display.clearDisplay();
    display.print("Low Soda Ash");
    display.display();
    return false;
  } else if (readCyanuricLevel) {
    display.clearDisplay();
    display.print("Low Cyanuric Acid");
    display.display();
    return false;
  } else {
    return true;
  }
}

void checkProbes() {
  //raw probe values orp ph temp


  //math

  //leave functoin with ph chlorine temp
}

void updateDisplay() {
  display.setTextSize(2); // could put in init but error messages may not be the correct size
  display.clearDisplay();
  // Display static text
  display.print("pH:   ");
  display.println(readPH);
  display.print("Cl:   ");
  display.println(chlorineReading);
  display.print("Temp: ");
  display.println(readTemp);
  display.display();
}

void runPumps() {
  //based on my readings turn on correct
  //hardcode in init value ie change 1ml i need 2.7sec on pump
  delay(pumpDelay * 1000);

}

void doNothing() {
  //loop for one minute
  for (int i = 0; i <= standbyTime; ++i) {
    //wait one minute
    //uses int values so cant exceed ~32k
    delay(30 * 1000);
    delay(30 * 1000);
  }
}
