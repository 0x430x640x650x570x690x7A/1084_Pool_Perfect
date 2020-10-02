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
//Display init variables
//Display: I2C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Display: LEDS
const byte mainPumpStatusPin = 23;
const byte chemicalLevelStatusPin = 25;
const byte sensorArrayStatusPin = 27;
const byte pumpArrayStatusPin = 29;

//level sensor init variables
const byte levelPinChlorine = 13;
const byte levelPinMuriatic = 14;
const byte levelPinSodaAsh = 15;
bool readChlorineLevel = 0; // == F
bool readMuriaticLevel = 1;
bool readSodaAshLevel = 1;


//probe init variables  ***need to add more variables for probes
const byte samplingInterval = 10; //ms
const byte probePinPH = A0;
const byte probePinORP = A1;
const byte probePinTemp = 6;
const float lowBoundpH = 7.3;
const float highBoundpH = 7.8;
const float lowBoundChlorine = 2; //ppm
const float highBoundChlorine = 3; //ppm
int rawReadingPH = 0;
//need float for floating point arithmetic
float correctedReadingPH = 5.23;
//in case we cant find correct gain for calibration
float myOffsetPH = 0.00;

int readORP = 0;
int readTemp = 98;
float chlorineReading = 0;

//relay init variables
const byte relayPin0Chlorine = 31;
const byte relayPin1Muriatic = 33;
const byte relayPin2SodaAsh = 35;
int pumpDelay = 1; //seconds
const byte standbyTime = 5; // in min

//current clamp init variables
const byte currentClampPin = A0;         //set arduino signal read pin
const byte ACTectionRange = 10; //set Non-invasive AC Current Sensor tection range (5A,10A,20A)
const byte currentCutOff = 5; // amps

byte sampleSize = 20;

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
  pinMode(currentClampPin, INPUT); // init current clamp input
  digitalWrite(mainPumpStatusPin, LOW);
  digitalWrite(chemicalLevelStatusPin, LOW);
  digitalWrite(sensorArrayStatusPin, LOW);
  digitalWrite(pumpArrayStatusPin, LOW);

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
    //doNothing();
    delay(1000);
  }
  //dont want to continuously check main pump power
  //doNothing();
  //LowPower.deepSleep();
  //need to wakeup
  delay(1000);
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
  display.setTextSize(2); // could put in init but error messages may not be the correct size
  display.clearDisplay();
  display.setCursor(0, 10);
  // Display static text
  display.print("pH:   ");
  display.println(correctedReadingPH);
  display.print("Cl:   ");
  display.println(chlorineReading);
  display.print("Temp: ");
  display.print(readTemp);
  display.setTextSize(1);
  display.println("F");
  display.display();
}

void initLevelSensor() {
  pinMode(levelPinChlorine, INPUT);
  pinMode(levelPinMuriatic, INPUT);
  pinMode(levelPinSodaAsh, INPUT);
}

void initProbe() {

}

void initRelay() {
  // set control line as outputs
  //DDRC |= 0x55; // B0101 0101
  pinMode(relayPin0Chlorine, OUTPUT);
  pinMode(relayPin1Muriatic, OUTPUT);
  pinMode(relayPin2SodaAsh, OUTPUT);

  //relay is active low, set all pins high
  //PORTC &= ~0x55;
  digitalWrite(relayPin0Chlorine, HIGH);
  digitalWrite(relayPin1Muriatic, HIGH);
  digitalWrite(relayPin2SodaAsh, HIGH);
}

bool checkMainPump() {
  float rawVoltage = 0;
  float correctedCurrentValue = 0;
  //float voltageRMS = 0;
  for (int i = 0; i < sampleSize; ++i)
  {
    rawVoltage += analogRead(currentClampPin);   //read peak voltage
    delay(samplingInterval);
  }

  correctedCurrentValue = ((((rawVoltage / sampleSize) * 0.707) * 5.0 / 1024 ) / 2) * ACTectionRange;
  if (correctedCurrentValue >= currentCutOff) {
    Serial.println("Main Pump ON");
    digitalWrite(mainPumpStatusPin, HIGH);
    return true;
  } else {
    Serial.println("Main Pump OFF");
    digitalWrite(mainPumpStatusPin, LOW);
    return false;
  }
}

bool checkChemicalLevels() {
  //read level of each level sensor
  //could do if between each read to not waste time reading all values if one is too low
  readChlorineLevel = digitalRead(levelPinChlorine);
  //readMuriaticLevel = digitalRead(levelPinMuriatic);
  //readSodaAshLevel = digitalRead(levelPinSodaAsh);
  //readCyanuricLevel = digitalRead(levelPinCyanuric);
  //if any reading is low then i need to say it is low
  if (readChlorineLevel && readMuriaticLevel && readSodaAshLevel) {
    Serial.println("Chemical Level HIGH");
    digitalWrite(chemicalLevelStatusPin, LOW);
    return true;
  } else {
    Serial.println("Chemical Level LOW");
    digitalWrite(chemicalLevelStatusPin, HIGH);
    return false;
  }
}

void checkProbes() {
  digitalWrite(sensorArrayStatusPin, HIGH);
  Serial.println("Sensor Array ON");
  //raw probe values orp ph temp
  //look into ssampling all probes at same time

  //delay for testing purposes
  delay(3000);
  /*
    //math
    //i know temp i know ph and i know ORP

    //finding chlorine levels
    //switches dont work with flaots
    switch (readpH * 10) {
    case 0 ... 68:
      //equation based off ORP
      //chlorineReading = do math;
      break;
    case 69:
      break;
    case 70:
      break;
    case 71:
      break;
    case 72:
      break;
    case 73:
      break;
    case 74:
      break;
    case 75:
      break;
    case 76:
      break;
    case 77:
      break;
    case 78:
      break;
    case 79:
      break;
    case 80:
      break;
    case 81 ... 140:
      break;
    default:
      break;
    }
    //leave function with ph chlorine temp
  */
  Serial.println("Sensor Array OFF");
  digitalWrite(sensorArrayStatusPin, LOW);
}

void updateDisplay() {
  Serial.println("Updating Display");
  //for testing
  correctedReadingPH++;
  chlorineReading++;
  readTemp++;
  if (correctedReadingPH >= 14)
    correctedReadingPH = 0;
  if (chlorineReading >= 3)
    chlorineReading = 0;
    
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.clearDisplay();
  // Display static text
  display.print("pH:   ");
  display.println(correctedReadingPH, 2);
  display.print("Cl:   ");
  display.println(chlorineReading, 2);
  display.print("Temp: ");
  display.print(readTemp);
  display.setTextSize(1);
  display.println("F");
  display.display();
}

void runPumps() {
  digitalWrite(pumpArrayStatusPin, HIGH);
  Serial.println("Pump Array ON");
  //based on my readings set correct pump delay
  /*
    //check chlorine then ph
    if (chlorineReading < lowBoundChlorine) {
    //increase chlorine ppm
    //turn on correct relay
    digitalWrite(relayPin0Chlorine, LOW);
    //wait for desired time
    delay(pumpDelay * 1000);
    digitalWrite(relayPin0Chlorine, HIGH);
    } else
    //we cant do anything if chlorine is too high
    }
  */

  //for testing purposes
  digitalWrite(relayPin0Chlorine, LOW);
  delay(1000);
  digitalWrite(relayPin0Chlorine, HIGH);
  delay(1000);
  digitalWrite(relayPin1Muriatic, LOW);
  delay(1000);
  digitalWrite(relayPin1Muriatic, HIGH);
  delay(1000);
  digitalWrite(relayPin2SodaAsh, LOW);
  delay(1000);
  digitalWrite(relayPin2SodaAsh, HIGH);
  delay(1000);

  Serial.println("Pump Array OFF");
  digitalWrite(pumpArrayStatusPin, LOW);
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
