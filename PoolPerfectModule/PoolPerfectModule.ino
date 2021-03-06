/*
INCOMPLETE DUE TO SCOPE CHANGE

   Incorporates all aspects of design:
      I2C display
      Noncontact level sensors
      pH, ORP and Temp probes
      Relay Module
      Current clamp
      Arduino

      to do:
      calculate chlorine
      pump delay
*/
//include all necessary libraries
//#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // needed for I2C display
#include "DallasTemperature.h" // for temperature sensor


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

//probe init variables
const byte probePinPH = A0;
const byte probePinORP = A1;
const byte probePinTemp = 6;
const byte samplingInterval = 10; //ms

// Probe: Temp
//set data from sensor
OneWire dataWireTempProbe(probePinTemp);
//create sensor object to use function calls
DallasTemperature tempSensor(&dataWireTempProbe);

//relay init variables
const byte relayPin0Chlorine = 31;
const byte relayPin1Muriatic = 33;
const byte relayPin2SodaAsh = 35;
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

//save all sensor info in this object
struct sensorInfo {
  const float lowBoundPH = 7.3;
  const float highBoundPH = 7.8;
  const float lowBoundChlorine = 2; //ppm
  const float highBoundChlorine = 3; //ppm
  //need float for floating point arithmetic
  float correctedReadingPH = 5.23;
  //in case we cant find correct gain for calibration
  float myOffsetPH = 0.00;
  float correctedReadingORP = 0;
  int myOffsetORP = -11;
  float chlorineReading = 0;
  int readTemp = 98;
} sensor;
//sensorInfo sensor;

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
  display.println(sensor.correctedReadingPH);
  display.print("Cl:   ");
  display.println(sensor.chlorineReading);
  display.print("Temp: ");
  display.print(sensor.readTemp);
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
  //pH
  pinMode(probePinPH, INPUT);
  //ORP
  pinMode(probePinORP, INPUT);
  //Temp
  //init sensor
  tempSensor.begin();
  //change sensor accuracy
  tempSensor.setResolution(11);
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

//tested
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

//tested
bool checkChemicalLevels() {
  bool readChlorineLevel = 0; // == F
  bool readMuriaticLevel = 1;
  bool readSodaAshLevel = 1;

  //read level of each level sensor
  //could do if between each read to not waste time reading all values if one is too low
  readChlorineLevel = digitalRead(levelPinChlorine);
  //readMuriaticLevel = digitalRead(levelPinMuriatic);
  //readSodaAshLevel = digitalRead(levelPinSodaAsh);

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
//UNTESTED
void checkProbes() {
  int rawReadingPH = 0;
  int rawReadingORP = 0;
  float rawReadingTemp = 0;

  digitalWrite(sensorArrayStatusPin, HIGH);
  Serial.println("Sensor Array ON");

  //raw probe values orp ph temp
  //look into ssampling all probes at same time
  for (int i = 0; i < sampleSize; ++i) {
    //pH
    rawReadingPH += analogRead(probePinPH);
    //ORP
    rawReadingORP += analogRead(probePinORP);
    //temp
    tempSensor.requestTemperatures();
    //take reading using 1 wire library
    rawReadingTemp += tempSensor.getTempCByIndex(0);
    delay(samplingInterval);
  }
  //do saomething with raw readings
  //pH
  //scaling factor * ((raw * voltage / adc bit size / samples) + deviation) + correctionValue
  sensor.correctedReadingPH = 3.5 * ((rawReadingPH * 5.0 / 1024 / sampleSize) + 0.05) + sensor.myOffsetPH;
  //ORP
  //((30 * voltage * 1k?) - 75? * avg reading * voltage * 1k? / 10 bit )) / 75?
  sensor.correctedReadingORP = ((30 * 5.0 * 1000) - (75 * ((float)rawReadingORP / sampleSize) * 5.0 * 1000 / 1024)) / 75 - sensor.myOffsetORP;
  //Temp
  sensor.readTemp = rawReadingTemp / sampleSize;
  /*
    //math MUST FIND CHLORINE
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

//tested
void updateDisplay() {
  Serial.println("Updating Display");
  /*
    //for testing
    correctedReadingPH++;
    chlorineReading++;
    readTemp++;
    if (correctedReadingPH >= 14)
    correctedReadingPH = 0;
    if (chlorineReading >= 3)
    chlorineReading = 0;
  */
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.clearDisplay();
  // Display static text
  display.print("pH:   ");
  display.println(sensor.correctedReadingPH, 2);
  display.print("Cl:   ");
  display.println(sensor.chlorineReading, 2);
  display.print("Temp: ");
  display.print(sensor.readTemp);
  display.setTextSize(1);
  display.println("F");
  display.display();
}
//partially UNTESTED
void runPumps() {
  int pumpDelay = 0; //seconds
  digitalWrite(pumpArrayStatusPin, HIGH);
  Serial.println("Pump Array ON");

  //based on my readings set correct pump delay*******
  //I NEED MATH

  //check pH thn chlorine
  if (sensor.correctedReadingPH < sensor.lowBoundPH) {
    //add soda ash
    //calculate delay for soda ash
    digitalWrite(relayPin2SodaAsh, LOW);
    //wait for desired time
    delay(pumpDelay * 1000);
    digitalWrite(relayPin2SodaAsh, HIGH);
  } else if (sensor.correctedReadingPH > sensor.highBoundPH) {
    //add muraitic acid
    //calculate pump delay for muriatic acid
    digitalWrite(relayPin1Muriatic, LOW);
    //wait for desired time
    delay(pumpDelay * 1000);
    digitalWrite(relayPin1Muriatic, HIGH);
  } else if (sensor.chlorineReading < sensor.lowBoundChlorine) {
    //increase chlorine ppm
    //calculate chlorine pump delay
    //turn on correct relay
    digitalWrite(relayPin0Chlorine, LOW);
    //wait for desired time
    delay(pumpDelay * 1000);
    digitalWrite(relayPin0Chlorine, HIGH);
  } else {
    //we cant do anything if chlorine is too high
  }


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

//untested
void doNothing() {
  //loop for one minute
  for (int i = 0; i <= standbyTime; ++i) {
    //wait one minute
    //uses int values so cant exceed ~32k
    delay(30 * 1000);
    delay(30 * 1000);
  }
}
