/*
   Incorporates all aspects of design:
      I2C display
      Noncontact level sensors
      pH, ORP and Temp probes
      Relay Module
      Current clamp
      Arduino

      to do:
      calculate chlorine
      pump vvvvvvDelay
*/
//include all necessary libraries
#include <Adafruit_SSD1306.h> // needed for I2C display
#include "VirtuinoCM.h" // integration with app
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

// Display: App
VirtuinoCM virtuino;
boolean debug = false;              // set this variable to false on the finale code to decrease the request time.

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
const byte standbyTime = .2; // in min
int pumpDelay = 5; //seconds

//current clamp init variables
const byte currentClampPin = A0;         //set arduino signal read pin
const byte ACTectionRange = 10; //set Non-invasive AC Current Sensor tection range (5A,10A,20A)
const byte currentCutOff = 5; // amps

byte sampleSize = 20;

//function prototypes
void initApp();
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
  float V0_lastValuePH = 7.5;
  //in case we cant find correct gain for calibration
  float V1_lastValueChlorine = 2.50;
  int V2_lastValueTemp = 88;
  //v3 is ph indicator
  //v4 is chlorine indicator
  static const byte V_memory_count = 5;          // the size of V memory can change it to a number <=255)
  float V[V_memory_count] = {V0_lastValuePH, V1_lastValueChlorine, V2_lastValueTemp};           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
} sensor;
//sensorInfo sensor;

void setup() {
  virtuinoRun();
  initApp();
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
  vDelay(2);
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
    //vDelay(1000);
  }
  //dont want to continuously check main pump power
  doNothing();
  
  //need to wakeup
  //vDelay(1000);
}

void initApp() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
  Serial1.begin(9600);
  //Serial1.setTimeout(50);

  virtuino.begin(onReceived, onRequested, 256); //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
}

void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  vDelay(2000);
  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("BootingSystem");
  display.display();
  vDelay(2000);
  display.setTextSize(2); // could put in init but error messages may not be the correct size
  display.clearDisplay();
  display.setCursor(0, 10);
  // Display static text
  display.print("pH:   ");
  display.println(sensor.V0_lastValuePH);
  display.print("Cl:   ");
  display.println(sensor.V1_lastValueChlorine);
  display.print("Temp: ");
  display.print(sensor.V2_lastValueTemp);
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
    vDelay(samplingInterval);
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

void checkProbes() {
  bool newPHReading = false;
  bool newChlorineReading = false;
  bool newTempReading = false;

  digitalWrite(sensorArrayStatusPin, HIGH);
  Serial.println("Sensor Array ON");

  Serial.println("Reading values");
  while (true) {
    if (sensor.V[0] != sensor.V0_lastValuePH) {           // The V0 has changed
      vDelay(500); // need a delay because sometimes multiple readings sent
      //Serial.println("pH = " + String(sensor.V[0]));  // print the value of V0
      sensor.V0_lastValuePH = sensor.V[0];// store the V0 to the variable V0_lastValue so as to know if it has changed
      newPHReading = true;
    }
    sensor.V[3] = sensor.V[0]; // update ph indicator

    if (sensor.V[1] != sensor.V1_lastValueChlorine) {            // The V1 has changed
      vDelay(500);
      //Serial.println("Chlorine = " + String(sensor.V[1]));  // print the value of V1
      sensor.V[4] = sensor.V[1]; // update chlorine indicator
      sensor.V1_lastValueChlorine = sensor.V[1];                 // store the V1 to the variable V1_lastValue so as to know if it has changed
      newChlorineReading = true;
    }
    sensor.V[4] = sensor.V[1]; // update chlorine indicator
    
    if (sensor.V[2] != sensor.V2_lastValueTemp) {     // The V1 has changed
      vDelay(500);
      //Serial.println("Temperature = " + String(sensor.V[2]));  // print the value of V1
      sensor.V2_lastValueTemp = sensor.V[2];                 // store the V1 to the variable V1_lastValue so as to know if it has changed
      newTempReading = true;
    }

    vDelay(10);
    if (newPHReading && newChlorineReading && newTempReading) {
      break;
    }
  }
  //Serial.println("Sending values");

  vDelay(2000);     // This is an example of the recommended delay function. Remove this if you don't need
  Serial.println("Sensor Array OFF");
  digitalWrite(sensorArrayStatusPin, LOW);
}

//tested
void updateDisplay() {
  Serial.println("Updating Display");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.clearDisplay();
  // Display static text
  display.print("pH:   ");
  display.println(sensor.V0_lastValuePH, 2);
  display.print("Cl:   ");
  display.println(sensor.V1_lastValueChlorine, 2);
  display.print("Temp: ");
  display.print(sensor.V2_lastValueTemp);
  display.setTextSize(1);
  display.println("F");
  display.display();
}

void runPumps() {
  digitalWrite(pumpArrayStatusPin, HIGH);
  Serial.println("Pump Array ON");

  //check pH thn chlorine
  if (sensor.V0_lastValuePH < sensor.lowBoundPH) {
    //add soda ash
    //calculate vDelay for soda ash
    digitalWrite(relayPin2SodaAsh, LOW);
    //wait for desired time
    vDelay(pumpDelay * 1000);
    digitalWrite(relayPin2SodaAsh, HIGH);
  } else if (sensor.V0_lastValuePH > sensor.highBoundPH) {
    //add muraitic acid
    //calculate pump vDelay for muriatic acid
    digitalWrite(relayPin1Muriatic, LOW);
    //wait for desired time
    vDelay(pumpDelay * 1000);
    digitalWrite(relayPin1Muriatic, HIGH);
  } else if (sensor.V1_lastValueChlorine < sensor.lowBoundChlorine) {
    //increase chlorine ppm
    //calculate chlorine pumpvDelay
    //turn on correct relay
    digitalWrite(relayPin0Chlorine, LOW);
    //wait for desired time
    vDelay(pumpDelay * 1000);
    digitalWrite(relayPin0Chlorine, HIGH);
  } else {
    //we cant do anything if chlorine is too high
  }

  Serial.println("Pump Array OFF");
  digitalWrite(pumpArrayStatusPin, LOW);
}

//untested
void doNothing() {
  //loop for one minute
  for (int i = 0; i <= standbyTime; ++i) {
    //wait one minute
    //uses int values so cant exceed ~32k
    vDelay(30 * 1000);
    vDelay(30 * 1000);
  }
}

/* This function is called every time Virtuino app sends a request to server to change a Pin value
   The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin
   The 'variableIndex' is the pin number index of Virtuino app
   The 'valueAsText' is the value that has sent from the app   */
void onReceived(char variableType, uint8_t variableIndex, String valueAsText) {
  if (variableType == 'V') {
    float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
    if (variableIndex < sensor.V_memory_count) sensor.V[variableIndex] = value;          // copy the received value to arduino V memory array
  }
}

// function everytime a virtuino pin is read
String onRequested(char variableType, uint8_t variableIndex) {
  if (variableType == 'V') {
    if (variableIndex < sensor.V_memory_count) return  String(sensor.V[variableIndex]); // return the value of the arduino V memory array
  }
  return "";
}

//driver function for interfacing with app
void virtuinoRun() {
  while (Serial1.available()) {
    char tempChar = Serial1.read();
    if (tempChar == CM_START_CHAR) {             // a new command is starting...
      virtuino.readBuffer = CM_START_CHAR;   // copy the new command to the virtuino readBuffer
      virtuino.readBuffer += Serial1.readStringUntil(CM_END_CHAR);
      virtuino.readBuffer += CM_END_CHAR;
      //if (debug) Serial.println("\nCommand= " + virtuino.readBuffer);
      String* response = virtuino.getResponse();   // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
      //if (debug) Serial.println("Response : " + *response);
      Serial1.print(*response);
      break;
    }
  }
}

//virtuino Delay
void vDelay(int vDelayInMillis) {
  long t = millis() + vDelayInMillis;
  while (millis() < t) virtuinoRun();
}
