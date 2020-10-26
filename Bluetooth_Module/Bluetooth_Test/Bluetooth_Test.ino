/* Edited example code for virtuino app using the Bluetooth HC-06 + Arduino MEGA - getting started

*/

#include "VirtuinoCM.h"
VirtuinoCM virtuino;

boolean debug = true;              // set this variable to false on the finale code to decrease the request time.
float V0_lastValuePH = 7.5;
float V1_lastValueChlorine = 2.5;
float V2_lastValueTemp = 88;
//V3 is pH indicator
//V4 is chlorine indicator
const byte V_memory_count = 5;          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count] = {V0_lastValuePH, V1_lastValueChlorine, V2_lastValueTemp};           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.

void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
  Serial1.begin(9600);
  //Serial1.setTimeout(50);
  virtuino.begin(onReceived, onRequested, 256); //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuinoRun();
}

void loop() {
  virtuinoRun();
  bool newPHReading = false;
  bool newChlorineReading = false;
  bool newTempReading = false;
  Serial.println("\nReading values");
  while (true) {
    if (V[0] != V0_lastValuePH) {           // The V0 has changed
      vDelay(500); // need a delay because sometimes multiple readings sent
      Serial.println("pH = " + String(V[0]));  // print the value of V0
      V0_lastValuePH = V[0];// store the V0 to the variable V0_lastValue so as to know if it has changed
      newPHReading = true;

    }
    V[3] = V[0];
    if (V[1] != V1_lastValueChlorine) {            // The V1 has changed
      vDelay(500);
      Serial.println("Chlorine = " + String(V[1]));  // print the value of V1
      V1_lastValueChlorine = V[1];                 // store the V1 to the variable V1_lastValue so as to know if it has changed
      newChlorineReading = true;

    }
    V[4] = V[1];
    if (V[2] != V2_lastValueTemp) {     // The V1 has changed
      vDelay(500);
      Serial.println("Temperature = " + String(V[2]));  // print the value of V1
      V2_lastValueTemp = V[2];                 // store the V1 to the variable V1_lastValue so as to know if it has changed
      newTempReading = true;
    }

    vDelay(10);
    if (newPHReading && newChlorineReading && newTempReading) {
      break;
    }
  }
  Serial.println("Sending values\n");

  vDelay(2000);     // This is an example of the recommended delay function. Remove this if you don't need
}


/* This function is called every time Virtuino app sends a request to server to change a Pin value
   The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin
   The 'variableIndex' is the pin number index of Virtuino app
   The 'valueAsText' is the value that has sent from the app   */
void onReceived(char variableType, uint8_t variableIndex, String valueAsText) {
  if (variableType == 'V') {
    float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
    if (variableIndex < V_memory_count) V[variableIndex] = value;          // copy the received value to arduino V memory array
  }
}

// function everytime a virtuino pin is read
String onRequested(char variableType, uint8_t variableIndex) {
  if (variableType == 'V') {
    if (variableIndex < V_memory_count) return  String(V[variableIndex]); // return the value of the arduino V memory array
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


//virtuino delay
void vDelay(int delayInMillis) {
  long t = millis() + delayInMillis;
  while (millis() < t) virtuinoRun();
}
