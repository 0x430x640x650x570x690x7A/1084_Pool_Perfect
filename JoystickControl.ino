// constants won't change. They're used here to
// set pin numbers:
const byte joystickSel = 13;     // the number of the joystick select pin
const byte joystickX = A3;       // the number of the joystick X-axis analog
const byte joystickY =  A0;     // the number of the joystick Y-axis analog

// variables will change:
byte joystickSelState = 0;      // variable for reading the joystick sel status
int joystickXState, joystickYState ;
byte state;

byte count = 0;

const int timer = 250;
unsigned long previousMillis = 0;

void pauseWithoutDelay(const long);

void setup() {
  Serial.begin(9600);
  // By default MSP432 has analogRead() set to 10 bits.
  // This Sketch assumes 12 bits. Uncomment to line below to set analogRead()
  // to 12 bit resolution for MSP432.
  //analogReadResolution(12);
  // initialize the LED pins as  output:
  DDRD |= B00111100;
  DDRB |= B00001111;
  //DDRC = B00000000;
  pinMode(joystickX, INPUT);
  pinMode(joystickY, INPUT);
  // initialize the pushbutton pin as an input:
  pinMode(joystickSel, INPUT_PULLUP);

}

void loop() {
  // button change for switching between axis inputs
  joystickSelState = digitalRead(joystickSel);
  if (joystickSelState == LOW) count++;
  if (count % 2 == 0) {
    // read the analog value of joystick x axis
    joystickXState = analogRead(joystickX);
    // scale the analog input range [0,4096] into the analog write range [0,255]
    state = map(joystickXState, 0, 1023, 0, 99);
    // output to the led
  } else {
    // read the analog value of joystick y axis
    joystickYState = analogRead(joystickY);
    // scale the analog input range [0,4096] into the analog write range [0,255]
    state = map(joystickYState, 0, 1023, 0, 99);
    // output to the led
  }

  // ones
  PORTB = (state % 10);
  //tens
  PORTD = (state / 10 % 10) << 2;

  pauseWithoutDelay(timer);
}

void pauseWithoutDelay(const long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return;
  }
  else
    pauseWithoutDelay(interval);
}
