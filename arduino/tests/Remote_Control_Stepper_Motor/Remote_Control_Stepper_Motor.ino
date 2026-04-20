// Test Code for Linear Solenoid and Stepper Motor control w/ buttons
// --- Stepper Pins ---
const int stepPin = 3; 
const int dirPin = 4;

// --- Solenoid Pin ---
const int solenoidPin = 7;

// --- External Button Pins ---
const int buttonUp = 10;
const int buttonDown = 11;
const int buttonStart = 12;


void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);
  
  // Internal pullups handle the "High" state for your external buttons
  pinMode(buttonUp, INPUT_PULLUP);
  pinMode(buttonDown, INPUT_PULLUP);
  pinMode(buttonStart, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("External Buttons Active. Hold to move rod.");
}

void loop() {
  // digitalRead returns LOW when the external button is pressed
  if (digitalRead(buttonUp) == LOW) {
    digitalWrite(dirPin, HIGH); 
    moveOneStep(2000);
  } 
  else if (digitalRead(buttonDown) == LOW) {
    digitalWrite(dirPin, LOW); 
    moveOneStep(2000);
  }

  // Solenoid Logic (Momentary)
  if (digitalRead(buttonStart) == LOW) {
    digitalWrite(solenoidPin, HIGH);   // ON while pressed
  } else {
    digitalWrite(solenoidPin, LOW);    // OFF when released
  }
}

// --- Speed/Torque Setting ---
// 800 is good for speed, 1200 is better for heavy lifting
void moveOneStep(int stepDelay) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);
}
