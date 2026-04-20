// Test Code to run the Creality 42-34(Z) Stepper Motor and Adafruit 49888 Driver
const int stepPin = 3; // Connect to A4988 STEP
const int dirPin = 4;  // Connect to A4988 DIR

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  digitalWrite(dirPin, HIGH); // Set direction

  // Spin 200 steps (1 full rotation if in full-step mode)
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // Speed: lower = faster
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  
  delay(1000); // Wait 1 second

  digitalWrite(dirPin, LOW); // Reverse direction
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  delay(1000);
}

