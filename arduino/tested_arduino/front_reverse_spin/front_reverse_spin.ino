// Front Motor driver connections
int FP1 = 12;
int FA1 = 51;
int FB1 = 49;

int FP2 = 13;
int FA2 = 52;
int FB2 = 50;

// Rear Motor driver connections
int RP1 = 10;
int RA1 = 25;
int RB1 = 27;

int RP2 = 11;
int RA2 = 24;
int RB2 = 26;

// About 1/4 of full speed (255 max)
const int motorSpeed = 75;

void setup() {
  pinMode(FP1, OUTPUT);
  pinMode(FP2, OUTPUT);
  pinMode(FA1, OUTPUT);
  pinMode(FB1, OUTPUT);
  pinMode(FA2, OUTPUT);
  pinMode(FB2, OUTPUT);

  pinMode(RP1, OUTPUT);
  pinMode(RP2, OUTPUT);
  pinMode(RA1, OUTPUT);
  pinMode(RB1, OUTPUT);
  pinMode(RA2, OUTPUT);
  pinMode(RB2, OUTPUT);

  stopMotors();
}

void loop() {
 // moveForward();
 // delay(5000);

 // moveReverse();
 // delay(5000);

  spinInPlace();
  delay(3000);

  stopMotors();
  delay(800);
}

void setSpeed(int speedVal) {
  analogWrite(FP1, speedVal);
  analogWrite(FP2, speedVal);
  analogWrite(RP1, speedVal);
  analogWrite(RP2, speedVal);
}

void stopMotors() {
  analogWrite(FP1, 0);
  analogWrite(FP2, 0);
  analogWrite(RP1, 0);
  analogWrite(RP2, 0);

  digitalWrite(FA1, LOW);
  digitalWrite(FB1, LOW);
  digitalWrite(FA2, LOW);
  digitalWrite(FB2, LOW);

  digitalWrite(RA1, LOW);
  digitalWrite(RB1, LOW);
  digitalWrite(RA2, LOW);
  digitalWrite(RB2, LOW);
}

void moveForward() {
  // Same direction pattern as your working forward code
  digitalWrite(FA1, LOW);
  digitalWrite(FB1, HIGH);  // Motor 1 forward

  digitalWrite(FA2, HIGH);
  digitalWrite(FB2, LOW);   // Motor 2 forward

  digitalWrite(RA1, HIGH);
  digitalWrite(RB1, LOW);   // Motor 3 forward

  digitalWrite(RA2, LOW);
  digitalWrite(RB2, HIGH);  // Motor 4 forward

  setSpeed(motorSpeed);
}

void moveReverse() {
  // Opposite of forward
  digitalWrite(FA1, HIGH);
  digitalWrite(FB1, LOW);

  digitalWrite(FA2, LOW);
  digitalWrite(FB2, HIGH);

  digitalWrite(RA1, LOW);
  digitalWrite(RB1, HIGH);

  digitalWrite(RA2, HIGH);
  digitalWrite(RB2, LOW);

  setSpeed(motorSpeed);
}

void spinInPlace() {
  // Right side opposite of left side for rotation
  // Left wheels: forward
  digitalWrite(FA2, HIGH);
  digitalWrite(FB2, LOW);   // Front left forward

  digitalWrite(RA1, HIGH);
  digitalWrite(RB1, LOW);   // Back left forward

  // Right wheels: reverse
  digitalWrite(FA1, HIGH);
  digitalWrite(FB1, LOW);   // Front right reverse

  digitalWrite(RA2, HIGH);
  digitalWrite(RB2, LOW);   // Back right reverse

  setSpeed(motorSpeed);
}
