// Front Motor driver connections
int FP1 = 12;
int FA1 = 10;
int FB1 = 11;

int FP2 = 13;
int FA2 = 8;
int FB2 = 9;

// Rear Motor driver connections
int RP1 = 5;
int RA1 = 4;
int RB1 = 7;

int RP2 = 2;
int RA2 = 3;
int RB2 = 6;


void setup() {
  // Set all the motor control pins to outputs
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

  // Turn off motors - Initial state
  digitalWrite(FP1, LOW);
  digitalWrite(FP2, LOW);
  digitalWrite(RP1, LOW);
  digitalWrite(RP2, LOW);
}

void loop() {
  //directionControl();
  delay(1000);
  
  speedControl();
  delay(1000);
}

// This function lets you control spinning direction of motors
void directionControl() {
  // Set motors to maximum speed
  digitalWrite(FP1, HIGH);
  digitalWrite(FP2, HIGH);

  // Turn on motor A & B
  digitalWrite(FA1, HIGH);
  digitalWrite(FB1, LOW);
  digitalWrite(FA2, HIGH);
  digitalWrite(FB2, LOW);
  delay(2000);

  // Now change motor directions
  digitalWrite(FA1, LOW);
  digitalWrite(FB1, HIGH);
  digitalWrite(FA2, LOW);
  digitalWrite(FB2, HIGH);
  delay(2000);

  // Turn off motors
  digitalWrite(FA1, LOW);
  digitalWrite(FB1, LOW);
  digitalWrite(FA2, LOW);
  digitalWrite(FB2, LOW);
}

// This function lets you control speed of the motors
void speedControl() {
  // Turn on motors all one direction
  digitalWrite(FA1, LOW);
  digitalWrite(FB1, HIGH);
  digitalWrite(FA2, HIGH);
  digitalWrite(FB2, LOW);

  digitalWrite(RA1, HIGH);
  digitalWrite(RB1, LOW);
  digitalWrite(RA2, LOW);
  digitalWrite(RB2, HIGH);

    delay(2000);

  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(FP1, i);
    analogWrite(FP2, i);
    analogWrite(RP1, i);
    analogWrite(RP2, i);
    delay(110);
  }

  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(FP1, i);
    analogWrite(FP2, i);
    analogWrite(RP1, i);
    analogWrite(RP2, i);
    delay(110);
  }

  // Now turn off motors
  digitalWrite(FP1, LOW);
  digitalWrite(FP2, LOW);
  digitalWrite(RP1, LOW);
  digitalWrite(RP2, LOW);
}