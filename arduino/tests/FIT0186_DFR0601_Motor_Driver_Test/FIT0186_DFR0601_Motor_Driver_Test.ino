// Test Code for FIT0186 Motors and DFR0601 Motor Drivers
// --- Motor 1 (Driver A, Channel 1) ---
const int pwm1 = 4;   const int ina1 = 22;  const int inb1 = 23;
const int enc1A = 18; const int enc1B = 30; // Interrupt on 18

// --- Motor 2 (Driver A, Channel 2) ---
const int pwm2 = 5;   const int ina2 = 24;  const int inb2 = 25;
const int enc2A = 19; const int enc2B = 31; // Interrupt on 19

// --- Motor 3 (Driver B, Channel 1) ---
const int pwm3 = 6;   const int ina3 = 26;  const int inb3 = 27;
const int enc3A = 20; const int enc3B = 32; // Interrupt on 20

// --- Motor 4 (Driver B, Channel 2) ---
const int pwm4 = 7;   const int ina4 = 28;  const int inb4 = 29;
const int enc4A = 21; const int enc4B = 33; // Interrupt on 21

// --- Encoder Variables ---
volatile long pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;

void setup() {
  Serial.begin(115200);
  
  // Set Control Pins
  int outPins[] = {pwm1, ina1, inb1, pwm2, ina2, inb2, pwm3, ina3, inb3, pwm4, ina4, inb4};
  for(int i=0; i<12; i++) pinMode(outPins[i], OUTPUT);
  
  // Set Encoder Pins
  int inPins[] = {enc1A, enc1B, enc2A, enc2B, enc3A, enc3B, enc4A, enc4B};
  for(int i=0; i<8; i++) pinMode(inPins[i], INPUT_PULLUP);

  // Attach Interrupts (Mega Pins 18, 19, 20, 21)
  attachInterrupt(digitalPinToInterrupt(enc1A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc2A), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(enc3A), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(enc4A), readEncoder4, RISING);

  Serial.println("4-Motor System Ready.");
}

void loop() {
  setAllMotors(150, true); // Speed 150, Forward=true

  // Debug Print
  Serial.print("P1:"); Serial.print(pos1);
  Serial.print(" | P2:"); Serial.print(pos2);
  Serial.print(" | P3:"); Serial.print(pos3);
  Serial.print(" | P4:"); Serial.println(pos4);
  
  delay(100);
}

void setAllMotors(int speed, bool forward) {
  digitalWrite(ina1, forward); digitalWrite(inb1, !forward);
  digitalWrite(ina2, forward); digitalWrite(inb2, !forward);
  digitalWrite(ina3, forward); digitalWrite(inb3, !forward);
  digitalWrite(ina4, forward); digitalWrite(inb4, !forward);
  
  analogWrite(pwm1, speed); analogWrite(pwm2, speed);
  analogWrite(pwm3, speed); analogWrite(pwm4, speed);
}

// --- Interrupt Functions ---
void readEncoder1() { (digitalRead(enc1B)) ? pos1++ : pos1--; }
void readEncoder2() { (digitalRead(enc2B)) ? pos2++ : pos2--; }
void readEncoder3() { (digitalRead(enc3B)) ? pos3++ : pos3--; }
void readEncoder4() { (digitalRead(enc4B)) ? pos4++ : pos4--; }
