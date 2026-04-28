const int solenoidPin = 4;

const int stepPin = 8;
const int dirPin  = 9;

const int startDelayUs = 400;
const int finalDelayUs = 170;
const int rampStepUs   = 10;
const int rampStepsPerSpeed = 40;

const unsigned long runTime = 5300;             // stepper run time
const unsigned long solenoidStartDelay = 3000;  // wait 3 sec after GO
const unsigned long solenoidOnTime = 1000;      // solenoid ON for 1 sec
const unsigned long postSolenoidDelay = 3000;   // wait after solenoid turns OFF
const unsigned long reverseDelay = 5000;        // wait 5 sec before reversing

bool sequenceStarted = false;
bool programDone = false;

String cmd = "";

void setup() {
  pinMode(solenoidPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(solenoidPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, LOW);

  Serial.begin(9600);

  delay(1000);
  while (Serial.available()) Serial.read();

  Serial.println("Type GO to run the sequence once");
}

void stepMotor(int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

void rampUpMotor() {
  for (int d = startDelayUs; d >= finalDelayUs; d -= rampStepUs) {
    for (int i = 0; i < rampStepsPerSpeed; i++) {
      stepMotor(d);
    }
  }
}

void runStepperForDuration() {
  digitalWrite(dirPin, LOW);   // original direction
  unsigned long startTime = millis();

  rampUpMotor();

  while (millis() - startTime < runTime) {
    stepMotor(finalDelayUs);
  }

  digitalWrite(stepPin, LOW);
}

void reverseStepperForDuration() {
  digitalWrite(dirPin, HIGH);  // reverse direction
  unsigned long startTime = millis();

  rampUpMotor();

  while (millis() - startTime < runTime) {
    stepMotor(finalDelayUs);
  }

  digitalWrite(stepPin, LOW);
}

void runSequenceOnce() {
  Serial.println("GO received");

  Serial.println("Waiting 3 seconds before solenoid...");
  delay(solenoidStartDelay);

  Serial.println("Solenoid ON");
  digitalWrite(solenoidPin, HIGH);
  delay(solenoidOnTime);

  Serial.println("Solenoid OFF");
  digitalWrite(solenoidPin, LOW);

  Serial.println("Waiting before stepper starts...");
  delay(postSolenoidDelay);

  Serial.println("Stepper starting...");
  runStepperForDuration();

  Serial.println("Waiting 5 seconds before reverse reset...");
  delay(reverseDelay);

  Serial.println("Stepper reversing...");
  reverseStepperForDuration();

  Serial.println("Sequence complete");
  digitalWrite(solenoidPin, LOW);
  digitalWrite(stepPin, LOW);

  programDone = true;
}

void loop() {
  if (programDone) {
    return;
  }

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      cmd.trim();
      cmd.toUpperCase();

      if (cmd == "GO" && !sequenceStarted) {
        sequenceStarted = true;
        runSequenceOnce();
      }

      cmd = "";
    } else {
      cmd += c;
    }
  }
}