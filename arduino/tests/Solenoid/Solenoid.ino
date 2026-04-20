const int solenoidPin = 13;
#define DO_PIN 7  // Arduino's pin connected to DO pin of the flame sensor

void setup() {
  Serial.begin(9600);
  pinMode(solenoidPin, OUTPUT);
  pinMode(DO_PIN, INPUT);
}
 
void loop() {

   int flame_state = digitalRead(DO_PIN);
    delay(1000);                       

  if (flame_state == HIGH)
  {
    Serial.println("The flame is NOT present => The fire is NOT detected");
    digitalWrite(solenoidPin, LOW);      
  }
  else
  {
    Serial.println("The flame is present => The fire is detected");
    digitalWrite(solenoidPin, HIGH);   
  }        
}