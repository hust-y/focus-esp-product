void setup() {
  // put your setup code here, to run once:
    pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(4,HIGH);
    delay(500);
    digitalWrite(4, LOW);
    delay (500);
}
