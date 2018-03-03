

void setup() {
  Serial.begin(9600);
  pinMode(1,INPUT);
}

void loop() {
  int n = analogRead(1);
  Serial.println(n);
  delay(100);
}
