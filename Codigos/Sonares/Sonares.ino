#define SEL0    40
#define SEL1    41
#define SEL2    42
#define ECHO    43
#define BINH    44
#define INIT    45

void setup() {
  Serial.begin(57600);
  pinMode(SEL0, OUTPUT);
  pinMode(SEL1, OUTPUT);
  pinMode(SEL2, OUTPUT);
  pinMode(INIT, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(SEL0, HIGH);
  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, LOW);
  digitalWrite(INIT, LOW);
}

void loop() {
  digitalWrite(INIT, HIGH);
  delay(3);
  Serial.println(digitalRead(ECHO));
  digitalWrite(INIT, LOW);
}
