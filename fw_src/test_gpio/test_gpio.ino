int GPIO5 = 5;
int ADC1;
float cell1_v;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(GPIO5, OUTPUT);
}

void loop() {
    // put your main code here, to run repeatedly:
    ADC1 = analogRead(A0);
    Serial.println(ADC1);
    cell1_v = ADC1 * (5.0 / 1023.0);
    Serial.println(cell1_v);
    cell1_v = cell1_v * 5.08;
    Serial.println(cell1_v);
    delay(1000);

    // Serial.println("Turning on bleed resistors");
    // digitalWrite(GPIO5, HIGH);
    // delay(10000);
    // Serial.println("Turning off bleed resistors");
    // digitalWrite(GPIO5, LOW);
    // delay(10000);
}
