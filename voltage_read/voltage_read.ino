// This is the voltage reading code
// During the testing, it was okay to connect the 4.2V into one of the ADC pin
// If the voltage is higher than 3.3V, the reading just becomes 1023.
// TODO: Find the value when it hits 3.0 and 2.5V to minimize data lost and implement it in the main code.

void setup() {
  Serial.begin(9600);
}

void loop() {
  int val_1 = analogRead(A1);
  int val_2 = analogRead(A2);
  int val_3 = analogRead(A3);
  Serial.print("Val 1, 2.5V: " + String(val_1));
  Serial.print(" Val 2, 3.3V: " + String(val_2));
  Serial.println("");
}
