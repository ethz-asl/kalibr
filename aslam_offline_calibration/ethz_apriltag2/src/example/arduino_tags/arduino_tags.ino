/*
  ArduinoTag
  Turns on an LED whenever Raspberry Pi sees an Apriltag.
 
  Michael Kaess 04/13
 */

// pin number for LED
int led = 9;

// information about detected tag
int tagId;
float x, y, z;

// runs once when the program starts
void setup() {
  // open serial port
  Serial.begin(115200);
  // initialize pin as output
  pinMode(led, OUTPUT);     
}

// runs over and over again
void loop() {
  // check if new data is available
  if (Serial.available() > 0) {
    tagId = Serial.parseInt();
    x = Serial.parseFloat();
    y = Serial.parseFloat();
    z = Serial.parseFloat();
    Serial.read(); // ignore newline character
    if (tagId >= 0) {
      digitalWrite(led, HIGH);
    } else {
      digitalWrite(led, LOW);
    }
  }
  // wait for 20ms before checking again for new data
  delay(20);
}
