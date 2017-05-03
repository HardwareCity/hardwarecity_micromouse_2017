// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PIN  8 //12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     9 //11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(230400); // Open serial monitor at 115200 baud to see ping results.
}

// Send string by serial chanel, of n values.  
void graph(int val1, int val2, int val3){
  Serial.print(val1);
  Serial.print(",");
  Serial.print(val2);
  Serial.print(",");
  Serial.println(val3);
}

void loop() {
  graph(sonar.ping_cm(), 0, 0); // Send ping, get distance in cm and print result (0 = outside set distance range)
}
