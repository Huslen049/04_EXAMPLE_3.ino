// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12    // sonar sensor TRIGGER
#define PIN_ECHO 13    // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0      // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25        // sampling interval (unit: msec) // MODIFIED: Changed from 100
#define PULSE_DURATION 10  // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0    // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0    // maximum distance to be measured (unit: mm)

// Constants for LED Brightness Calculation
#define DIST_CENTER 200.0 // Center distance for max brightness (mm)
// PWM_SLOPE = (255.0 / (DIST_CENTER - _DIST_MIN)) = 255.0 / 100.0
#define PWM_SLOPE 2.55 

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW); // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  // 1. Read Distance
  float distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // 2. LED Brightness Control (Proportional to Distance)
  int brightness; // PWM value (0: max bright, 255: off)

  if (distance >= _DIST_MIN && distance <= _DIST_MAX) {
    // In desired Range (100mm <= distance <= 300mm)
    // Formula: PWM = PWM_SLOPE * |distance - DIST_CENTER|
    
    // Calculate difference from center
    float dist_diff = abs(distance - DIST_CENTER); 
    
    // Calculate the floating point PWM value
    float pwm_float = PWM_SLOPE * dist_diff;
    
    // Constrain the value between 0 and 255 and cast to integer.
    // round() is used for accurate conversion (e.g., 127.5 -> 128)
    brightness = (int)round(constrain(pwm_float, 0.0, 255.0));
  } else {    
    // Outside desired Range (< 100mm or > 300mm or error)
    // LED OFF (255)
    brightness = 255;
    
    // Original code logic to adjust distance for serial output (Kept for consistency)
    if ((distance == 0.0) || (distance > _DIST_MAX)) {
        distance = _DIST_MAX + 10.0;    // Set Higher Value
    } else if (distance < _DIST_MIN) {
        distance = _DIST_MIN - 10.0;    // Set Lower Value
    }
  }

  // Apply the calculated brightness using analogWrite (PWM)
  analogWrite(PIN_LED, brightness);

  // output the distance to the serial port
  Serial.print("Min:");      Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");
  
  // do something here
  // delay(50); // REMOVED as per instruction

  // wait until next sampling time.
  delay(INTERVAL);
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
