const int LED_PIN = 2; // PWMèoóÕÇ∑ÇÈPin No.
// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT 13
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
 // calculate duty, 8191 from 2 ^ 13 - 1
 uint32_t duty = (8191 / valueMax) * min(value, valueMax);

// write duty to LEDC
 ledcWrite(channel, duty);
}

void setup() {
 // Setup timer and attach timer to a led pin
 ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
 ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
}

void loop(){
  for(int i=0; i<256; i++){
    ledcAnalogWrite(LEDC_CHANNEL_0, i);
    delay(100); // change delay time can breath faster or slower
   }
}


/*
void setup() {
  pinMode(2, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because 
                                    // it is acive low on the ESP-01)
  delay(2000);                      // Wait for a second
  digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
  delay(1000);                      // Wait for two seconds (to demonstrate the active low LED)
}

*/