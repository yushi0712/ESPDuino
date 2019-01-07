#define READ_START 10
#define READ_END 30

void setup() {

	int pin=0;
	for(pin=READ_START; pin <= READ_END; pin++) {
		pinMode(pin, INPUT);     // Initialize the LED_BUILTIN pin as an output
	}

	Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
	int pin=0;
	int rd;
	for(pin=READ_START; pin <= READ_END; pin++) {
		rd = digitalRead(pin);
		Serial.printf("[%d] %d \n",pin, rd);
		
	}
	  delay(2000);                      // Wait for a second
}

