
#define LINE_TRACKING_SENSOR_L	(26)
#define LINE_TRACKING_SENSOR_C	(17)
#define LINE_TRACKING_SENSOR_R	(39)


void setup() 
{
	Serial.begin(115200);

	pinMode(LINE_TRACKING_SENSOR_L, INPUT);
	pinMode(LINE_TRACKING_SENSOR_C, INPUT);
	pinMode(LINE_TRACKING_SENSOR_R, INPUT);

}

void loop()
{
	int sensor_L = digitalRead(LINE_TRACKING_SENSOR_L);
	int sensor_C = digitalRead(LINE_TRACKING_SENSOR_C);
	int sensor_R = digitalRead(LINE_TRACKING_SENSOR_R);

	Serial.print(sensor_L);
	Serial.print("    ");
	Serial.print(sensor_C);
	Serial.print("    ");
	Serial.print(sensor_R);
	Serial.print("    ");
	Serial.println();
	delay(1000);
}