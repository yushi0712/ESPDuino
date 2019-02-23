// MPU-6050 Accelerometer + Gyro

// I2CにアクセスするためにWireライブラリを使用
#include <Wire.h>

#include "dev_MPU6050.h"


// I2C
#define IO_PIN_SDA 				SDA
#define IO_PIN_SCL 				SCL

// LED
#define	IO_PIN_LED				(18)


// デバイス初期化時に実行される
void setup() 
{
	// LED
	pinMode(IO_PIN_LED, OUTPUT);

	// ボーレートを115200bpsにセット
	Serial.begin(115200);

	// I2C初期化	
	Wire.begin(IO_PIN_SDA, IO_PIN_SCL);

	MPU6050_init(&Wire);

}

float pre_acc = 0;
void loop() 
{
	int error;
	float	acc_x, acc_y, acc_z;
	float	gyro_x, gyro_y, gyro_z;
	float	temperature;

	// 加速度、角速度、温度を取得
	error = MPU6050_get_all(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z, &temperature);

	Serial.print(error, DEC);
	Serial.print("\t");

	Serial.print(temperature, 1);
	Serial.print("\t");

	float abs_acc2 = (acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
	float diff_acc = abs_acc2 - pre_acc;
	if(diff_acc < 0) {
		diff_acc = -diff_acc;
	}
	pre_acc = abs_acc2;
	Serial.print(diff_acc, 2);
	Serial.print("\t");
	Serial.println("");

	if(diff_acc < 0.2) {
		digitalWrite(IO_PIN_LED,LOW);
	}
	else {
		digitalWrite(IO_PIN_LED,HIGH);
	}

	delay(50);
}

