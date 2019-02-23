// MPU-6050 Accelerometer + Gyro

void MPU6050_init(TwoWire* pWire);

int MPU6050_get_all(
		float* p_acc_x,
		float* p_acc_y,
		float* p_acc_z,
		float* p_gyro_x,
		float* p_gyro_y,
		float* p_gyro_z,
		float* p_temperature);


