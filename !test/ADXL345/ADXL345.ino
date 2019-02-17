// MPU-6050 Accelerometer + Gyro

// I2C�ɃA�N�Z�X���邽�߂�Wire���C�u�������g�p
#include <Wire.h>

// I2C
#define IO_PIN_SDA 				SDA
#define IO_PIN_SCL 				SCL

// ���W�X�^�A�h���X
#define MPU6050_ACCEL_XOUT_H 0x3B  // R  
#define MPU6050_WHO_AM_I     0x75  // R
#define MPU6050_PWR_MGMT_1   0x6B  // R/W
#define MPU6050_I2C_ADDRESS  0x68

#define SWAP(x,y) swap = x; x = y; y = swap

// �\���̒�`
typedef union accel_t_gyro_union {
	struct {
		uint8_t x_accel_h;
		uint8_t x_accel_l;
		uint8_t y_accel_h;
		uint8_t y_accel_l;
		uint8_t z_accel_h;
		uint8_t z_accel_l;
		uint8_t t_h;
		uint8_t t_l;
		uint8_t x_gyro_h;
		uint8_t x_gyro_l;
		uint8_t y_gyro_h;
		uint8_t y_gyro_l;
		uint8_t z_gyro_h;
		uint8_t z_gyro_l;
	}
	reg;
	struct {
		int16_t x_accel;
		int16_t y_accel;
		int16_t z_accel;
		int16_t temperature;
		int16_t x_gyro;
		int16_t y_gyro;
		int16_t z_gyro;
	}
	value;
};

TwoWire* g_pWire = NULL;


void ADXL345_init(TwoWire* pWire)
{
	int error;
	uint8_t c;

	g_pWire = pWire;

	// ����̓ǂݏo��
	error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
	Serial.print("WHO_AM_I : ");
	Serial.print(c, HEX);
	Serial.print(", error = ");
	Serial.println(error, DEC);

	// ���샂�[�h�̓ǂݏo��
	error = MPU6050_read(MPU6050_PWR_MGMT_1, &c, 1);
	Serial.print("PWR_MGMT_1 : ");
	Serial.print(c, HEX);
	Serial.print(", error = ");
	Serial.println(error, DEC);

	// MPU6050����J�n
	MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0);
}

// �f�o�C�X���������Ɏ��s�����
void setup() 
{
	// LED
	pinMode(13, OUTPUT);

	// �{�[���[�g��115200bps�ɃZ�b�g
	Serial.begin(9600);
	//  Serial.begin(115200);

	// I2C������	
	Wire.begin(IO_PIN_SDA, IO_PIN_SCL);

	ADXL345_init(&Wire);

}

float pre_acc = 0;
void loop() 
{
	int error;
	float	acc_x, acc_y, acc_z;
	float	gyro_x, gyro_y, gyro_z;
	float	temperature;

	// �����x�A�p���x�A���x���擾
	error = ADXL_get_all(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z, &temperature);

	Serial.print(error, DEC);
	Serial.print("\t");

	Serial.print(temperature, 1);
	Serial.print("\t");

	float abs_acc2 = (acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
	float diff_acc = abs_acc2-pre_acc;
	pre_acc = abs_acc2;
	Serial.print(diff_acc, 2);
	Serial.print("\t");
	Serial.println("");

	if(diff_acc < 0.5) {
		digitalWrite(13,LOW);
	}
	else {
		digitalWrite(13,HIGH);
	}

	delay(500);
}

int ADXL_get_all(
		float* p_acc_x,
		float* p_acc_y,
		float* p_acc_z,
		float* p_gyro_x,
		float* p_gyro_y,
		float* p_gyro_z,
		float* p_temperature)
{
	int error;
	accel_t_gyro_union accel_t_gyro;

	// �����x�A�p���x�̓ǂݏo��
	// accel_t_gyro�͓ǂݏo�����l��ۑ�����\���́A���̌��̈����͎��o���o�C�g��
	error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel_t_gyro, sizeof(accel_t_gyro));

	// �擾�ł���f�[�^�̓r�b�O�G���f�B�A���Ȃ̂ŏ�ʃo�C�g�Ɖ��ʃo�C�g�̓���ւ��iAVR�̓��g���G���f�B�A���j
	uint8_t swap;
	SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
	SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
	SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
	SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

	// �擾���������x�l�𕪉�\�Ŋ����ĉ����x(G)�ɕϊ�����
	*p_acc_x = accel_t_gyro.value.x_accel / 16384.0; //FS_SEL_0 16,384 LSB / g
	*p_acc_y = accel_t_gyro.value.y_accel / 16384.0;
	*p_acc_z = accel_t_gyro.value.z_accel / 16384.0;

	// �擾�����p���x�l�𕪉�\�Ŋ����Ċp���x(degrees per sec)�ɕϊ�����
	*p_gyro_x = accel_t_gyro.value.x_gyro / 131.0;//FS_SEL_0 131 LSB / (��/s)
	*p_gyro_y = accel_t_gyro.value.y_gyro / 131.0;
	*p_gyro_z = accel_t_gyro.value.z_gyro / 131.0;

	// ���x�̌v�Z�B���̓��W�X�^�}�b�v�ɍڂ��Ă܂��B���̎��������������c�B
	*p_temperature = ( (float) accel_t_gyro.value.temperature + 12412.0) / 340.0;

	return	error;
}

// MPU6050_read
int MPU6050_read(int start, uint8_t *buffer, int size) 
{
	int i, n, error;

	if(g_pWire == NULL) {
		return	-99;
	}

	g_pWire->beginTransmission(MPU6050_I2C_ADDRESS);
	n = g_pWire->write(start);
	if(n != 1) {
		return (-10);
	}
	n = g_pWire->endTransmission(false);// hold the I2C-bus
	if(n != 0) {
		return (n);
	}
	// Third parameter is true: relase I2C-bus after data is read.
	g_pWire->requestFrom(MPU6050_I2C_ADDRESS, size, true);
	i = 0;
	while(g_pWire->available() && i < size) {
		buffer[i++] = g_pWire->read();
	}
	if( i != size) {
		return (-11);
	}
	
	return (0); // return : no error
}

// MPU6050_write
int MPU6050_write(int start, const uint8_t *pData, int size) 
{
	int n, error;

	if(g_pWire == NULL) {
		return	-99;
	}

	g_pWire->beginTransmission(MPU6050_I2C_ADDRESS);
	n = g_pWire->write(start);// write the start address
	if(n != 1) {
		return (-20);
	}
	n = g_pWire->write(pData, size);// write data bytes
	if(n != size) {
		return (-21);
	}
	error = g_pWire->endTransmission(true); // release the I2C-bus
	if(error != 0) {
		return (error);
	}

	return (0);// return : no error
}

// MPU6050_write_reg
int MPU6050_write_reg(int reg, uint8_t data) 
{
	int error;
	error = MPU6050_write(reg, &data, 1);
	Serial.print("error = ");
	Serial.println(error);
	return (error);
};
