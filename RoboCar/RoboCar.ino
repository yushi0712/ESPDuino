#include "BluetoothSerial.h"
#include "esp_system.h"

// Motor
#define	IO_PIN_MOTOR_1			(14)
#define	IO_PIN_MOTOR_2			(12)
#define	IO_PIN_MOTOR_3			(13)
#define	IO_PIN_MOTOR_4			(23)
#define	IO_PIN_MOTOR_ENA		(16)
#define	IO_PIN_MOTOR_ENB		(27)
// LED
#define	IO_PIN_LED				(18)
// US Sendor
#define	IO_PIN_US_ECHO			(36)
#define	IO_PIN_US_TRIG			( 5)
// Servo
#define	IO_PIN_SERVO			(25)
// ê‘äOê¸
#define IO_PIN_INFRARED			(19)
// Line Tracking Sensor
#define IO_PIN_LINETRACK_LEFT	(26)
#define IO_PIN_LINETRACK_CENTER	(17)
#define IO_PIN_LINETRACK_RIGHT	(39)

// DACÇÃchannel
#define DAC_CH_MOTOR_A			(0)
#define DAC_CH_MOTOR_B			(1)
#define DAC_CH_SERVO			(2)


#define	MOTOR_RIGHT_FRONT		0x01
#define	MOTOR_RIGHT_REAR		0x02
#define	MOTOR_LEFT_FRONT		0x04
#define	MOTOR_LEFT_REAR			0x08

#define	MOTOR_RIGHT				(MOTOR_RIGHT_FRONT|MOTOR_RIGHT_REAR)
#define	MOTOR_LEFT				(MOTOR_LEFT_FRONT|MOTOR_LEFT_REAR)
#define	MOTOR_FRONT				(MOTOR_RIGHT_FRONT|MOTOR_LEFT_FRONT)
#define	MOTOR_REAR				(MOTOR_RIGHT_REAR|MOTOR_LEFT_REAR)

#define	MOTOR_SPEED				170	// 1-256

#define	MOTOR_DIR_STOP				(0)
#define	MOTOR_DIR_FWD				(1)
#define	MOTOR_DIR_REV				(2)

// state of motor
#define	STATE_MOTOR_STOP			(0)
#define	STATE_MOTOR_MOVING_FORWARD	(1)
#define	STATE_MOTOR_MOVING_BACKWARD	(2)
#define	STATE_MOTOR_TURNING_RIGHT	(3)
#define	STATE_MOTOR_TURNING_LEFT	(4)
#define	STATE_MOTOR_ROTATING_CW		(5)
#define	STATE_MOTOR_ROTATING_CCW	(6)

#define	CTRLMODE_AUTO_DRIVE			(0)
#define	CTRLMODE_MANUAL_DRIVE		(1)
#define	CTRLMODE_LINE_TRACKING		(2)



//BluetoothSerial SerialBT;


int	g_state_motor = STATE_MOTOR_STOP;
int g_motor_speed_right;
int g_motor_speed_left;
int	g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;

float servo_coeff_a;
float servo_coeff_b;


void _servo_angle(int angle)
{
	if(angle < -90.0) {
		angle = -90.0;
	}
	if(angle > 90.0) {
		angle = 90.0;
	}

	int val = (int)(servo_coeff_a*angle + servo_coeff_b);
	ledcWrite(DAC_CH_SERVO, val);
	delay(500);
	ledcWrite(DAC_CH_SERVO, 0);	
}

int _us_get_distance()   
{
	digitalWrite(IO_PIN_US_TRIG, LOW);   
	delayMicroseconds(20);
	digitalWrite(IO_PIN_US_TRIG, HIGH);  
	delayMicroseconds(20);
	digitalWrite(IO_PIN_US_TRIG, LOW);   
	float Fdistance = pulseIn(IO_PIN_US_ECHO, HIGH);  
	Fdistance= Fdistance/58;       

	return (int)Fdistance;
}  

void _ctrl_motor_left(int dir, int speed)
{
	ledcWrite(DAC_CH_MOTOR_A, speed);  
	g_motor_speed_right = speed;

	if(dir == MOTOR_DIR_FWD) {
		digitalWrite(IO_PIN_MOTOR_1,HIGH);
		digitalWrite(IO_PIN_MOTOR_2,LOW);
	}
	else if(dir == MOTOR_DIR_REV) {
		digitalWrite(IO_PIN_MOTOR_1,LOW);
		digitalWrite(IO_PIN_MOTOR_2,HIGH);
	}
}

void _ctrl_motor_right(int dir, int speed)
{
	ledcWrite(DAC_CH_MOTOR_B, speed);  
	g_motor_speed_left = speed;

	if(dir == MOTOR_DIR_FWD) {
		digitalWrite(IO_PIN_MOTOR_3,LOW);
		digitalWrite(IO_PIN_MOTOR_4,HIGH);
	}
	else if(dir == MOTOR_DIR_REV) {
		digitalWrite(IO_PIN_MOTOR_3,HIGH);
		digitalWrite(IO_PIN_MOTOR_4,LOW);
	}
}

void _move_forward(int speed)
{
	_ctrl_motor_right(MOTOR_DIR_FWD, speed);
	_ctrl_motor_left(MOTOR_DIR_FWD, speed);
	
	g_state_motor = STATE_MOTOR_MOVING_FORWARD;
	
	Serial.println("go forward");
}

void _move_backward(int speed)
{
	_ctrl_motor_right(MOTOR_DIR_REV, speed);
	_ctrl_motor_left(MOTOR_DIR_REV, speed);
	
	g_state_motor = STATE_MOTOR_MOVING_BACKWARD;
	
	Serial.println("go backward");
}

void _turn_left(int dir, int speed, int level)
{
	int speed_right = speed + level;
	int speed_left = speed - level;
	if(speed_left < 0) {
		speed_left = 0;
	}
		
	_ctrl_motor_right(dir, speed_right);
	_ctrl_motor_left(dir, speed_left);

	g_state_motor = STATE_MOTOR_TURNING_LEFT;
	
	Serial.println("turn left!");
}

void _turn_right(int dir, int speed, int level)
{
	int speed_right = speed - level;
	int speed_left = speed + level;
	if(speed_right < 0) {
		speed_right = 0;
	}
	_ctrl_motor_right(dir, speed_right);
	_ctrl_motor_left(dir, speed_left);
	
	g_state_motor = STATE_MOTOR_TURNING_RIGHT;
	
	Serial.println("turn right!");
}

void _rotate_ccw(int speed)
{
	_ctrl_motor_right(MOTOR_DIR_FWD, speed);
	_ctrl_motor_left(MOTOR_DIR_REV, speed);
	
	g_state_motor = STATE_MOTOR_ROTATING_CCW;
	
	Serial.println("rotate ccw!");
}

void _rotate_cw(int speed)
{
	_ctrl_motor_right(MOTOR_DIR_REV, speed);
	_ctrl_motor_left(MOTOR_DIR_FWD, speed);
	
	g_state_motor = STATE_MOTOR_ROTATING_CW;
	
	Serial.println("rotate cw!");
}

void _stop()
{
	_ctrl_motor_left(0, 0);
	_ctrl_motor_right(0, 0);

	g_state_motor = STATE_MOTOR_STOP;

	Serial.println("Stop!");
}

void setup()
{
	g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;

	Serial.begin(9600);
	//SerialBT.begin("ESP32");

	pinMode(IO_PIN_US_ECHO, INPUT);    
	pinMode(IO_PIN_US_TRIG, OUTPUT);  
	pinMode(IO_PIN_LED, OUTPUT);
	pinMode(IO_PIN_MOTOR_1,OUTPUT);
	pinMode(IO_PIN_MOTOR_2,OUTPUT);
	pinMode(IO_PIN_MOTOR_3,OUTPUT);
	pinMode(IO_PIN_MOTOR_4,OUTPUT);
	pinMode(IO_PIN_MOTOR_ENA,OUTPUT);
	pinMode(IO_PIN_MOTOR_ENB,OUTPUT);
	pinMode(IO_PIN_SERVO,OUTPUT);
	pinMode(IO_PIN_LINETRACK_LEFT, INPUT);    
	pinMode(IO_PIN_LINETRACK_CENTER, INPUT);    
	pinMode(IO_PIN_LINETRACK_RIGHT, INPUT);    

	// MotorÇÃèâä˙ê›íË
	ledcSetup(DAC_CH_MOTOR_A, 980, 8);
	ledcSetup(DAC_CH_MOTOR_B, 980, 8);
	ledcAttachPin(IO_PIN_MOTOR_ENA, DAC_CH_MOTOR_A);
	ledcAttachPin(IO_PIN_MOTOR_ENB, DAC_CH_MOTOR_B);

	_stop();

	// ServoÇÃèâä˙ê›íË
	float servo_min = 26.0;  // (26/1024)*20ms Å‡ 0.5 ms  (-90Åã)
	float servo_max = 123.0; // (123/1024)*20ms Å‡ 2.4 ms (+90Åã)
	servo_coeff_a = (servo_max-servo_min)/180.0;
	servo_coeff_b = (servo_max+servo_min)/2.0;
	ledcSetup(DAC_CH_SERVO, 50, 10);  // 0ch 50 Hz 10bit resolution
	ledcAttachPin(IO_PIN_SERVO, DAC_CH_SERVO); 

    _servo_angle(0);//********xxxxx setservo position according to scaled value
    delay(500); 

}

void loop()
{
	int dist = _us_get_distance();

	// Stop in case of obstacle
    if(dist <= 20) {
		if((g_state_motor==STATE_MOTOR_MOVING_FORWARD) || 
			(g_state_motor==STATE_MOTOR_TURNING_RIGHT) ||
			(g_state_motor==STATE_MOTOR_TURNING_LEFT)) 
		{
			_stop();
		}
	}
	char getstr = Serial.read();

	if(getstr == 'a') {
		g_ctrl_mode = CTRLMODE_AUTO_DRIVE;
		_stop();
	}
	else if(getstr == 'm') {
		g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;
		_stop();
	}

	if(g_ctrl_mode == CTRLMODE_MANUAL_DRIVE) {
		if(getstr=='f') {
			_move_forward(MOTOR_SPEED);
		}
		else if(getstr=='b') {
			_move_backward(MOTOR_SPEED);
			delay(200);
		}
		else if(getstr=='l') {
			_rotate_ccw(MOTOR_SPEED);
			delay(200);
		}
		else if(getstr=='r') {
			_rotate_cw(MOTOR_SPEED);
			delay(200);
		}
		else if(getstr=='L') {
			_turn_left(MOTOR_DIR_FWD, 120, 70);
			delay(200);
		}
		else if(getstr=='R') {
			_turn_right(MOTOR_DIR_FWD, 120, 70);
			delay(200);
		}
		else if(getstr=='C') {
			_turn_left(MOTOR_DIR_REV, 120, 70);
			delay(200);
		}
		else if(getstr=='D') {
			_turn_right(MOTOR_DIR_REV, 120, 70);
			delay(200);
		}
		else if(getstr=='s') {
			_stop();		 
		}
	}
	else if(g_ctrl_mode == CTRLMODE_AUTO_DRIVE) {
		int right_distance = 0, left_distance = 0, middle_distance = 0;

		if(getstr=='s') {
			g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;
			_stop();		 
		}		
		else {
			_servo_angle(0);//setservo position according to scaled value
			delay(500); 
			middle_distance = _us_get_distance();

			if(middle_distance<=30) {     
				_stop();
				delay(500); 	  
				_servo_angle(-80);  
				delay(1000);      
				right_distance = _us_get_distance();

				delay(500);
				_servo_angle(0);              
				delay(1000);                                                  
				_servo_angle(80);              
				delay(1000); 
				left_distance = _us_get_distance();

				delay(500);
				_servo_angle(0);              
				delay(1000);
				if(right_distance>left_distance) {
					_rotate_cw(150);
					delay(360);
				}
				else if(right_distance<left_distance) {
					_rotate_ccw(150);
					delay(360);
				}
				else if((right_distance<=30)||(left_distance<=30)) {
					_move_backward(MOTOR_SPEED);
					delay(180);
				}
				else {
					_move_forward(MOTOR_SPEED);
				}
			}  
			else {
				_move_forward(MOTOR_SPEED);
			}
		}
	}

}


