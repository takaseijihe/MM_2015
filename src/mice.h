#include "iodefine.h"
#include "stdarg.h"
#include "serial.h"

#include "common.h"
#include "drv.h"
#include "ctrl.h"
#include "binary.h"

#define KUKAKU 180
#define  motor_enable  PE.DRL.BIT.B8
#define  motor_driver_reset PE.DRL.BIT.B2
#define led_green PB.DR.BIT.B1
#define led_red PB.DR.BIT.B3
//#define taiyagaikei_left 51.650
//#define taiyagaikei_right 51.700
#define taiyagaikei_left 51.250
#define taiyagaikei_right 51.2
#define length_tread_right 83.0
#define length_tread_left  83.2
#define pai 3.1415
#define CWCCW_right PE.DRL.BIT.B0
#define CWCCW_left PE.DRL.BIT.B4
#define SW_YELLOW PE.DRL.BIT.B10
#define SW_BROWN PE.DRL.BIT.B12
#define SW_3 PE.DRL.BIT.B14
#define LED_LEFT PA.DRL.BIT.B12
#define LED_FRONTLEFT PA.DRL.BIT.B13
#define	LED_RIGHT PA.DRL.BIT.B14
#define	LED_FRONTRIGHT PA.DRL.BIT.B15
#define P_GAIN  0.2
#define DIFF_THRESHOLD_RIGHT 2
#define DIFF_THRESHOLD_LEFT 2
#define LED_R_TH 60//le26ri35le112ri100
#define LED_L_TH 55
#define LED_FR_TH 49//107
#define LED_FL_TH 77//424
#define LED_FR_REF 76
#define LED_FL_REF 148
#define LED_L_REF 20
#define LED_R_REF 27
#define DESTINATION_X 4
#define DESTINATION_Y 4
#define PREDISTANCE_SLALOM 5
#define FOLDISTANCE_SLALOM 5

volatile int direction_x = 0;
volatile int direction_y = 1;
volatile int coordinate_x = 0;
volatile int coordinate_y = 1;
unsigned char wall[16][16];
unsigned char map[16][16];
unsigned char hosumap_x[256];
unsigned char hosumap_y[256];
unsigned char vector_path[256];
volatile int count_map = 0;
volatile float led_r_th = 8;
volatile float led_l_th = 2;
volatile float led_fr_th = 47;
volatile float led_fl_th = 141;
extern volatile void init_all(void);
volatile int G_TimerCount;
void Mtr(void);
void Led_Twinkle(void);
void Led_init(void);
void SenRead(void);
void Spk(void);
void logger_sensor(void);

//1mSごとにカウントアップされる変数
void AD_BATTERY();
void q_walk_map_maker(char, char);
volatile void SEN_LEFT();
volatile void SEN_FRONTLEFT();
volatile void SEN_RIGHT();
volatile void SEN_FRONTRIGHT();
volatile void acceleration(int, int, int, int);
volatile void accel_velocity(int, int, int, int, int);
volatile void ConstVel(int, int);
volatile void deceleration(int, int, int, int);
volatile void wait(int a);
volatile void daikei();
volatile void coordinate(int);
volatile void store_map_wall();
volatile void draw_map();
volatile void test_map();
volatile void store_map_step();
volatile void adachiho();
volatile void tyousinti_test();
volatile void circuit(int);
volatile void map_path();
volatile void saitan();
volatile void slalom_right(float,float,float,float);
volatile void slalom_left(float, float, float, float);
void P_control();
volatile void hidariteho(void);
volatile void hidariteho_slalom(void);

void logger_sensor(void);
void print_log_sensor(void);

unsigned volatile long long i = 50000;
unsigned volatile int log_count = 0;
unsigned volatile int SEN_left;
unsigned volatile int SEN_frontleft;
unsigned volatile int SEN_right;
unsigned volatile int SEN_frontright;
unsigned volatile int BAT;
unsigned volatile int Battery;
unsigned volatile int count = 0;
unsigned volatile int tgra_right;
unsigned volatile int tgra_left;
volatile char j, k, l, m;

static unsigned volatile int SENleft_off;
static unsigned volatile int SENfrontleft_off;
static unsigned volatile int SENright_off;
static unsigned volatile int SENfrontright_off;
static unsigned volatile int SENleft_offset;
static unsigned volatile int SENfrontleft_offset;
static unsigned volatile int SENright_offset;
static unsigned volatile int SENfrontright_offset;
static unsigned volatile int P_control_flag = 0;
static unsigned int map_flag = 0;
static unsigned int breakable_flag = 0;

static volatile int log_sen_right[200];
static volatile int log_sen_frontright[200];
static volatile int log_sen_frontleft[200];
static volatile int log_sen_left[200];
unsigned volatile char path[256];

volatile float control;
volatile float sen_left_difference;
volatile float sen_right_difference;
volatile float sen_left_old;
volatile float sen_right_old;
static volatile int error = 0;
volatile float tmp_sen_l_th;
volatile float tmp_sen_r_th;
volatile float direction_number[100];

volatile float accelerate = 1000.0;
volatile float velocity = 150.0;
volatile float accelerate_angle = 10.0;
volatile float velocity_angle = 10.0;
volatile float velocity_right;
volatile float velocity_left;
volatile float theta = 10.0;
volatile float distance = 0.0;
volatile float distance_sankaku;
volatile float distance_accelerate;
volatile float distance_decelerate;
volatile float tmp_accelerate;

volatile float velocity_min;
volatile float velocity_max;
volatile float velocity_term;
volatile float velocity_init;

volatile float distance_max;
volatile float angle_body = 0;
volatile float angle_velocity = 0;
volatile float angle_accelerate = 0;
volatile float angle_decelerate = 0;
volatile float angle_max;
volatile float steplength_left = 0.9 * (pai / 180.0) * (taiyagaikei_left / 2.0);
volatile float steplength_right = 0.9 * (pai / 180.0)
		* (taiyagaikei_right / 2.0);
unsigned volatile int T = 0;
unsigned volatile char sw_1 = 0x00;
unsigned volatile char sw_2 = b00000000;
unsigned volatile char sw_3 = b00000000;

volatile void interrupt_cmt0() //割り込み関数
{

	CMT0.CMCSR.BIT.CMF = 0; //フラグクリア
	count++;
	T++;
	if (sw_1 == 0xff) {
		daikei();
	}
	SenRead();
	if (P_control_flag != 1) {

		P_control();
	}
	if (SW_3 == 0) {

		breakable_flag = 1;
		motor_enable = 0;
	}
	angle_velocity = angle_velocity +angle_accelerate * 0.001;
	angle_body = angle_body + angle_velocity *0.001;
	velocity = velocity + accelerate * 0.001 ;
	distance = distance + velocity * 0.001;

	velocity_left = velocity - control - (angle_velocity*length_tread_left*pai)/360;
	velocity_right = velocity + control + (angle_velocity*length_tread_left*pai)/360;
	tgra_left = (int) (steplength_left * (6250000) / velocity_left);
	tgra_right = (int) (steplength_right * (6250000) / velocity_right);

}
volatile void input_daikei(int vel_min, int vel_max, int dis_max, int accel) {
	velocity_min = vel_min;
	velocity_max = vel_max;
	distance_max = dis_max;
	accelerate = accel;
}
volatile void turn(int vel, int vel_angle, int angle) {
	velocity = vel;
	velocity_angle = vel_angle;
	velocity_right = length_tread_right * velocity_angle + velocity;
	velocity_left = velocity - length_tread_right * velocity_angle;
	CWCCW_right = 1;
	CWCCW_left = 0;
	MTU2.TSTR.BYTE = 1;
	while (velocity_angle * 0.001 * T < angle)
		;
	MTU2.TSTR.BYTE = 0;
	T = 0;
}

volatile void tyousinti_right(int vel_min, int vel_max, float angle, int accel) {
	P_control_flag = 1;
	CWCCW_right = 0;
	CWCCW_left = 0;
	distance = 0;
	velocity_min = vel_min;
	velocity_max = vel_max;
	angle_max = angle * pai / 180;
	distance_max = length_tread_right * angle_max / 2;
	accelerate = accel;
	velocity = velocity_min;
	tmp_accelerate = accelerate;
	distance_sankaku = (velocity_max * velocity_max
			- velocity_min * velocity_min) / (2 * accelerate);
	//distance_sankaku跡地

	//myprintf("distance_max=%f\t", distance_max);
	while (1) {

		//velocity_right = velocity + angle_velocity*T*0.001;
		//velocity_left = velocity - angle_velocity*T*0.001;
		//myprintf("distance_sankaku=%f\t", distance_sankaku);
		if (distance >= distance_sankaku
				&& distance <= (distance_max - distance_sankaku)) {
			accelerate = 0.0;
			velocity = velocity_max;
		}
		if (distance <= distance_max
				&& distance >= (distance_max - distance_sankaku)) {
			accelerate = (-tmp_accelerate);
		}
		if ((distance >= distance_max) || (velocity < 150)) {
			//myprintf("distancejissai_max=%f\t", distance);
			velocity = velocity_min;
			accelerate = 0.0;

			distance = 0.0;

			P_control_flag = 0;
			break;
		}
	}

}

void logger_sensor(void) {
	log_sen_right[log_count] = SENright_offset;
	log_sen_frontright[log_count] = SENfrontright_offset;
	log_sen_left[log_count] = SENleft_offset;
	log_sen_frontleft[log_count] = SENfrontleft_offset;
	log_count++;
}
void print_log_sensor(void) {
	for (i = 0; i < 1000; i++) {
	}
	for (log_count = 0; log_count < 200; log_count++) {
		myprintf("right sensor offset:%d\t", log_sen_right[log_count]);
		myprintf("front right sensor offset:%d\t",
				log_sen_frontright[log_count]);
		myprintf("left sensor offset:%d\t", log_sen_left[log_count]);
		myprintf("front left sensor offset:%d\t", log_sen_frontleft[log_count]);
		myprintf("1:left turn,2:right turn,3:turn %d\n\r",
				direction_number[log_count]);
	}
}

volatile void tyousinti_left(int vel_min, int vel_max, float angle, int accel) {
	P_control_flag = 1;
	CWCCW_right = 1;
	CWCCW_left = 1;
	distance = 0;
	velocity_min = vel_min;
	velocity_max = vel_max;
	angle_max = angle * pai / 180;
	distance_max = length_tread_left * angle_max / 2;
	accelerate = accel;
	velocity = velocity_min;
	tmp_accelerate = accelerate;
	distance_sankaku = (velocity_max * velocity_max
			- velocity_min * velocity_min) / (2 * accelerate);
	//distance_sankaku跡地

	while (1) {

		//velocity_right = velocity + angle_velocity*T*0.001;
		//velocity_left = velocity - angle_velocity*T*0.001;
		if (distance >= distance_sankaku
				&& distance <= (distance_max - distance_sankaku)) {

			accelerate = 0.0;
			velocity = velocity_max;
		}
		if (distance <= distance_max
				&& distance >= (distance_max - distance_sankaku)) {
			accelerate = (-tmp_accelerate);

		}
		if ((distance >= distance_max) || (velocity < 150)) {
			velocity = velocity_min;
			accelerate = 0.0;
			distance = 0.0;

			P_control_flag = 0;
			break;
		}
	}

}
volatile void acceleration(int vel_min, int vel_max, int dis_max, int accel) {
	CWCCW_right = 0;
	CWCCW_left = 1;

	distance = 0;
	velocity_min = vel_min;
	velocity_max = vel_max;
	distance_max = dis_max;
	accelerate = accel;
	velocity = velocity_min;

	tmp_accelerate = accelerate;
	distance_sankaku = (velocity_max * velocity_max
			- velocity_min * velocity_min) / (2 * accelerate);
	//distance_sankaku跡地

	while (1) {

		//velocity_right = velocity + angle_velocity*T*0.001;
		//velocity_left = velocity - angle_velocity*T*0.001;

		if (distance >= distance_sankaku && distance < distance_max) {

			accelerate = 0.0;
			velocity = velocity_max;

		}

		if (distance >= distance_max) {
			accelerate = 0.0;
			distance = 0.0;
			break;
		}
	}

}
volatile void ConstVel(int constant_velocity, int dis_max) {
	CWCCW_right = 0;
	CWCCW_left = 1;

	distance = 0;
	velocity_max = constant_velocity;
	distance_max = dis_max;
	while (1) {

		if (distance < distance_max) {

			accelerate = 0.0;
			velocity = velocity_max;

		}

		if (distance >= distance_max) {

			accelerate = 0.0;
			distance = 0.0;
			break;

		}
	}

}
volatile void deceleration(int vel_min, int vel_max, int dis_max, int accel) {
	CWCCW_right = 0;
	CWCCW_left = 1;

	distance = 0;
	velocity_min = vel_min;
	velocity_max = vel_max;
	distance_max = dis_max;
	velocity = velocity_min;

	tmp_accelerate = accelerate;
	distance_sankaku = (velocity_max * velocity_max
			- velocity_min * velocity_min) / (2 * accelerate);

	while (1) {
		if (distance <= (distance_max - distance_sankaku)) {
			accelerate = 0.0;
			velocity = velocity_max;
		}
		if (distance <= (distance_max - distance_sankaku)) {
			accelerate = -accel;
		}
		if ((distance >= distance_max) || (velocity < 150)) {
			velocity = velocity_min;
			accelerate = 0.0;
			distance = 0.0;
			break;
		}

	}

}
volatile void accel_velocity(int vel_init, int vel_term, int vel_max,
		int dis_max, int accel) {
	CWCCW_right = 0;
	CWCCW_left = 1;
	P_control_flag = 0;
	distance = 0;

	velocity_init = vel_init;
	velocity_max = vel_max;
	velocity_term = vel_term;
	distance_max = dis_max;
	accelerate = accel;
	velocity = velocity_min;
	//v*
	tmp_accelerate = accelerate;
	distance_accelerate = (velocity_max * velocity_max
			- velocity_init * velocity_init) / (2 * accelerate);
	distance_decelerate = (velocity_max * velocity_max
			- velocity_term * velocity_term) / (2 * accelerate);
	//distance_sankaku跡地

	//velocity_right = velocity + angle_velocity*T*0.001;
	//velocity_left = velocity - angle_velocity*T*0.001;
	if ((distance_accelerate + distance_decelerate) < distance_max) {
		while (1) {
			if (distance < distance_accelerate) {
				accelerate = accel;
			}
			if (distance >= distance_accelerate
					&& distance <= (distance_max - distance_decelerate)) {

				accelerate = 0.0;
				velocity = velocity_max;

			}
			if (distance <= distance_max
					&& distance >= (distance_max - distance_decelerate)) {
				accelerate = (-tmp_accelerate);

			}
			if ((distance >= distance_max) || (velocity < 150)) {
				velocity = velocity_min;
				accelerate = 0.0;
				distance = 0.0;

				break;
			}
		}
	}
	if ((distance_accelerate + distance_decelerate) >= distance_max) {
		distance_accelerate = ((-velocity_init * velocity_init
				+ velocity_term * velocity_term + 2 * accelerate * distance_max)
				/ (4 * accelerate));
		distance_accelerate = ((velocity_init * velocity_init
				- velocity_term * velocity_term + 2 * accelerate * distance_max)
				/ (4 * accelerate));
		while (1) {
			if (distance < distance_accelerate) {
				accelerate = accel;
			}
			if (distance <= distance_max
					&& distance >= (distance_max - distance_decelerate)) {
				accelerate = (-tmp_accelerate);

			}
			if ((distance >= distance_max) || (velocity < 150)) {
				velocity = velocity_min;
				accelerate = 0.0;
				distance = 0.0;

				break;
			}
		}

	}
}

volatile void daikei(int vel_min, int vel_max, int dis_max, int accel) {
	CWCCW_right = 0;
	CWCCW_left = 1;

	distance = 0;
	velocity_min = vel_min;
	velocity_max = vel_max;
	distance_max = dis_max;
	accelerate = accel;
	velocity = velocity_min;

	tmp_accelerate = accelerate;
	distance_sankaku = (velocity_max * velocity_max
			- velocity_min * velocity_min) / (2 * accelerate);
	//distance_sankaku跡地

	while (1) {

		//velocity_right = velocity + angle_velocity*T*0.001;
		//velocity_left = velocity - angle_velocity*T*0.001;

		if (distance >= distance_sankaku
				&& distance <= (distance_max - distance_sankaku)) {

			accelerate = 0.0;
			velocity = velocity_max;

		}
		if (distance <= distance_max
				&& distance >= (distance_max - distance_sankaku)) {
			accelerate = (-tmp_accelerate);

		}
		if ((distance >= distance_max) || (velocity < 150)) {
			velocity = velocity_min;
			accelerate = 0.0;
			distance = 0.0;

			break;
		}
	}

}
volatile void circuit(int kukaku) {
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//yoko1
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//tate1
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//yoko2
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//tate2
	tyousinti_right(150, 500, 2700, 1000);
	/*1週目*/
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//yoko
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//tate
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//yoko
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//tate
	tyousinti_right(150, 500, 90, 1000);
	/*2週目*/
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//yoko
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//tate
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//yoko
	tyousinti_right(150, 500, 90, 1000);
	daikei(150, 2000, KUKAKU * kukaku, 1000);		//tate
	tyousinti_right(150, 500, 90, 1000);
	/*3週目*/
	daikei(150, 2000, KUKAKU * kukaku, 1000);
	tyousinti_right(150, 500, 90, 1000);
}
volatile void wait_ms(int ms) {
	count = 0;

	while (count < ms) {

	}
	//henko
}
volatile void interrupt_motor_right(void) {
	MTU20.TSR.BIT.TGFB = 0;
	MTU20.TGRA = tgra_right;

}
volatile void interrupt_motor_left(void) {
	MTU21.TSR.BIT.TGFB = 0;
	//led_red = 1;
	//led_green = 1;
	MTU21.TGRA = tgra_left;
}
volatile void saitan() {
	int yoko, tate, count, count2, count3;
	count2 = 0;
	count = 150;
	while (path[count2] != 0) {
		if (path[count2] <= 15) {
			accel_velocity(count, 500, 2000, (180 * path[count2]), 2000);
		} else if (path[count2] == 20) {
			accel_velocity(500, 150, 500, 180, 2000);

			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作

			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_right(150, 500, 90, 2000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作

			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			accel_velocity(150, 500, 500, 180, 2000);
			count = 150;
		} else if (path[count2] == 30) {
			accel_velocity(500, 150, 500, 180, 2000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作

			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_left(150, 500, 90, 2000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作

			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			accel_velocity(150, 500, 500, 180, 2000);

		}
		count2++;
	}
}
volatile void map_path(void) {
	int yoko, tate, count, count2, count3;

	vector_path[0] = 0;
	yoko = 0;
	tate = 0;
	count = 1;
	while (yoko != DESTINATION_X || tate != DESTINATION_Y) {
		if (((wall[yoko][tate + 1] & 0x10) == 0x10)
				&& (map[yoko][tate + 1] == map[yoko][tate] - 1)) { //北壁がなくかつ歩数マップ-1

			vector_path[count] = 0;

			count++;
			tate++;

		} else if (((wall[yoko][tate - 1] & 0x10) == 0x10)
				&& (map[yoko][tate - 1] == map[yoko][tate] - 1) && (tate > 0)) { //南

			vector_path[count] = 1;

			count++;
			tate--;

		} else if (((wall[yoko - 1][tate] & 0x10) == 0x10)
				&& (map[yoko - 1][tate] == map[yoko][tate] - 1) && (yoko > 0)) { //西
			vector_path[count] = 2;
			count++;
			yoko--;
		} else if (((wall[yoko + 1][tate] & 0x10) == 0x10)
				&& (map[yoko + 1][tate] == map[yoko][tate] - 1)) { //東
			vector_path[count] = 3;
			count++;
			yoko++;
		}
	}
	count = 0;
	count2 = 0;
	count3 = 0;
	while (1) {
		myprintf("houkou=%d\n\r", vector_path[count]);
		if ((vector_path[count2] != 0) && (vector_path[count2] != 1)
				&& (vector_path[count2] != 2) && (vector_path[count2] != 3)) {
			break;
		}
		if (vector_path[count2 + 1] == vector_path[count2]) {
			count3++;
			count2++;

		} else {
			path[count] = count3; //path migi20 hidari30
			count++;
			count3 = 0;
		}

		if (((vector_path[count2 + 1] == 3) && (vector_path[count2] == 0))
				|| ((vector_path[count2 + 1] == 1) && (vector_path[count2] == 3))
				|| ((vector_path[count2 + 1] == 2) && (vector_path[count2] == 1))
				|| ((vector_path[count2 + 1] == 0) && (vector_path[count2] == 2))) {
			path[count] = 20;

		} else if (((vector_path[count2 + 1] == 0) && (vector_path[count2] == 3))
				|| ((vector_path[count2 + 1] == 3) && (vector_path[count2] == 1))
				|| ((vector_path[count2 + 1] == 1) && (vector_path[count2] == 2))
				|| ((vector_path[count2 + 1] == 2) && (vector_path[count2] == 0))) {
			path[count] = 30;

		}

		count++;
		count2++;

	}
	myprintf("path=%d\n\r", path[count]);
}

void Led_init(void) {
	if (PE.DRL.BIT.B10 == 0)
		PB.DR.BIT.B1 = 1;
	if (PE.DRL.BIT.B12 == 0)
		PB.DR.BIT.B3 = 1;
	if (PE.DRL.BIT.B10 == 1)
		PB.DR.BIT.B1 = 0;
	if (PE.DRL.BIT.B12 == 1)
		PB.DR.BIT.B3 = 0;

}

void Mtr(void) {
	PE.DRL.BIT.B8 = 1;
}
void AD_BATTERY(void) {
	AD0.ADCR.BIT.ADST = 0;
	AD0.ADCSR.BIT.CH = 0;
	AD0.ADCR.BIT.ADST = 1;
	while (AD0.ADCSR.BIT.ADF == 0)
		;
	AD0.ADCSR.BIT.ADF = 0;

	BAT = AD0.ADDR0 >> 6;
}
volatile void SEN_LEFT(void) {	//センサ値を呼んで代入するだけの関数1
	AD1.ADCR.BIT.ADST = 0;
	AD1.ADCSR.BIT.CH = 0;
	AD1.ADCR.BIT.ADST = 1;
	while (AD1.ADCSR.BIT.ADF == 0)
		;
	AD1.ADCSR.BIT.ADF = 0;

	SEN_left = AD1.ADDR4 >> 6;
}
volatile void SEN_FRONTLEFT(void) {	//センサ値を呼んで代入するだけの関数2
	AD1.ADCR.BIT.ADST = 0;
	AD1.ADCSR.BIT.CH = 1;
	AD1.ADCR.BIT.ADST = 1;
	while (AD1.ADCSR.BIT.ADF == 0)
		;
	AD1.ADCSR.BIT.ADF = 0;

	SEN_frontleft = AD1.ADDR5 >> 6;
}
volatile void SEN_RIGHT(void) {	//センサ値を呼んで代入するだけの関数3
	AD1.ADCR.BIT.ADST = 0;
	AD1.ADCSR.BIT.CH = 2;
	AD1.ADCR.BIT.ADST = 1;
	while (AD1.ADCSR.BIT.ADF == 0)
		;
	AD1.ADCSR.BIT.ADF = 0;

	SEN_right = AD1.ADDR6 >> 6;
}

volatile void SEN_FRONTRIGHT(void) {	//センサ値を呼んで代入するだけの関数4
	AD1.ADCR.BIT.ADST = 0;
	AD1.ADCSR.BIT.CH = 3;
	AD1.ADCR.BIT.ADST = 1;
	while (AD1.ADCSR.BIT.ADF == 0)
		;
	AD1.ADCSR.BIT.ADF = 0;

	SEN_frontright = AD1.ADDR7 >> 6;
}
volatile void adatiho(void) {
	led_red = 0;
	for (i = 0; i++; i <= 15) {
		for (j = 0; j++; j <= 15) {
			wall[i][j] = 0x00;
		}
	}

	wall[0][0] |= 0x0e;

	wall[1][0] |= 0x44;	//0,0での壁情報
//acceleration(150, 300, 90, 2000);//最初の90mm直進
//deceleration(150, 300, 90, 2000);
	accel_velocity(150, 300, 300, 90, 2000);
//daikei(150, 300, 90, 2000);

	while (1) { //以降ループに突入
		store_map_wall();
		q_walk_map_maker(DESTINATION_X, DESTINATION_Y);
		//draw_map();
		// 先頭位置, 末尾位置
		//myprintf("x=%d\n\r", coordinate_x);
		//myprintf("y=%d\n\r", coordinate_y);
		//logger_sensor();
		if (breakable_flag == 1) {
			breakable_flag = 0;
			//draw_map();
			break;

		}
		if (coordinate_x == DESTINATION_X && coordinate_y == DESTINATION_Y) {

			accel_velocity(300, 150, 300, 90, 2000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0;

			motor_enable = 0;

			//while(1){
			//if(SW_1 == 0){
			led_red = 0;
			led_green = 0;
			map_path();
			break;
			//}
			//}
		}
		if (coordinate_x < 16 && coordinate_y < 16) {
			led_red = 1;
			if (direction_x == 0 && direction_y == 1)				//北
					{
				//draw_map();
				led_green = 1;
				if (((wall[coordinate_x][coordinate_y] & 0x01) == 0)
						&& (map[coordinate_x][coordinate_y + 1]
								== map[coordinate_x][coordinate_y] - 1)) {//北壁がなくかつ歩数マップ-1
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;

					//ConstVel(300, 180);
					//daikei(150, 300, 180, 2000);
					accel_velocity(300, 300, 300, 180, 1);

					coordinate(4);

				} else if (((wall[coordinate_x][coordinate_y] & 0x04) == 0)
						&& (map[coordinate_x - 1][coordinate_y]
								== map[coordinate_x][coordinate_y] - 1)) { //西

					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 1;

					//deceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_left(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(1);

				} else if (((wall[coordinate_x][coordinate_y] & 0x08) == 0)
						&& (map[coordinate_x + 1][coordinate_y]
								== map[coordinate_x][coordinate_y] - 1)) { //東

					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 2;

					//deceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);
					coordinate(2);

				} else {
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 3;

					//daikei(150, 300, 90, 2000);
					//deceleration(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 180, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);
					coordinate(3);

				}
			} else if (direction_x == 1 && direction_y == 0)				//東
					{
				//draw_map();
				if (((wall[coordinate_x][coordinate_y] & 0x08) == 0)
						&& (map[coordinate_x + 1][coordinate_y]
								== map[coordinate_x][coordinate_y] - 1)) {//東壁がなくかつ歩数マップ-1
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;

					//ConstVel(300, 180);
					//daikei(150, 300, 180, 2000);
					accel_velocity(300, 300, 300, 180, 1);
					coordinate(4);

				} else if (((wall[coordinate_x][coordinate_y] & 0x01) == 0)
						&& (map[coordinate_x][coordinate_y + 1]
								== map[coordinate_x][coordinate_y] - 1)) {

					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 1;

					//deceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_left(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(1);

				} else if (((wall[coordinate_x][coordinate_y] & 0x02) == 0)
						&& (map[coordinate_x][coordinate_y - 1]
								== map[coordinate_x][coordinate_y] - 1)) {
					store_map_wall(); //右ターン

					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 2;

					//deceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(2);

				} else {

					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 3;

					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					//deceleration(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 180, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(3);

				}
			} else if (direction_x == -1 && direction_y == 0)				//西
					{
				//draw_map();
				if (((wall[coordinate_x][coordinate_y] & 0x04) == 0)
						&& (map[coordinate_x - 1][coordinate_y]
								== map[coordinate_x][coordinate_y] - 1)) {//西壁がなくかつ歩数マップ-1
					//直進
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;

					//ConstVel(300, 180);
					//daikei(150, 300, 180, 2000);
					accel_velocity(300, 300, 300, 180, 1);

					coordinate(4);

				} else if (((wall[coordinate_x][coordinate_y] & 0x01) == 0) //北壁
						&& (map[coordinate_x][coordinate_y + 1]
								== map[coordinate_x][coordinate_y] - 1)) { //

					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 1;

					//deceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(2);

				} else if (((wall[coordinate_x][coordinate_y] & 0x02) == 0)
						&& (map[coordinate_x][coordinate_y - 1]
								== map[coordinate_x][coordinate_y] - 1)) {

					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 2;

					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_left(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(1);

				} else {
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 3;

					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					//deceleration(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 180, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);
					coordinate(3);

				}
			} else if (direction_x == 0 && direction_y == -1)				//南
					{
				//draw_map();
				if (((wall[coordinate_x][coordinate_y] & 0x02) == 0)
						&& (map[coordinate_x][coordinate_y - 1]
								== map[coordinate_x][coordinate_y] - 1)) {//南壁がなくかつ歩数マップ-1
					//直進
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;

					//ConstVel(300, 180);
					//daikei(150, 300, 180, 2000);
					accel_velocity(300, 300, 300, 180, 1);
					coordinate(4);

				} else if (((wall[coordinate_x][coordinate_y] & 0x08) == 0)
						&& (map[coordinate_x + 1][coordinate_y]
								== map[coordinate_x][coordinate_y] - 1)) { //東
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 1;

					//deceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_left(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(1);

				} else if (((wall[coordinate_x][coordinate_y] & 0x04) == 0)
						&& (map[coordinate_x - 1][coordinate_y]
								== map[coordinate_x][coordinate_y] - 1)) { //西
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 2;

					//deceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);
					coordinate(2);

				} else {
					SENleft_offset = 0;
					SENfrontleft_offset = 0;
					SENright_offset = 0;
					SENfrontright_offset = 0;
					direction_number[log_count] = 3;

					//daikei(150, 300, 90, 2000);
					accel_velocity(300, 150, 300, 90, 2000);
					//deceleration(150, 300, 90, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					tyousinti_right(150, 300, 180, 2000);
					MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
					wait_ms(500);
					MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
					MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
					//acceleration(150, 300, 90, 2000);
					//daikei(150, 300, 90, 2000);
					accel_velocity(150, 300, 300, 90, 2000);

					coordinate(3);

				}
			}
		}
	}
}

void q_walk_map_maker(char goal_x, char goal_y) //ゴール座標を引数に持つ
{
	volatile unsigned char x, y;
	volatile unsigned char q[257]; // 区画の座標(0～255)を入れる配列
	volatile short head, tail;          // 先頭位置, 末尾位置

	for (x = 0; x < 16; x++)		// マップの初期化
			{
		for (y = 0; y < 16; y++) {
			map[x][y] = 255;
		}
	}
	map[goal_x][goal_y] = 0;			// 目標地点に距離０を書き込む
	q[0] = (goal_x * 16 + goal_y);		// 目標地点の座標を記憶 // 1111(x) 1111(y)
	head = 0;							// 先頭位置を初期化
	tail = 1;							// 末尾位置は、最後の情報位置＋１

	while (head != tail)				// 配列の中身が空ならループを抜ける
	{
		y = q[head] & 0x0f;       		// 配列から区画の座標を取り出す
		x = q[head] >> 4;				//4bit右シフトでxの値を代入
		head++;							// 情報を取り出したので先頭位置をずらす。

		if (y < 16)							// 北側
				{
			if ((wall[x][y] & 0x01) == 0)			//北に移動可能(壁がないか，未探索であれば移動可能)
					{
				if (map[x][y + 1] == 255) {
					map[x][y + 1] = map[x][y] + 1;
					q[tail] = (x * 16 + y + 1);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす

				}
			}
		}
		if (x < 16)     // 東側
				{
			if ((wall[x][y] & 0x08) == 0) //東に移動可能(壁がないか，未探索であれば移動可能)
					{
				if (map[x + 1][y] == 255) {
					map[x + 1][y] = map[x][y] + 1;
					q[tail] = ((x + 1) * 16 + y);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす

				}
			}
		}
		if (y > 0)     // 南側
				{
			if ((wall[x][y] & 0x02) == 0) //南に移動可能(壁がないか，未探索であれば移動可能)
					{
				if (map[x][y - 1] == 255) {
					map[x][y - 1] = map[x][y] + 1;
					q[tail] = (x * 16 + y - 1);     // 次の区画の座標を記憶
					tail++;     // 情報を入れたので末尾位置をずらす

				}
			}
		}
		if (x > 0)     // 西側
				{
			if ((wall[x][y] & 0x04) == 0) //西に移動可能(壁がないか，未探索であれば移動可能)
					{
				if (map[x - 1][y] == 255) {
					map[x - 1][y] = map[x][y] + 1;
					q[tail] = ((x - 1) * 16 + y);     // 次の区画の座標を記憶
					tail++;     // 情=報を入れたので末尾位置をずらす

				}
			}
		}
	}
}

volatile void draw_map(void) {

	myprintf("DRAWING A MAP OF THE MAZE....HAVE A NICE DAY!\n\r");
	for (k = 15; k >= 0; k--) {
		for (j = 0; j <= 15; j++) {
			if ((wall[j][k] & 0x01) == 0x01) {     //北壁

				myprintf("+---+");
			} else
				myprintf("+   +");
		}
		myprintf("\n\r");
		for (j = 0; j <= 15; j++) {
			if ((wall[j][k] & 0x04) == 0x04) {
				myprintf("|");
			} else
				myprintf(" ");
			myprintf("%3d", map[j][k]);
			//	myprintf("   ");
			if ((wall[j][k] & 0x08) == 0x08) {
				myprintf("|");
			} else
				myprintf(" ");
		}
		myprintf("\n\r");
		for (j = 0; j <= 15; j++) {
			if ((wall[j][k] & 0x02) == 0x02) {     //南壁
				myprintf("+---+");
			} else
				myprintf("+   +");
		}
		myprintf("\n\r");

	}
	led_red = 0;
	led_green = 0;
}
volatile void store_map_wall(void) {

	if (direction_x == 0 && direction_y == 1) {     //北を向いている時
		wall[coordinate_x][coordinate_y] |= 0x10;
		if (SENleft_offset >= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
		&& SENfrontright_offset >= LED_FR_TH      //北と東西あり
				&& SENfrontleft_offset >= LED_FL_TH) {
			wall[coordinate_x][coordinate_y] |= 0x0d;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上の南
			}
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04;     //右の西
			}
		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontleft_offset >= LED_FL_TH) {     //西と北
			wall[coordinate_x][coordinate_y] |= 0x05;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上の南
			}
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;
			}

		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontright_offset >= LED_FR_TH) {     //東と北
			wall[coordinate_x][coordinate_y] |= 0x09;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上の南
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04;
			}
		} else if (SENfrontleft_offset >= LED_FL_TH
				&& SENfrontright_offset >= LED_FR_TH) {     //東と西
			wall[coordinate_x][coordinate_y] |= 0x0c;
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04;     //右の西
			}
		} else if (SENleft_offset >= LED_L_TH - 5
				&& SENright_offset >= LED_R_TH - 5) {     //北
			wall[coordinate_x][coordinate_y] |= 0x01;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上区画の南
			}

		} else if (SENfrontleft_offset >= LED_FL_TH) {     //西
			wall[coordinate_x][coordinate_y] |= 0x04;
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左区画の東
			}

		} else if (SENfrontright_offset >= LED_FR_TH) {     //東
			wall[coordinate_x][coordinate_y] |= 0x08;
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04;     //右区画の西
			}
		} else
			wall[coordinate_x][coordinate_y] |= 0x00;

	} else if (direction_x == 1 && direction_y == 0) {     //東
		wall[coordinate_x][coordinate_y] |= 0x10;
		if (SENleft_offset >= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
		&& SENfrontright_offset >= LED_FR_TH
		&& SENfrontleft_offset >= LED_FL_TH) { //北東南
			wall[coordinate_x][coordinate_y] |= 0x0b;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02; //上の南
			}
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01; //下の北
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}
		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontleft_offset >= LED_FL_TH) { //北東
			wall[coordinate_x][coordinate_y] |= 0x09;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02; //上の南
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}
		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontright_offset >= LED_FR_TH) { //東南
			wall[coordinate_x][coordinate_y] |= 0x0a;
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01; //下の北
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}

		} else if (SENfrontleft_offset >= LED_FL_TH
				&& SENfrontright_offset >= LED_FR_TH) { //北南
			wall[coordinate_x][coordinate_y] |= 0x03;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02; //上の南
			}
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01; //下の北
			}

		} else if (SENleft_offset >= LED_L_TH - 5
				&& SENright_offset >= LED_R_TH - 5) { //東
			wall[coordinate_x][coordinate_y] |= 0x08;
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}

		} else if (SENfrontleft_offset >= LED_FL_TH) { //北
			wall[coordinate_x][coordinate_y] |= 0x01;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02; //上の南
			}

		} else if (SENfrontright_offset >= LED_FR_TH) { //南
			wall[coordinate_x][coordinate_y] |= 0x02;
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01; //下の北
			}

		} else
			wall[coordinate_x][coordinate_y] |= 0x00;
	} else if (direction_x == -1 && direction_y == 0) {     //西
		wall[coordinate_x][coordinate_y] |= 0x10;
		if (SENleft_offset >= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
		&& SENfrontright_offset >= LED_FR_TH
		&& SENfrontleft_offset >= LED_FL_TH) {     //北南西
			wall[coordinate_x][coordinate_y] |= 0x07;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上の南
			}
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}

		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontleft_offset >= LED_FL_TH) {     //南西
			wall[coordinate_x][coordinate_y] |= 0x06;
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}

		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontright_offset >= LED_FR_TH) {     //西北
			wall[coordinate_x][coordinate_y] |= 0x05;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上の南
			}
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}

		} else if (SENfrontleft_offset >= LED_FL_TH
				&& SENfrontright_offset >= LED_FR_TH) {     //北南
			wall[coordinate_x][coordinate_y] |= 0x03;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上の南
			}

			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}

		} else if (SENleft_offset >= LED_L_TH - 5
				&& SENright_offset >= LED_R_TH - 5) {     //西
			wall[coordinate_x][coordinate_y] |= 0x04;
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}
		} else if (SENfrontleft_offset >= LED_FL_TH) {     //南
			wall[coordinate_x][coordinate_y] |= 0x02;
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}

		} else if (SENfrontright_offset >= LED_FR_TH) {     //北
			wall[coordinate_x][coordinate_y] |= 0x01;
			if (coordinate_y < 15) {
				wall[coordinate_x][coordinate_y + 1] |= 0x02;     //上の南
			}

		} else
			wall[coordinate_x][coordinate_y] |= 0x00;

	} else if (direction_x == 0 && direction_y == -1) {     //南
		wall[coordinate_x][coordinate_y] |= 0x10;
		if (SENleft_offset >= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
		&& SENfrontright_offset >= LED_FR_TH
		&& SENfrontleft_offset >= LED_FL_TH) {     //南東西

			wall[coordinate_x][coordinate_y] |= 0x0d;

			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}

		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontleft_offset >= LED_FL_TH) { //南東
			wall[coordinate_x][coordinate_y] |= 0x0a;
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}

		} else if (SENleft_offset
				>= LED_L_TH - 5&& SENright_offset >= LED_R_TH - 5
				&& SENfrontright_offset >= LED_FR_TH) { //南西
			wall[coordinate_x][coordinate_y] |= 0x06;
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}

		} else if (SENfrontleft_offset
				>= LED_FL_TH - 5&& SENfrontright_offset >= LED_FR_TH) {    //東西
			wall[coordinate_x][coordinate_y] |= 0x0c;
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}

			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}

		} else if (SENleft_offset >= LED_L_TH - 5
				&& SENright_offset >= LED_R_TH - 5) { //南
			wall[coordinate_x][coordinate_y] |= 0x02;
			if (coordinate_y >= 1) {
				wall[coordinate_x][coordinate_y - 1] |= 0x01;     //下の北
			}
		} else if (SENfrontleft_offset >= LED_FL_TH) {     //東
			wall[coordinate_x][coordinate_y] |= 0x08;
			if (coordinate_x < 15) {
				wall[coordinate_x + 1][coordinate_y] |= 0x04; //右の西
			}
		} else if (SENfrontright_offset >= LED_FR_TH) { //西
			wall[coordinate_x][coordinate_y] |= 0x04;
			if (coordinate_x >= 1) {
				wall[coordinate_x - 1][coordinate_y] |= 0x08;     //左の東
			}

		} else
			wall[coordinate_x][coordinate_y] |= 0x00;
	}
}
volatile void test_map(void) {
	for (i = 0; i++; i <= 15) {
		for (j = 0; j++; j <= 15) {
			wall[i][j] = 0x00;
		}
	}

	wall[0][0] |= 0x0e;

	wall[1][0] |= 0x44;     //0,0での壁情報

	wall[0][1] |= 0x0d;

	wall[1][1] |= 0x04;

	wall[0][2] |= 0x02;

	q_walk_map_maker(DESTINATION_X, DESTINATION_Y);
	draw_map();

}
volatile void coordinate(int dir) {     //coordinate座標
	switch (dir) {
	case 1:     //left
		if (direction_x == 0 && direction_y == 1) { //direction_x,direction_y 01北10東-10西0-1南
			direction_x = -1;
			direction_y = 0;
		} else if (direction_x == 1 && direction_y == 0) {
			direction_x = 0;
			direction_y = 1;
		} else if (direction_x == -1 && direction_y == 0) {
			direction_x = 0;
			direction_y = -1;
		} else if (direction_x == 0 && direction_y == -1) {
			direction_x = 1;
			direction_y = 0;
		}
		break;
	case 2:     //right
		if (direction_x == 0 && direction_y == 1) {
			direction_x = 1;
			direction_y = 0;
		} else if (direction_x == 1 && direction_y == 0) {
			direction_x = 0;
			direction_y = -1;
		} else if (direction_x == -1 && direction_y == 0) {
			direction_x = 0;
			direction_y = 1;
		} else if (direction_x == 0 && direction_y == -1) {
			direction_x = -1;
			direction_y = 0;
		}
		break;
	case 3:     //turn
		if (direction_x == 0 && direction_y == 1) {
			direction_x = 0;
			direction_y = -1;
		} else if (direction_x == 1 && direction_y == 0) {
			direction_x = -1;
			direction_y = 0;
		} else if (direction_x == -1 && direction_y == 0) {
			direction_x = 1;
			direction_y = 0;
		} else if (direction_x == 0 && direction_y == -1) {
			direction_x = 0;
			direction_y = 1;
		}
		break;
	default:
		break;

	}
	/*if(coordinate_x == DESTINATION_X && coordinate_y == DESTINATION_Y){
	 myprintf("error");
	 led_red =1;
	 led_green = 1;
	 daikei(150, 300, 90, 1000);
	 motor_enable =0;

	 }*/
	coordinate_x += direction_x;
	coordinate_y += direction_y;

}

volatile void hidariteho(void) {
	led_red = 0;
//acceleration(150, 300, 90, 2000);//最初の90mm直進
//deceleration(150, 300, 90, 1000);
//daikei(150, 300, 90, 1000);
	accel_velocity(150, 500, 500, 90, 1000);
//wait_ms(100);
	while (1) { //以降ループに突入
		logger_sensor();
		if (breakable_flag == 1) {
			breakable_flag = 0;
			break;

		}
		if (coordinate_x == DESTINATION_X && coordinate_y == DESTINATION_Y) {
			led_red = 1;
			led_green = 1;

			//daikei(150, 300, 90, 1000);
			accel_velocity(150, 150, 500, 90, 1000);
			coordinate(4);
			motor_enable = 0;
			myprintf("goal");
			//while(1){
			//if(SW_1 == 0){
			for (i = 0; i++; i <= 500)
				;
			q_walk_map_maker(DESTINATION_X, DESTINATION_Y);
			draw_map();
			//	break;
			//}
			//}
		}
		if (SENfrontleft_offset < LED_FL_TH) {
			store_map_wall();
			led_green = 0;
			led_red = 1;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;
			direction_number[log_count] = 1;

			//deceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);

			accel_velocity(500, 150, 500, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作

			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_left(150, 500, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			//acceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);
			accel_velocity(150, 500, 500, 90, 1000);
			coordinate(1);
		} else if (SENleft_offset <= LED_L_TH && SENright_offset <= LED_R_TH) {
			store_map_wall();
			led_green = 0;
			led_red = 0;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;

			//ConstVel(300, 180);
			//daikei(150, 300, 180, 1000);
			accel_velocity(500, 500, 500, 180, 1000);
			coordinate(4);
		} else if (SENfrontright_offset < LED_FR_TH) {
			store_map_wall();
			led_green = 1;
			led_red = 0;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;
			direction_number[log_count] = 2;

			//deceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);
			accel_velocity(500, 150, 500, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_right(150, 300, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			//acceleration(150, 300, 90, 1000);
			accel_velocity(150, 500, 500, 90, 1000);
			//daikei(150, 300, 90, 1000);
			coordinate(2);
		} else {
			store_map_wall();
			led_green = 1;
			led_red = 1;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;
			direction_number[log_count] = 3;

			//daikei(150, 300, 90, 1000);
			accel_velocity(500, 150, 500, 90, 1000);
			//deceleration(150, 300, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_right(150, 500, 180, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			//acceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);
			accel_velocity(150, 500, 500, 90, 1000);
			coordinate(3);
		}

	}

}

volatile void hidariteho_slalom(void) {
	led_red = 0;
//acceleration(150, 300, 90, 2000);//最初の90mm直進
//deceleration(150, 300, 90, 1000);
//daikei(150, 300, 90, 1000);
	accel_velocity(150, 500, 500, 90, 1000);
//wait_ms(100);
	while (1) { //以降ループに突入
		logger_sensor();
		if (breakable_flag == 1) {
			breakable_flag = 0;
			break;

		}
		if (coordinate_x == DESTINATION_X && coordinate_y == DESTINATION_Y) {
			led_red = 1;
			led_green = 1;

			//daikei(150, 300, 90, 1000);
			accel_velocity(150, 150, 500, 90, 1000);
			coordinate(4);
			motor_enable = 0;
			myprintf("goal");
			//while(1){
			//if(SW_1 == 0){
			for (i = 0; i++; i <= 500)
				;
			q_walk_map_maker(DESTINATION_X, DESTINATION_Y);
			draw_map();
			//	break;
			//}
			//}
		}
		if (SENfrontleft_offset < LED_FL_TH) {
			store_map_wall();
			led_green = 0;
			led_red = 1;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;
			direction_number[log_count] = 1;

			//deceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);

			accel_velocity(500, 150, 500, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作

			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_left(150, 500, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			//acceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);
			accel_velocity(150, 500, 500, 90, 1000);
			coordinate(1);
		} else if (SENleft_offset <= LED_L_TH && SENright_offset <= LED_R_TH) {
			store_map_wall();
			led_green = 0;
			led_red = 0;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;

			//ConstVel(300, 180);
			//daikei(150, 300, 180, 1000);
			accel_velocity(500, 500, 500, 180, 1000);
			coordinate(4);
		} else if (SENfrontright_offset < LED_FR_TH) {
			store_map_wall();
			led_green = 1;
			led_red = 0;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;
			direction_number[log_count] = 2;

			//deceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);
			accel_velocity(500, 150, 500, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_right(150, 300, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			//acceleration(150, 300, 90, 1000);
			accel_velocity(150, 500, 500, 90, 1000);
			//daikei(150, 300, 90, 1000);
			coordinate(2);
		} else {
			store_map_wall();
			led_green = 1;
			led_red = 1;
			SENleft_offset = 0;
			SENfrontleft_offset = 0;
			SENright_offset = 0;
			SENfrontright_offset = 0;
			direction_number[log_count] = 3;

			//daikei(150, 300, 90, 1000);
			accel_velocity(500, 150, 500, 90, 1000);
			//deceleration(150, 300, 90, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			tyousinti_right(150, 500, 180, 1000);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作
			//acceleration(150, 300, 90, 1000);
			//daikei(150, 300, 90, 1000);
			accel_velocity(150, 500, 500, 90, 1000);
			coordinate(3);
		}

	}

}

void P_control(void) {
	if ((SENfrontright_offset > led_fr_th)
			&& (SENfrontleft_offset > led_fl_th)) {
		error = (SENfrontleft_offset - LED_FL_REF)
				- (SENfrontright_offset - LED_FR_REF);
	} else if ((SENfrontright_offset <= led_fr_th)
			&& (SENfrontleft_offset < led_fl_th)) {
		error = 0;
	} else if (SENfrontright_offset > led_fr_th) {
		error = -2 * (SENfrontright_offset - LED_FR_REF);
	} else {
		error = 2 * (SENfrontleft_offset - LED_FL_REF);
	}

	control = P_GAIN * error;
	sen_left_difference = SENfrontleft_offset - sen_left_old;
	sen_right_difference = SENfrontright_offset - sen_right_old;
	sen_left_old = SENfrontleft_offset;
	sen_right_old = SENfrontright_offset;
	if ((abs(sen_right_difference) >= DIFF_THRESHOLD_RIGHT)
			|| (SENfrontright_offset >= LED_FR_REF - 5)
			|| (SENfrontright_offset <= LED_FR_REF + 5) || (velocity < 280)) { //右壁偏差制御オフ
		tmp_sen_r_th = led_r_th;
		led_r_th = LED_R_REF + 50;
		error = 0;
	} else {
		led_r_th = tmp_sen_r_th;
	}
	if ((abs(sen_left_difference) >= DIFF_THRESHOLD_LEFT)
			|| (SENfrontleft_offset >= LED_FL_REF - 5)
			|| (SENfrontleft_offset <= LED_FL_REF + 5) || (velocity < 280)) { //左壁偏差制御オフ
		tmp_sen_l_th = led_l_th;
		led_l_th = LED_L_REF + 50;
		error = 0;
	} else {
		led_l_th = tmp_sen_l_th;
	}

}

void SenRead(void) {
	if (T % 4 != 0) {
		LED_LEFT = 1;
		LED_FRONTLEFT = 1;
		LED_RIGHT = 1;
		LED_FRONTRIGHT = 1;
		for (i = 0; i < 100; i++) {
		}
		SEN_LEFT();
		SENleft_offset = (SEN_left - SENleft_off);
		SEN_FRONTLEFT();
		SENfrontleft_offset = (SEN_frontleft - SENfrontleft_off);
		SEN_RIGHT();
		SENright_offset = (SEN_right - SENright_off);
		SEN_FRONTRIGHT();
		SENfrontright_offset = (SEN_frontright - SENfrontright_off);

	} else {

		LED_LEFT = 0;
		LED_FRONTLEFT = 0;
		LED_RIGHT = 0;
		LED_FRONTRIGHT = 0;
		for (i = 0; i < 100; i++) {
		}
		SEN_LEFT();
		SENleft_off = SEN_left;
		SEN_FRONTLEFT();
		SENfrontleft_off = SEN_frontleft;
		SEN_RIGHT();
		SENright_off = SEN_right;
		SEN_FRONTRIGHT();
		SENfrontright_off = SEN_frontright;

	}
}
void Led_Twinkle(void) {

	while (1) {
		for (i = 50000; i > 0; i--) {
			PB.DR.BIT.B1 = 1;
			PB.DR.BIT.B3 = 0;
		}
		for (; i < 50000; i++) {
			PB.DR.BIT.B1 = 0;
			PB.DR.BIT.B3 = 1;
		}
	}
}
volatile void tyousinti_test() {
	while (1) {
		while (1) {
			if (SW_YELLOW == 0) {
				wait_ms(500);
				sw_1++;
			}
			if (sw_1 == 1) {
				led_red = 1;
				led_green = 0;
			} else if (sw_1 == 2) {
				led_green = 1;
				led_red = 0;
			} else if (sw_1 >= 3) {

				sw_1 = 1;
			}
			if (SW_BROWN == 0) {
				wait_ms(500);
				break;
			}

		}
		switch (sw_1) {
		case 1:
			motor_driver_reset = 1; //Motor Driver Reset
			wait_ms(1);

			motor_driver_reset = 0; //Motor Driver Reset解除
			wait_ms(10);

			motor_enable = 1; //Motor Enable=1,励磁条件
			wait_ms(1000);
			//daikei(150, 500, 2700, 1000);
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作

			//hidariteho();
			tyousinti_right(150, 300, 360, 1000);
			//accel_valocity(150, 300, 300, 180, 2000);
			//circuit(15);
			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作
			//daikei(150, 500, 720, 1000);

			sw_1 = 0;
			break;
		case 2:
			motor_driver_reset = 1; //Motor Driver Reset
			wait_ms(1);

			motor_driver_reset = 0; //Motor Driver Reset解除
			wait_ms(10);

			motor_enable = 1; //Motor Enable=1,励磁条件
			wait_ms(1000);
			MTU2.TSTR.BIT.CST0 = 1; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 1; //動作TCNT1動作

			tyousinti_left(150, 300, 360, 1000);

			MTU2.TSTR.BIT.CST0 = 0; //9.3.14 タイマスタートレジスタ(TSTR)TCNT0動作
			MTU2.TSTR.BIT.CST1 = 0; //動作TCNT1動作

			sw_1 = 0;
			break;

		default:

			break;

		}
		break;

	}

}
volatile void slalom_right(float vel , float angle_vel , float angle_accel , float ang_max)
{
	float tmp_accelerate;

	float ang1,ang2,ang3;
	float tmp_angle_accelerate;

	//distance_sankaku跡地

	//myprintf("distance_max=%f\t", distance_max);
		accel_velocity(vel, vel, vel, 10, 1000);

		P_control_flag = 1;
		CWCCW_right = 0;
		CWCCW_left = 1;
		distance = 0;
		angle_body = 0;
		angle_velocity = 0;
		angle_accelerate = 0;
		angle_decelerate = 0;
		distance = 0;
		velocity = vel;
		tmp_angle_accelerate = angle_accel;
		ang1=ang_max/3;
		ang2=(ang_max*2)/3;
		ang3=ang_max;
		while (1) {
			if ((angle_body < ang1) &&(velocity_right>=150) ) {
				angle_accelerate = angle_accel;
			}
			if (((angle_body >= ang1)&&(angle_body <= ang2))||(velocity_right<150)) {

				angle_accelerate = 0.0;
				if(velocity_right<150){
					velocity_right = 150;
				}

			}
			if ((angle_body <= ang3)
					&& (angle_body >= ang2)&&(velocity_left>=vel)) {
				angle_accelerate = (-tmp_angle_accelerate);

			}
			if (angle_body >= ang3 )  {
				angle_velocity = 0.0;
				angle_accelerate = 0.0;
				angle_body = 0.0;

				break;
			}
		}
		accel_velocity(vel, vel, vel, 10, 1000);


}
volatile void slalom_left(float vel , float angle_vel , float angle_accel , float ang_max)//bugなおす
{
	float tmp_accelerate;

	float ang1,ang2,ang3;
	float tmp_angle_accelerate;

	//distance_sankaku跡地

	//myprintf("distance_max=%f\t", distance_max);
		accel_velocity(vel, vel, vel, 10, 1000);
		P_control_flag = 1;
		CWCCW_right = 0;
		CWCCW_left = 1;
		distance = 0;
		angle_body = 0;
		angle_velocity = 0;
		angle_accelerate = 0;
		angle_decelerate = 0;
		distance = 0;
		velocity = vel;
		tmp_angle_accelerate = -angle_accel;
		ang1=-ang_max/3;
		ang2=-(ang_max*2)/3;
		ang3=-ang_max;
		while (1) {
			if ((-angle_body > ang1)&&(velocity_left>=150)) {
				angle_accelerate = -angle_accel;
			}
			if (((-angle_body <= ang1)
					&& (-angle_body >= ang2))||(velocity_left<150)) {

				angle_accelerate = 0.0;

			}
			if ((-angle_body >= ang3)
					&& (-angle_body <= ang2)&&(velocity_right>=vel)) {
				angle_accelerate = (-tmp_angle_accelerate);

			}
			if (angle_body <= ang3)  {
				angle_velocity = 0.0;
				angle_accelerate = 0.0;
				angle_body = 0.0;

				break;
			}
		}
		accel_velocity(vel, vel, vel, 10, 1000);


}

void Spk(void) {
//レベルアップ音
	int tempo = 200;
//
	ctrl_bz_melody(BZ_FREQ_SO, 1, tempo);
	ctrl_bz_melody(BZ_FREQ_SO, 1, tempo);
	ctrl_bz_melody(BZ_FREQ_SO, 1, tempo);
	ctrl_bz_melody(BZ_FREQ_FA, 1, tempo);
	ctrl_bz_melody(BZ_FREQ_REST, 1, tempo);
	ctrl_bz_melody(BZ_FREQ_LA, 1, tempo);
	ctrl_bz_melody(BZ_FREQ_SO, 4, tempo);
}
