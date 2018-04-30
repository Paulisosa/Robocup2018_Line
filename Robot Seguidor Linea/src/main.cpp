#include "mbed.h"
#include "mcp3208.h"
#include "EV3UARTSensor.h"
#include "SysTimer.h"
#include "seguidor.h"

DigitalOut ledr(LED_RED, 1);
DigitalOut ledg(LED_GREEN, 1);
DigitalOut ledb(LED_BLUE, 1);
//DigitalOut stime(PTC3,0);
DigitalOut en_motors(PTB20, 1);
DigitalIn sw(SW2);
SPI device(PTD2, PTD3, PTD1);
MCP3208 mcp(device, PTD0);
RawSerial s3(PTC15, PTC14, 2400);
RawSerial s4(PTC17, PTC16, 2400);

EV3UARTSensor sensor1;
EV3UARTSensor sensor2;
float sample1[5]; //der
float sample2[5]; //izq

MotorDC mizq(PTC11, PTC5, PTC7);
MotorDC mder(PTC10, PTC9, PTC0);


int32_t line_array[7];

float err;
float vel;
float vel2;
float derror;
float idt = 100.;
float kp = 0.8f;
float kd = 0.05f;
float ki = 0.0f;
float dt = 1./idt;
float ierror;

uint8_t mode;
int i = 0;
bool dissensor = true;

void seguidor();
void cal();
void printSensors();
void checkgap();
volatile uint32_t* tm;

uint32_t t1;
uint32_t t2;

Ticker slego;

bool sensor_out_range;
bool gap = false;
bool inter = false;

int inter_verde = 0;

bool read_color = true;


void linepid() {


	derror = -err;
	err = obtenerError(line_array);
	derror += err;

	ierror += err * dt;

	if (ierror > 0.01)
		ierror = 0.01;
	if (ierror < -0.01)
		ierror = -0.01;

	//printf("%.3f\t%.3f\n",err,vel2);

	vel = kp * err + kd * derror * idt + ki * ierror;
	motorSpeed(vel, mizq, mder);

}

Serial pc(USBTX, USBRX);

void leercolor() {
	sensor1.check_for_data();
	sensor2.check_for_data();
	sensor1.fetch_sample(sample1, 0);
	sensor2.fetch_sample(sample2, 0);
}

int main() {
	
	//mder.speed(0.5);
	sensor_out_range = false;
	initSystemClock();
	tm = getPointerMillis();
	sensor1.begin(s4);
	sensor2.begin(s3);

	sensor1.connect(ledg); //der
	sensor2.connect(ledr); //izq
	delay_ms(100);
	sensor1.set_mode(ColColor);
	sensor2.set_mode(ColColor);
	//slego.attach_us(&leercolor,50000);

//	 while (1) {
//
//	 sensor1.check_for_data();
//	 sensor2.check_for_data();
//	 sensor1.fetch_sample(sample1, 0);
//	 sensor2.fetch_sample(sample2, 0);
//
//	 printf("%.1f\t%.1f\n", sample2[0], sample1[0]);
//	 printSensors();
//	 //err = obtenerError(line_array);
//	 err = calibrar(line_array);
//	 printf("err\t%.3f\n", err);
//	 delay_ms(100);
//	 }
	t1 = *tm;
	pc.baud (9600);
	while (1) {
		line_array[0] = mcp.iread_input(5);
		line_array[1] = mcp.iread_input(4);
		line_array[2] = mcp.iread_input(3);
		line_array[3] = mcp.iread_input(2);
		line_array[4] = mcp.iread_input(1);
		line_array[5] = mcp.iread_input(0);
		//line_array[6] = mcp.iread_input(6);
		linepid();

		while ((*tm - t1) < 10) {
			sensor1.check_for_data();
			sensor2.check_for_data();
			sensor1.fetch_sample(sample1, 0);
			sensor2.fetch_sample(sample2, 0);
		}
		t1 = *tm;
		/*
		line_array[6] = mcp.iread_input(6);
		if ((line_array[6] > 950) && sensor_out_range == true) {

			if ((sample1[0] == 6.0) && (sample2[0] == 6.0)) {
				//spid.detach();
				ierror = 0;
				//resetear iderr
				gap = true;
				mder = 0.0;
				mizq = 0.0;
				vel = 0;
				//return;

			}
			if (gap) {
				ledg = 1;
				ledr = 1;
				mder = -0.2;
				mizq = -0.2;
				while (line_array[6] > 950) {
					line_array[6] = mcp.iread_input(6);
					sensor1.check_for_data();
					sensor2.check_for_data();
					sensor1.fetch_sample(sample1, 0);
					sensor2.fetch_sample(sample2, 0);
					if ((sample1[0] == 1.0) || (sample2[0] == 1.0)) {
						ledg = 0;
						gap = false;
						break;
					}
				}
				mizq = 0.0;
				mder = 0.0;
				if (gap) {
					ledr = 0;
					while (1)
						;
				}

				delay_ms(200);
				if (err > 0.0) {
					motorSpeed(0.7, mizq, mder);
					while (line_array[6] > 950) {
						line_array[6] = mcp.iread_input(6);
					}

					//delay_ms(400);

				} else {
					motorSpeed(-0.7, mizq, mder);
					while (line_array[6] > 950) {
						line_array[6] = mcp.iread_input(6);
					}
					//delay_ms(400);
				}
				mizq = 0.0;
				mder = 0.0;
				ierror = 0;
			}
		} else if ((sample1[0] == 3.0) || (sample2[0] == 3.0)) {

			inter_verde++;

			if (inter_verde == 2) {
				ledg = 1;
				ledb = 0;
				mizq = 0.0;
				mder = 0.0;
				ierror = 0;
				err = 0.0;

				while (1) {
				}
			}

		}
		/* else if ((sample1[0] == 3.0) || (sample2[0] == 3.0)) {
		 ledg = 1;
		 ledb = 0;
		 mizq = 0.0;
		 mder = 0.0;
		 ierror = 0;
		 err = 0.0;
		 t2 = *tm;
		 while ((*tm - t2) < 10) {
		 sensor1.check_for_data();
		 sensor2.check_for_data();
		 sensor1.fetch_sample(sample1, 0);
		 sensor2.fetch_sample(sample2, 0);
		 }
		 if ((sample1[0] == 3.0) || (sample2[0] == 3.0)) {
		 while (1) {
		 ledb = !ledb;
		 delay_ms(150);
		 }

		 }
		 ledb = 1;

		 }*/

	}
}
void cal() {

	line_array[0] = mcp.iread_input(5);
	line_array[1] = mcp.iread_input(4);
	line_array[2] = mcp.iread_input(3);
	line_array[3] = mcp.iread_input(2);
	line_array[4] = mcp.iread_input(1);
	line_array[5] = mcp.iread_input(0);
	line_array[6] = mcp.iread_input(6);

	printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n", line_array[0], line_array[1],
			line_array[2], line_array[3], line_array[4], line_array[5],
			line_array[6]);
	calibrar(line_array);
	//err = obtenerError(line_array);
	//printf("err\t%.3f\n",err);

}
/*
 void calibrarSensores(){

 line_array[0] = mcp.iread_input(5);
 line_array[1] = mcp.iread_input(4);
 line_array[2] = mcp.iread_input(3);
 line_array[3] = mcp.iread_input(2);
 line_array[4] = mcp.iread_input(1);
 line_array[5] = mcp.iread_input(0);
 line_array[6] = mcp.iread_input(6);

 }
 */
void printSensors() {
	line_array[0] = mcp.iread_input(5);
	line_array[1] = mcp.iread_input(4);
	line_array[2] = mcp.iread_input(3);
	line_array[3] = mcp.iread_input(2);
	line_array[4] = mcp.iread_input(1);
	line_array[5] = mcp.iread_input(0);
	line_array[6] = mcp.iread_input(6);

	printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n", line_array[0], line_array[1],
			line_array[2], line_array[3], line_array[4], line_array[5],
			line_array[6]);
}