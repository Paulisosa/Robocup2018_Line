#include "mbed.h"
#include "mcp3208.h"
#include "MotorDC.h"

#define NUM_SENSORS 6
#define DIS0		55
#define DIS1		165
#define DIS2		275
#define DIS_MAX		73
#define DIS_MIN		-65//-66
#define MAX_VALUE_SENSOR 4096
#define PWM_MAX_IZQ 0.75f
#define PWM_MIN_IZQ 0.1f
#define PWM_MAX_DER 0.75f
#define PWM_MIN_DER 0.1f
//#define INVERT_SENSORS
//#define INVERT_ERROR

extern bool sensor_out_range;

float obtenerError(int32_t array_value[]);
float ctrlPID(int32_t sensor_value[]);
float calibrar(int32_t array_value[]);
void motorSpeed(float pwm, MotorDC &m_izq, MotorDC &m_der);
void motorSpeed2(float pwm, MotorDC &m_izq, MotorDC &m_der);
