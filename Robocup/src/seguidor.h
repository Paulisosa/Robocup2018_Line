#include "mbed.h"
#include "mcp3208.h"
#include "MotorDC.h"

#define NUM_SENSORS	     6
#define DIS0			 55
#define DIS1			 165
#define DIS2			 275
#define DIS_MAX			 73
#define DIS_MIN			 -65
#define MAX_VALUE_SENSOR 4096
#define VEL_MINIMA 0.1f

//Estructura que contiene las variables necesarias
//para el correcto funcionamiento de la funcion seguirLineaPid()
struct VariablesPID
{
public:
	VariablesPID(float _kp, float _kd, float _ki)
	{
		kp = _kp;
		kd = _kd;
		ki = _ki; 
		errorProporcional = 0.;
		errorDerivada = 0.;
		errorIntegral = 0.;
		idt = 100.0f;
		dt = 1. / idt;
	}
	float kp;
	float kd;
	float ki;
	float errorProporcional;
	float errorDerivada;
	float errorIntegral;
	float idt;
	float dt;
};

//Obtiene el error Proporcional. 
float obtenerError(int32_t array_value[]);

void motorSpeed(float pwm, MotorDC &m_izq, MotorDC &m_der, float velMaxima);

//Factor kp: Valores bajos hacen que el robot no siga la linea.
//           Valores altos generan muchas osilaciones.
//Factor ki: Valores bajos no causan impacto.
//           Valores altos generan grandes osilaciones.
//Factor kd: Valores altos procovan sobre amortiguacion. 
void seguirLineaPID(int32_t line_array[], VariablesPID& vp, 
					MotorDC &m_izq, MotorDC &m_der, float velMaxima);
