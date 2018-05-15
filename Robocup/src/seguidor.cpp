#include "seguidor.h"

#ifdef	INVERT_ERROR
	const int32_t dis_sensor[]= {DIS2,DIS1,DIS0,-DIS0,-DIS1,-DIS2};
#else
	const int32_t dis_sensor[] = { -DIS2 - 10, -DIS1 - 10, -DIS0 - 10, DIS0, DIS1, DIS2 };
#endif

float obtenerError(int32_t array_value[]) 
{
	int32_t sum_sen = 0;
	int32_t sum = 0;
	int32_t dis = 0;
	static float delta = 0;

	for (uint8_t i = 0; i < NUM_SENSORS; i++) 
	{
		#ifdef INVERT_SENSORS
			sum_sen+=(MAX_VALUE_SENSOR-array_value[i]);
		#else
			sum_sen += array_value[i];
		#endif
		
		sum += (array_value[i] * dis_sensor[i]);
	}

	dis = sum / sum_sen;

	if (sum_sen > 10000) 
	{
		if (delta > 0.)
			delta = 1.;
		else 
			delta = -1.;
	} 
	else 
	{
		delta = (2. * ((float) dis - DIS_MIN)) / (DIS_MAX - DIS_MIN) - 1.;
	}
	return delta;
}

void motorSpeed(float pwm, MotorDC &m_izq, MotorDC &m_der, float velMaxima)
{
	float pwm_der, pwm_izq;

	//Limitar la salida de pwm 
	if (pwm > 1.0)
		pwm = 1.0;
	if (pwm < -1.0)
		pwm = -1.0;

	if (pwm > 0.0) //Girar a la izquierda
	{					
		m_der.setSpeed(velMaxima);

		if (pwm <= 0.5) 
		{
			pwm_izq = velMaxima - 2 * pwm * (velMaxima - 0.1f);
			m_izq.setSpeed(pwm_izq);
		} 
		else 
		{
			pwm_izq = 0.1f + 2 * (pwm - 0.5) * (velMaxima - 0.1f);
			m_izq.setSpeed(-pwm_izq);
		}
	} 
	else if (pwm < 0.0) // Girar a la derecha
	{		
		m_izq.setSpeed(velMaxima);
		pwm = -pwm;

		if (pwm <= 0.5) 
		{
			pwm_der = velMaxima - 2 * pwm * (velMaxima  - 0.1f);
			m_der.setSpeed(pwm_der);
		} 
		else 
		{
			pwm_der = 0.1f + 2 * (pwm - 0.5) * (velMaxima - 0.1f);
			m_der.setSpeed(-pwm_der);
		}
	} 
	else //Ir derecho
	{							
		m_izq.setSpeed(velMaxima);
		m_der.setSpeed(velMaxima);
	}
}

//Derivativo = proporcional - proporcional pasado 
//Proporcional = posicion actual - posicion deseada
//Integral = sumatoria de errores acumulados 
//Velocidad de motores = kp * Derivativo + ki * Integral + kd * Derivativo
void seguirLineaPID(int32_t line_array[], VariablesPID& vp,
					MotorDC &m_izq, MotorDC &m_der, float velMaxima)
{
	vp.errorDerivada = -vp.errorProporcional;
	vp.errorProporcional = obtenerError(line_array);
	vp.errorDerivada += vp.errorProporcional;
	vp.errorIntegral += vp.errorProporcional * vp.dt;

	//Limitar la integral para no causar problemas. 
	if (vp.errorIntegral > 0.01) vp.errorIntegral = 0.01;
	if (vp.errorIntegral < -0.01) vp.errorIntegral = -0.01;

	float pwm = vp.kp * vp.errorProporcional + vp.kd * vp.errorDerivada * 
				vp.idt + vp.ki * vp.errorIntegral;
	motorSpeed(pwm, m_izq, m_der, velMaxima);
}