#include "seguidor.h"

#ifdef	INVERT_ERROR
const int32_t dis_sensor[]= {DIS2,DIS1,DIS0,-DIS0,-DIS1,-DIS2};
#else
const int32_t dis_sensor[] = { -DIS2 - 10, -DIS1 - 10, -DIS0 - 10,
DIS0, DIS1, DIS2 };
#endif

float calibrar(int32_t array_value[]) {

	int32_t sum_sen = 0;
	int32_t sum = 0;
	int32_t dis = 0;
	static int32_t dis_min = 0;
	static int32_t dis_max = 0;
	static float delta = 0;

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {

		if ((array_value[4]) < 420 || (array_value[5]) < 420) {
			array_value[3] = -array_value[3];
			array_value[4] = -array_value[4];
		}
	}

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {

#ifdef INVERT_SENSORS
		sum_sen+=(MAX_VALUE_SENSOR-array_value[i]);
#else
		sum_sen += array_value[i];
#endif

		sum += (array_value[i] * dis_sensor[i]);
	}

	dis = sum / sum_sen;

	dis_min = MIN(dis_min, dis);
	dis_max = MAX(dis_max, dis);

	delta = (2. * ((float) dis - dis_min)) / (dis_max - dis_min) - 1.;
	printf("sum\t%d\n", sum_sen);
	printf("dist\t%d\t%d\n", dis_min, dis_max);

	return delta;
}

float obtenerError(int32_t array_value[]) {

	int32_t sum_sen = 0;
	int32_t sum = 0;
	int32_t dis = 0;
	static float delta = 0;

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {

#ifdef INVERT_SENSORS
		sum_sen+=(MAX_VALUE_SENSOR-array_value[i]);
#else
		sum_sen += array_value[i];
#endif

		sum += (array_value[i] * dis_sensor[i]);
	}

	//printf("sum\t%d\n",sum_sen);

	dis = sum / sum_sen;

//	if((array_value[5]<300)||(array_value[0]<300)){
//		if (delta > 0.) {
//					delta = 1.;
//
//				} else {
//					delta = -1.;
//
//			}
//	}else{
//		delta = (2. * ((float) dis - DIS_MIN)) / (DIS_MAX - DIS_MIN) - 1.;
//	}

	if (sum_sen > 7300) {
		if (delta > 0.) {
			delta = 1.;

		} else {
			delta = -1.;

		}
		sensor_out_range = true;
	} else {
		delta = (2. * ((float) dis - DIS_MIN)) / (DIS_MAX - DIS_MIN) - 1.;
		sensor_out_range = false;
	}

	//delta = (2. * ((float) dis - DIS_MIN)) / (DIS_MAX - DIS_MIN) - 1.;
	return delta;
}

float ctrlPID(int32_t sensor_value[]) {

	static float err = 0;

	err = obtenerError(sensor_value);

	//computar pid

	return err;
}

void motorSpeed(float pwm, MotorDC &m_izq, MotorDC &m_der) {

	float pwm_der, pwm_izq;

	if (pwm > 1.0)
		pwm = 1.0;
	if (pwm < -1.0)
		pwm = -1.0;

	if (pwm > 0.0) {					// Girar a la izquierda

		m_der.setSpeed(PWM_MAX_DER);

		if (pwm <= 0.5) {
			pwm_izq = PWM_MAX_IZQ - 2 * pwm * (PWM_MAX_IZQ - PWM_MIN_IZQ);
			m_izq.setSpeed(pwm_izq);
		} else {
			pwm_izq = PWM_MIN_IZQ
					+ 2 * (pwm - 0.5) * (PWM_MAX_IZQ - PWM_MIN_IZQ);
			m_izq.setSpeed(-pwm_izq);
		}

	} else if (pwm < 0.0) {		// Girar a la derecha

		m_izq.setSpeed(PWM_MAX_IZQ);
		pwm = -pwm;

		if (pwm <= 0.5) {
			pwm_der = PWM_MAX_DER - 2 * pwm * (PWM_MAX_DER - PWM_MIN_DER);
			m_der.setSpeed(pwm_der);
		} else {
			pwm_der = PWM_MIN_DER
					+ 2 * (pwm - 0.5) * (PWM_MAX_DER - PWM_MIN_DER);
			m_der.setSpeed(-pwm_der);
		}

	} else {							// Ir derecho
		m_izq.setSpeed(PWM_MAX_IZQ);
		m_der.setSpeed(PWM_MAX_DER);
	}

}

void motorSpeed2(float pwm, MotorDC &m_izq, MotorDC &m_der) {

	float pwm_der, pwm_izq;

	if (pwm > 1.0)
		pwm = 1.0;
	if (pwm < -1.0)
		pwm = -1.0;

	if (pwm < 0.0) {					// Girar a la izquierda

		pwm_izq = 0.5 + pwm * (0.5 - 0.0);
		m_izq.setSpeed(pwm_izq);

	} else if (pwm > 0.0) {		// Girar a la derecha

		pwm_der = 0.4 + pwm * (0.4 - 0.0);
		m_der.setSpeed(pwm_der);

	} else {							// Ir derecho
		m_izq.setSpeed(0.0);
		m_der.setSpeed(0.0);
	}

}
