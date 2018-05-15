#include "MotorDC.h"
#include "mbed.h"
#include "mcp3208.h"
#include "EV3UARTSensor.h"
#include "SysTimer.h"
#include "hcsr04.h"

//Estructura que contiene las variables necesarias para el correcto funcionamiento
//del algoritmo PID: 
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
	float kp; 	//Factor proporcional
	float kd;	//Factor derivada
	float ki;   //Factor integral
	float errorProporcional;
	float errorDerivada;	  
	float errorIntegral;      
	float idt;
	float dt;
};

//Obtiene la posicion del robot en base a la linea: 
int32_t obtenerPosicion(int32_t valoresArray[], int32_t dis_sensor[], const int32_t& numeroSensores, int32_t& sumaSensores)
{
	sumaSensores = 0;
	int32_t suma = 0;
	int32_t posicion = 0;
	
	for (uint8_t i = 0; i < numeroSensores; i++) 
	{
		sumaSensores += valoresArray[i];
		suma += (valoresArray[i] * dis_sensor[i]);
	}
	posicion = suma / sumaSensores;
	return posicion;
}

float obtenerError(const int32_t& posicion, const int32_t& sumaSensores, const float& disMin, const float& disMax) 
{
	static float delta = 0; 
	delta = (2. * ((float) posicion - disMin)) / (disMax - disMin) - 1.;
	return delta;
}

//Derivativo = proporcional - proporcional pasado 
//Proporcional = posicion actual - posicion deseada
//Integral = sumatoria de errores acumulados 
//Velocidad de motores = kp * Derivativo + ki * Integral + kd * Derivativo
float obtenerPID(float desviacionLinea, VariablesPID& vp)
{
	vp.errorDerivada = -vp.errorProporcional;
	vp.errorProporcional = desviacionLinea; 
	vp.errorDerivada += vp.errorProporcional;
	vp.errorIntegral += vp.errorProporcional * vp.dt;

	//Limitar la integral para no causar problemas. 
	if (vp.errorIntegral > 0.01) vp.errorIntegral = 0.01;
	if (vp.errorIntegral < -0.01) vp.errorIntegral = -0.01;

	float pwm = vp.kp * vp.errorProporcional + vp.kd * vp.errorDerivada * 
				vp.idt + vp.ki * vp.errorIntegral;
	return pwm;
}

void setearVelocidades(float pwm, MotorDC &m_izq, MotorDC &m_der, float velMaxima, float velMinima)
{
	float pwmSlow;

	//Limitar la salida de pwm 
	if (pwm > 1.0)	pwm = 1.0;
	if (pwm < -1.0) pwm = -1.0;

	if(fabs(pwm) <= 0.5)
		pwmSlow =  velMaxima - 2 * fabs(pwm) * (velMaxima - velMinima);
	else 
	{
		pwmSlow =  velMinima + 2 * (fabs(pwm) - 0.5) * (velMaxima - velMinima);
		pwmSlow = -pwmSlow;
	}
	if(pwm > 0.0)
	{
		m_der.setSpeed(velMaxima);
		m_izq.setSpeed(pwmSlow);
	}
	else if (pwm < 0.0)
	{
		m_izq.setSpeed(velMaxima);
		m_der.setSpeed(pwmSlow);
	}
	else
	{
		m_izq.setSpeed(velMaxima);
		m_der.setSpeed(velMaxima);
	}
}

//Guarda los valores leidos en el circuito integrado MCP3208
//en valoresArray[]
void actualizarInfrarrojos(int32_t valoresArray[], MCP3208& mcp)
{
	valoresArray[3] = mcp.iread_input(5);
	valoresArray[2] = mcp.iread_input(4);
	valoresArray[1] = mcp.iread_input(3);
	valoresArray[0] = mcp.iread_input(2);
	valoresArray[6] = mcp.iread_input(1);
	valoresArray[5] = mcp.iread_input(0);
	valoresArray[4] = mcp.iread_input(6);
}

//Actualiza el valor de sampleIzq y sampleDer mediante la lectura de los sensores EV3
void actualizarSensoresLego(EV3UARTSensor& sensorIzq, EV3UARTSensor& sensorDer,
							float sampleIzq[],  float sampleDer[])
{
	sensorIzq.check_for_data();
	sensorDer.check_for_data();
	sensorIzq.fetch_sample(sampleIzq, 0);
	sensorDer.fetch_sample(sampleDer, 0);
}

//Devuelve true si el sensor cny70 del extre derecho lee linea
// y el sensor cny70 del extre derecho lee negro
bool giro_pronunciado(int32_t valoresArray[], int blanco)
{
	if(valoresArray[5] < blanco || valoresArray[0] < blanco ||  
	   valoresArray[1] < blanco || valoresArray[4] < blanco)
		return true;
	return false;
}

bool giro_pronunciado_izquierda(float sampleIzq[], float sampleDer[])
{
	if(sampleIzq[0] == 1.0 && sampleDer[0] != 1.0) 
		return true;
	return false;
}

bool giro_pronunciado_derecha(float sampleIzq[], float sampleDer[])
{
	if(sampleIzq[0] != 1.0 && sampleDer[0] == 1.0) 
		return true;
	return false;
}

//Devuelve true si alguno de los sensores Lego leen negro. 
bool giro_pronunciado(float sampleIzq[], float sampleDer[])
{
	if(sampleIzq[0] == 1.0 || sampleDer[0] == 1.0)
		return true;
	return false;
}

//Printea los valores de valoresArray para calibrar sensores CNY70
void printear_infrarrojos(int32_t valoresArray[], Serial& pc)
{
	pc.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n", valoresArray[0], valoresArray[1],
			valoresArray[2], valoresArray[3], valoresArray[4], valoresArray[5],
			valoresArray[6]);
}

//Devuelve true si los dos sensores EV3 leen verde
bool giro_atras(float sampleIzq[],  float sampleDer[])
{
	if (sampleIzq[0] == 3.0 && sampleDer[0] == 3.0)
		return true; 
	return false;
}

//Devuelve true si el sensor izquierda
bool interseccion_izquierda(float sampleIzq[],  float sampleDer[])
{
	if (sampleIzq[0] == 3.0 && sampleDer[0] != 3.0)
		return true; 
	return false;
}
bool interseccion_derecha(float sampleIzq[],  float sampleDer[])
{
	if (sampleIzq[0] != 3.0 && sampleDer[0] == 3.0)
		return true; 
	return false;
}

void girar_izquierda(MotorDC& mizq, MotorDC& mder, MCP3208& mcp,
				EV3UARTSensor& sensorIzq, EV3UARTSensor& sensorDer, float sampleIzq[],  float sampleDer[])
{
	//Avanzar hasta que pase la linea asi el robot realiza un buen giro 
	while(sampleIzq[0] != 1.0 ) // Avanzar mientras este en el cuadrado verde
	{
		mder.setSpeed(0.05f);
		mizq.setSpeed(0.05f);
		actualizarSensoresLego(sensorIzq,sensorDer,sampleIzq,sampleDer);
	}
	while(sampleIzq[0] == 1.0 || sampleIzq[0] == 2.0) // Avanzar mientras este en la linea negra
	{
		mder.setSpeed(0.05f);
		mizq.setSpeed(0.05f);
		actualizarSensoresLego(sensorIzq,sensorDer,sampleIzq,sampleDer);
	}
	while(sampleIzq[0] != 1.0) //Girar mientras sensor izquierdo no este en linea 
	{
		mder.setSpeed(0.4f);
		mizq.setSpeed(-0.4f);
		actualizarSensoresLego(sensorIzq,sensorDer,sampleIzq,sampleDer);
	}
	while(sampleIzq[0] == 1.0) //Girar mientras sensor izquierdo este en linea 
	{
		mder.setSpeed(0.4f);
		mizq.setSpeed(-0.4f);
		actualizarSensoresLego(sensorIzq,sensorDer,sampleIzq,sampleDer);
	}
	while(sampleDer[0] != 1.0) //Girar mientras sensor externo derecho no este en inea
	{
		mder.setSpeed(0.4f);
		mizq.setSpeed(-0.4f);
		actualizarSensoresLego(sensorIzq,sensorDer,sampleIzq,sampleDer);
	}
}

void girar_atras(MotorDC& mizq, MotorDC& mder, MCP3208& mcp,
				EV3UARTSensor& sensorIzq, EV3UARTSensor& sensorDer, float sampleIzq[],  float sampleDer[])
{
	while(sampleIzq[0] != 1.0)
	{
		mder.setSpeed(0.3f);
		mizq.setSpeed(-0.3f);
		actualizarSensoresLego(sensorIzq,sensorDer,sampleIzq,sampleDer);
	}
}

void esquivarObstaculo(MotorDC& mizq, MotorDC& mder, HCSR04& ultraIzq)
{
	delay_ms(500);
	int32_t lectura = ultraIzq.distance();
	mizq.setSpeed(0.6);
	mder.setSpeed(-0.6);
	while(lectura>20)
		lectura = ultraIzq.distance();
	
	while(1)
	{
		int32_t obj = 10;
		lectura = ultraIzq.distance();
		int32_t diferencia = obj - lectura;
		if(obj-lectura== 0) 
		{
			mizq.setSpeed(0.5f);
			mder.setSpeed(0.5f);
		}
		else if(obj-lectura > 0)
		{
			mizq.setSpeed(0.6f);
			mder.setSpeed(0);
		}
		else
		{
			mizq.setSpeed(0);
			mder.setSpeed(0.6f);
		}
	}
}

int main() 
{
	//Conexion serial:
	Serial pc(USBTX, USBRX);
	pc.baud(9600);

	//Conexiones SPI y MCP3208: 
	SPI device(PTD2, PTD3, PTD1);
	MCP3208 mcp(device, PTD0);

	//Motores:
	DigitalOut en_motors(PTB20, 1);
	DigitalIn sw(SW2);
	MotorDC mizq(PTC11, PTC5, PTC7);
	MotorDC mder(PTC10, PTC9, PTC0);

	//Arrays para guardar las lecturas
	//de los sensores infrarrojos: 
	const int32_t DIS0 = 55;
	const int32_t DIS1 = 165;
	const int32_t DIS2 = 275;
	const int32_t DIS_MAX	= 73;
	const int32_t DIS_MIN = -65;
	const int32_t numeroSensores = 6;
	int32_t valoresArray[7];
	int32_t dis_sensor[] = { -DIS2 - 10, -DIS1 - 10, -DIS0 - 10, DIS0, DIS1, DIS2 };

	//Comenzar cronometro: 
	initSystemClock();
	volatile uint32_t* tiempoActual = getPointerMillis();
	delay_ms(100);
	uint32_t tiempoInicio = *tiempoActual;
	
	//Conexiones sensores Lego:     
	RawSerial serialIzq(PTC15, PTC14, 2400);
    RawSerial serialDer(PTC17, PTC16, 2400);
    
    DigitalOut ledr(LED_RED, 1);
    DigitalOut ledg(LED_GREEN, 1);

    EV3UARTSensor sensorIzq;
    EV3UARTSensor sensorDer;
	float sampleIzq[5]; 
    float sampleDer[5]; 
    
    sensorIzq.begin(serialIzq);
    sensorDer.begin(serialDer);
    sensorIzq.connect(ledg);
    sensorDer.connect(ledr);
    sensorIzq.set_mode(ColColor);
    sensorDer.set_mode(ColColor);  

	//Variables PID: 
	VariablesPID vp(0.9f,0.06f,0.6f);
	int32_t posicion = 0; 	  //Posicion del robot en base a la linea 
	int32_t sumaSensores = 0; //Suma de las lecturas de los sensores CNY70
	float error = 0.0f;  	  //Error proporcional del calculo PID
	float pwm = 0.0f;    	  //Valor del calculo de PID

	//Ultrasonidos:
	HCSR04 ultraIzq(D3,D4);

	while (true) 
	{	
		posicion = obtenerPosicion(valoresArray,dis_sensor,5,sumaSensores);
		error = obtenerError(posicion,sumaSensores,DIS_MIN,DIS_MAX);
		pwm = obtenerPID(error,vp);
		setearVelocidades(pwm,mizq,mder,0.6f,0.1f);
		tiempoInicio = *tiempoActual;
		while(*tiempoActual-tiempoInicio<10)
			actualizarSensoresLego(sensorIzq,sensorDer,sampleIzq,sampleDer);
	} 
} 