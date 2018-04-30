#include "mbed.h"
#include "mcp3208.h"
#include "EV3UARTSensor.h"
#include "SysTimer.h"
#include "seguidor.h"

//Guarda los valores leidos en el circuito integrado MCP3208
//en line_array[]
void actualizarInfrarrojos(int32_t line_array[], MCP3208& mcp)
{
	line_array[0] = mcp.iread_input(5);
	line_array[1] = mcp.iread_input(4);
	line_array[2] = mcp.iread_input(3);
	line_array[3] = mcp.iread_input(2);
	line_array[4] = mcp.iread_input(1);
	line_array[5] = mcp.iread_input(0);
	line_array[6] = mcp.iread_input(6);
}

//Devuelve true si todos los infrarrojos leen blanco 
bool gap(int32_t line_array[])
{
	for(int i = 0; i < 7; i++)
		if(line_array[i] < 950)
			return false;
	return true;
}

//Devuelve true si el sensor cny70 del extre derecho lee linea
// y el sensor cny70 del extre derecho lee negro
bool giro_linea_derecha(int32_t line_array[])
{
	if(line_array[5] < 950 && line_array[0] > 950)
		return true;
	return false;
}

int main() 
{
	//Conexiones SPI y MCP3208: 
	SPI device(PTD2, PTD3, PTD1);
	MCP3208 mcp(device, PTD0);

	//Motores:
	DigitalOut en_motors(PTB20, 1);
	DigitalIn sw(SW2);
	MotorDC mizq(PTC11, PTC5, PTC7);
	MotorDC mder(PTC10, PTC9, PTC0);

	//Array para guardar las lecturas
	//de los sensores infrarrojos: 
	int32_t line_array[7];

	//Comenzar cronometro: 
	initSystemClock();
	volatile uint32_t* tiempoActual = getPointerMillis();
	delay_ms(100);
	uint32_t tiempoInicio = *tiempoActual;
	
	//Variables PID: 
	VariablesPID vp;

	while (true) 
	{
		actualizarInfrarrojos(line_array,mcp);
		if(gap(line_array))
		{  // Avanzar hasta que detecte linea 
			mizq.setSpeed(PWM_MAX_IZQ);
			mder.setSpeed(PWM_MAX_DER);
		}
		
		else
		{
			seguirLineaPID(line_array, 0.8f,0.05f,0.2f,vp,mizq,mder);    
		}	         
		while ((*tiempoActual - tiempoInicio) < 10) // Esperar 10 milisegundos antes 
         {}                                     	//de realizar otra accion 
		 tiempoInicio = *tiempoActual;	 
	} 
} 