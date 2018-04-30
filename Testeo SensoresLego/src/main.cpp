#include <mbed.h>
#include "EV3UARTSensor.h"
#include "SysTimer.h"

int main() {
    DigitalOut ledr(LED_RED, 1);
    DigitalOut ledg(LED_GREEN, 1);

    EV3UARTSensor sensorIzq;
    EV3UARTSensor sensorDer;

    sensorIzq.connect(ledr); 
	sensorDer.connect(ledg); 

    sensorIzq.set_mode(ColColor);
	sensorDer.set_mode(ColColor);

    float sampleDer[5]; 
    float sampleIzq[5]; 
    
    initSystemClock();

    Serial pc(USBTX, USBRX);
    pc.baud(9600);

    while(1) 
    {
        sensorIzq.check_for_data();
		sensorDer.check_for_data();
		sensorIzq.fetch_sample(sampleIzq, 0);
        sensorDer.fetch_sample(sampleDer, 0);
        pc.printf("%.1f\t%.1f\n", sampleIzq[0], sampleDer[0]);
        delay_ms(500);
    }
}