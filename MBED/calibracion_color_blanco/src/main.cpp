//Codigo de calibracion del color blanco usando sensor CNY70.
//Colocar debajo del sensor un objeto de color blanco.
//Asegurarse de que llega luz y que las conexiones sean correctas.

#include <mbed.h>

Serial pc(USBTX, USBRX);
AnalogIn cny70(PTC10); // cambiar Pin si es necesario

int main() {
    while(1) {
      float lectura = cny70.read();
      pc.printf("Lectura: %f/n", lectura);
      wait(1.0);
    }
}
