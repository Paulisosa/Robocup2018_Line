//Codigo de testeo del sensor HCSR04.
//Asegurarse de las conexiones sean correctas.

#include <mbed.h>
#include <hcsr04.h>

Serial pc(USBTX, USBRX);
HCSR04 ultrasonido(D10,D10);

int main() {
    while(1) {
        long lectura = ultrasonido.distance();
        pc.printf("Lectura: %f/n", lectura);
        wait(1.0);
    }
}
