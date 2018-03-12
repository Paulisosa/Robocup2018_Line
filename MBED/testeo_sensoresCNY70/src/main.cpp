/*
  Codigo de testeo de sensores CNY70:

  Colocar todos los sensores a testear debajo
  de un objeto de color blanco. Asegurarse de que
  llega luz y que las conexiones sean correctas.
*/

#include <mbed.h>

const int blanco = 200; //valor obtenido con el proyecto calibracion_color_blanco
const int num_sensores = 6; //numero de sensores a testear
Serial pc(USBTX, USBRX);
AnalogIn sensores_color[num_sensores] = {A0,A1,A2,A3,A4,A5}; //colocar los pines necesarios

int main() {
    while(1) {
      for(int i = 0; i < num_sensores; i++)
      {
        if(sensores_color[i].read() >= 200)
          pc.printf("Sensor %d: lee blanco\n", i+1);
        else
          pc.printf("Sensor %d: no lee blanco\n", i+1);
      }
      pc.printf("------------------------\n");
      wait(1.0);
    }
}
