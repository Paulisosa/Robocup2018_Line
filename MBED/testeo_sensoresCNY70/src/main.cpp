#include <mbed.h>
#include <mcp3208.h>

SPI device(PTD2,PTD3,PTD1);
MCP3208 mcp(device,PTD0);
Serial pc(USBTX, USBRX);

const int NUM_CNY70 = 7;
int canales_cny70[NUM_CNY70] = {0,1,2,3,4,5,6};

int main() {
    while(1) {
      for(int i = 0; i < NUM_CNY70; i++)
      {
        float lectura = mcp.read_input(canales_cny70[i]);
        pc.printf("Lectura sensor %i:", i);
        pc.printf("%f\n",lectura);
      }
      pc.printf("-----------------------\n");
      wait(2.0);
    }
}
