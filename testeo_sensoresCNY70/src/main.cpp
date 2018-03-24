#include <mbed.h>
#include <mcp3208.h>

SPI device(PTD2,PTD3,PTD1);
MCP3208 mcp(device,PTD0);
Serial pc(USBTX, USBRX);

const int canal_CNY70 = 0;

int main()
{
  while(true)
  {
      int lectura = mcp.read_int_input(canal_CNY70);
      pc.printf("Lectura: %d\n", lectura);
      wait(2.0);
    }
}
