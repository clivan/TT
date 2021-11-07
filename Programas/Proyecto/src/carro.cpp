#include "../include/carro.hpp"

void Carro::initUART()
{
  uart0=open("/dev/ttyS2", O_WRONLY/*| O_APPEND*/); //Sólo escritura | adjuntar la nueva información al final del archivo
  if (uart0==-1)
    {
      cout<<"Error: No se puede acceder al UART. Asegurarse de que no se encuentra en uso"<<endl;
    }
  struct termios options;
  tcgetattr(uart0, &options);
  options.c_cflag=B115200 | CS8 | CLOCAL; //9600 baudios | 8 bits | fuente local
  options.c_iflag=IGNPAR;
  options.c_oflag=0;
  options.c_lflag=0;
  tcflush(uart0, TCIFLUSH);
  tcsetattr(uart0, TCSANOW, &options);
}

void Carro::cerrar()
{
  close(uart0);
}

void Carro::setVal(int val)
{
  uint8_t dig[3];
  dig[0]=val/1000;
  dig[1]=(val/100)%10;
  dig[2]=(val/10)%10;
  dig[3]=val%10;
  write(uart0, dig, sizeof(val));
}
