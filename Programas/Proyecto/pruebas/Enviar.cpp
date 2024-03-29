//Librerías generales
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <string>
//Librerías propias
//#include "../include/cam.hpp"
//#include "../include/control.hpp"
//#include "../include/imu.hpp"
#include "../include/carro.hpp"
//#include "../include/odom.hpp"

int main(int argc, char** argv)
{
  Carro carro;
  char *endptr;
  carro.initUART();
  int servo;
  servo=std::strtol(argv[1], &endptr, 10);
  carro.setVal(servo);
  std::cout<<servo<<std::endl;
  //carro.setVal(1565); //1565 para velocidad mínima
  /*for(int i=0; i<5; i++)
    {
      carro.setVal(2075);
      sleep(1);
      carro.setVal(2090);
      sleep(1);
      carro.setVal(2125);
      sleep(1);
      }*/
  carro.cerrar();
  return 0;
}
