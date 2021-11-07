#ifndef CARRO_HPP
#define CARRO_HPP

#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

class Carro
{
private:
  int uart0=-1;
public:
  void initUART();
  void cerrar();
  void setVal(int val);
};

#endif
