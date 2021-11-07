#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <iostream>
#include <vector>

class Control
{
public:
  //Errores
  double eP;
  double eI;
  double eD;
  //Coeficientes
  double Kp_;
  double Ki_;
  double Kd_;
  //Constructor
  Control();
  //Destructor
  virtual ~Control();
  void init(double Kp, double Ki, double Kd);
  void error(double cons);
  double tErr();
};

#endif
