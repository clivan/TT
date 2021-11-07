#include "../include/control.hpp"

using namespace std;

//Método constructror
Control::Control()
{
  eP=0.0;
  eI=0.0;
  eD=0.0;
}

//Método destructor
Control::~Control(){}

//Control::init: Inicializar controlador PID
/*
 *Inicializa las variables globales de la clase con los valores locales dados.
 *Recibe: double Kp, ganancia proporcional.
 *Recibe: double Ki, ganancia integral.
 *Recibe: double Kd, ganancia derivativa.
*/
void Control::init(double Kp, double Ki, double Kd)
{
  Kp_=Kp;
  Ki_=Ki;
  Kd_=Kd;
}

//Control::error: Actualiza los valores del error en el controlador
/*
 *Asigna el nuevo valor de error a cada una de las variables globales.
 *Recibe: double cons, valor de la inclinación actual
*/
void Control::error(double cons)
{
  eI+=cons;
  eD=cons-eP;
  eP=cons;
}

//Control::tErr: Calcula la compensación por el controlador
/*
 *Emplea todos los errores y las ganancias para calcular la compensación.
 *Retorna: double tErr, controlador PID.
*/
double Control::tErr()
{
  return -(Kp_*eP+Ki_*eI+Kd_*eD);
}
