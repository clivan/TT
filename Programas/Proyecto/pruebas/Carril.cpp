
/*
 *Ejemplo sobre la implementación de la detección de carril.
*/

//Librerías generales

//Librerías propias
#include "../include/cam.hpp"
//#include "../include/control.hpp"
//#include "../include/imu.hpp"
//#include "../include/carro.hpp"
//#include "../include/odom.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  Camara camara;
  VideoCapture capturar(CAP_INTELPERC);
  Mat imagen, filtrada, bordes, blancos, masc, imagenL;
  vector<Vec4i> lin;
  vector<vector<Vec4i>> lin2;
  vector<Point> carril;
  if (!capturar.isOpened())
    {
      cerr<<"Error: No se puede iniciar la cámara."<<endl;
      return -1;
    }
  for(;;)
    {
      capturar.grab();
      capturar.set(CAP_INTELPERC_IMAGE, 30);
      capturar.retrieve(imagen, CAP_INTELPERC_IMAGE);
      //      imshow("Original", imagen);
      filtrada=camara.filtro(imagen);
      //      imshow("Filtrada", filtrada);
      blancos=camara.blanco(filtrada);
      bordes=camara.borde(filtrada);
      //      imshow("Bordes", bordes);
      masc=camara.RDI(bordes);
      //imshow("RDI", masc);
      lin=camara.detectarL(masc);
      lin2=camara.separarL(lin, masc);
      carril=camara.MinCu(lin2, imagen);
      imagenL=camara.dibujarC(imagen, carril);
      imshow("Carril", imagenL);
      if (waitKey(30)>=0)
	{
	  break;
	}
    }
  return 0;
}
