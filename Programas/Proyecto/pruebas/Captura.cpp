/*
 *Ejemplo sobre la implementación de la captura de imágenes usando VideoCapture.
 *Se muestra la imagen original en una ventana y en la otra se muestran los bordes de la imagen.
*/

//Librerías generales

//Librerías propias
#include "../include/cam.hpp"
//#include "../include/control.hpp"
//#include "../include/imu.hpp"
//#include "../include/carro.hpp"
//#include "../include/odom.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  Camara camara;
  /*VideoCapture capturar(0);*/
  VideoCapture capturar(CAP_INTELPERC);
  Mat imRGB, imIR, imD;
  if (!capturar.isOpened())
    {
      cerr<<"Error: No se puede iniciar la cámara."<<endl;
      return -1;
    }
  for(;;)
    {
      /*capturar>>imRGB;
      if (imRGB.empty())
	{
	  cerr<<"Error: Cuadro en blanco."<<endl;
	  break;
	  }*/
      capturar.grab();
      capturar.set(CAP_INTELPERC_IMAGE | CAP_PROP_FPS, 10);
      capturar.set(CAP_INTELPERC_IR_MAP | CAP_PROP_FPS, 10);
      capturar.set(CAP_INTELPERC_DEPTH_MAP | CAP_PROP_FPS, 10);
      capturar.retrieve(imRGB, CAP_INTELPERC_IMAGE);
      //capturar.retrieve(imIR, CAP_INTELPERC_IR_MAP);
      //capturar.retrieve(imD, CAP_INTELPERC_DEPTH_MAP);
      //equalizeHist(imIR, imIR);
      //imD.convertTo(imD, CV_8UC1, -255.0/10000.0, 255.0);
      //      applyColorMap(imD, imD, COLORMAP_JET);
      //applyColorMap(imIR, imIR, COLORMAP_JET);
      imshow("Color", imRGB);
      //imshow("IR", imIR);
      //imshow("Profundidad", imD);
      if (waitKey(30)>=0)
	{
	  break;
	}
    }
  return 0;
}
