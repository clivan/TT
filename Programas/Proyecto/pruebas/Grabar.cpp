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
  //VideoCapture capturar(0);
  VideoWriter grabadora, grabadoraD, grabadoraIR;
  bool color=true;
  bool bn=false;
  int codec=VideoWriter::fourcc('M', 'J', 'P', 'G');
  double fps=25.0;
  string archivo="./Salida.avi";
  string archivo2="./SalidaD.avi";
  string archivo3="./SalidaIR.avi";
  grabadora.open(archivo, codec, fps, Size(640, 480), color);
  grabadoraD.open(archivo2, codec, fps, Size(640, 480), color);
  grabadoraIR.open(archivo3, codec, fps, Size(640, 480), color);
  //capturar.set(CAP_PROP_FRAME_WIDTH, 640.0);
  //capturar.set(CAP_PROP_FRAME_HEIGHT, 480.0);
  VideoCapture capturar(CAP_INTELPERC);
  Mat imRGB, imIR, imD;
  if (!capturar.isOpened())
    {
      cerr<<"Error: No se puede iniciar la cámara."<<endl;
      return -1;
    }
  if (!grabadora.isOpened())
    {
      cerr<<"No se puede abrir el archivo para grabar"<<endl;
      return -1;
    }
  if (!grabadoraD.isOpened())
    {
      cerr<<"No se puede abrir el archivo para grabar"<<endl;
      return -1;
    }
  if (!grabadoraIR.isOpened())
    {
      cerr<<"No se puede abrir el archivo para grabar"<<endl;
      return -1;
    }
  cout<<"Escribiendo "<<archivo<<" y "<<archivo2<<endl<<"Presione cuaquier tecla para detener"<<endl;
  for(;;)
    {
      //capturar>>imRGB;
      capturar.grab();
      capturar.retrieve(imRGB, CAP_INTELPERC_IMAGE);
      capturar.retrieve(imIR, CAP_INTELPERC_IR_MAP);
      capturar.retrieve(imD, CAP_INTELPERC_DEPTH_MAP);
      imD.convertTo(imD, CV_8UC3, -255.0/10000.0, 255.0);
      equalizeHist(imIR, imIR);
      applyColorMap(imIR, imIR, COLORMAP_JET);
      if (imRGB.empty())
	{
	  cerr<<"Error: Cuadro en blanco."<<endl;
	  break;
	}
      grabadora.write(imRGB);
      imshow("Color", imRGB);
      imshow("Profundidad", imD);
      imshow("IR", imIR);
      grabadoraD.write(imD);
      grabadoraIR.write(imIR);
      if (waitKey(30)>=0)
	{
	  break;
	}
    }
  return 0;
}
