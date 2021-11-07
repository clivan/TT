/*
 *Ejemplo sobre la medición de la distancia hasta el punto central de la imagen.
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

int main(int argc, char* argv[])
{
  Camara camara;
  VideoCapture cap(VideoCaptureAPIs::CAP_INTELPERC);
  //VideoWriter grabadora;
  //bool color=true;
  //int codec=VideoWriter::fourcc('M', 'J', 'P', 'G');
  //double fps=10.0;
  //string archivo="./Distancia.avi";
  //grabadora.open(archivo, codec, fps, Size(640, 480), color);
  Mat imD, mostrar;
  //double dist;
  if(!cap.isOpened())
    {
      cerr<<"Error: No se puede iniciar la cámara."<<endl;
      return -1;
    }
  /*if (!grabadora.isOpened());
  {
    cerr<<"Error: No se puede abrir el archivo para grabar."<<endl;
    return -1;
    }*/
  for(;;)
    {
      cap.grab();
      cap.set(CAP_INTELPERC_DEPTH_MAP | CAP_PROP_FPS, 10);
      cap.retrieve(imD, CAP_INTELPERC_DEPTH_MAP);
      mostrar=camara.umbProf(imD, 2400);
      //dist=camara.getDist(imD, Point(240, 320));
      //threshold(imD, mostrar, 2400, 65535, THRESH_BINARY);
      imshow("Profundidad", mostrar);
      //      grabadora.write(mostrar);
      if(waitKey(30)>=0)
	{
	  break;
	}
    }
}