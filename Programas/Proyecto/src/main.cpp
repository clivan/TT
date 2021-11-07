//Librerías generales del sistema

//Librerías propias
#include "../include/cam.hpp"
#include "../include/control.hpp"
#include "../include/imu.hpp"
#include "../include/carro.hpp"
//#include "../include/odom.hpp"

using namespace cv;
using std::cerr;
using std::cout;
using std::cin;
using std::endl;

int main(int argc, char** argv)
{
  Camara camara;
  Control control;
  Carro carro;
  //VideoCapture capturar(0);
  VideoCapture capturar(CAP_INTELPERC);
  VideoWriter grabadoraRGB, grabadoraD, grabadoraIR, grabadoraP; //Declarar VideoWriter
  //Parámetros de la grabadora
  double color=true;
  bool bn=false;
  int codec=VideoWriter::fourcc('M', 'J', 'P', 'G');
  double fps=15.0;
  //Archivos de salida
  string archivo="../salidas/Color.avi";
  string archivo2="../salidas/Profundidad.avi";
  string archivo3="../salidas/IR.avi";
  string archivo4="../salidas/Salida.avi";
  //Inicializar grabadoras
  grabadoraRGB.open(archivo, codec, fps, Size(640, 480), color);
  grabadoraD.open(archivo2, codec, fps, Size(640, 480), color);
  grabadoraIR.open(archivo3, codec, fps, Size(640, 480), color);
  grabadoraP.open(archivo4, codec, fps, Size(640, 480), bn);
  //capturar.set(CAP_PROP_FRAME_WIDTH, 640.0);
  //capturar.set(CAP_PROP_FRAME_HEIGHT, 480.0);
  //Parámetros del PID
  double kp=0.1;
  double ki=0.0001;
  double kd=1.0;
  double pend, ang;
  control.init(kp, ki, kd);
  Mat imRGB, imIR, imD; //Imágenes adquiridas.
  Mat filtrada, bordes, blancos, masc, imagenL; //Imágenes para detección de carril.
  Mat imgO, imgE, imgM, H, imgMM; //Imágenes para detección se señalética.
  //Parámetros del algoritmo SURF
  std::vector<DMatch> Bmatches;
  std::vector<Point2f> esqE;
  //String Izquierda("../Figuras/Izquierda.png");
  //String Derecha("../Figuras/Derecha.png");
  //String Alto("../Figuras/Alto.png");
  if (!capturar.isOpened())
    {
      cerr<<"Error: No se puede iniciar la cámara."<<endl;
      return -1;
    }
  if (!grabadoraRGB.isOpened())
    {
      cerr<<"Error: No se puede abrir el archivo para grabar"<<endl;
      return -1;
    }
  if (!grabadoraD.isOpened())
    {
      cerr<<"Error: No se puede abrir el archivo para grabar"<<endl;
      return -1;
    }
  if (!grabadoraIR.isOpened())
    {
      cerr<<"Error: No se puede abrir el archivo para grabar"<<endl;
      return -1;
    }
  if (!grabadoraP.isOpened())
    {
      cerr<<"Error: No se puede abrir el archivo para grabar"<<endl;
      return -1;
    }
  for(;;)
    {
      //capturar>>imRGB;
      capturar.grab();
      capturar.retrieve(imRGB, CAP_INTELPERC_IMAGE);
      capturar.retrieve(imIR, CAP_INTELPERC_IR_MAP);
      capturar.retrieve(imD, CAP_INTELPERC_DEPTH_MAP);
      equalizeHist(imIR, imIR);
      imD.convertTo(imD, CV_8UC1, -255.0/10000.0, 255.0);
      applyColorMap(imIR, imIR, COLORMAP_JET);
      //      equalizeHist(imD, imD);
      applyColorMap(imD, imD, COLORMAP_JET);
      grabadoraRGB.write(imRGB);
      imshow("Color", imRGB);
      grabadoraD.write(imD);
      imshow("Profundidad", imD);
      grabadoraIR.write(imIR);
      imshow("IR", imIR);
      filtrada=camara.filtro(imRGB);
      blancos=camara.blanco(imRGB);
      bordes=camara.borde(filtrada/*blancos*/);
      imshow("Procesada", bordes);
      grabadoraP.write(bordes);
      if (waitKey(30)>=0)
	{
	  break;
	}
    }
  return 0;
}
