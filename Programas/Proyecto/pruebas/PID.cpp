//Librerías generales

//Librerías propias
#include "../include/cam.hpp"
#include "../include/control.hpp"
//#include "../include/imu.hpp"
#include "../include/carro.hpp"
//#include "../include/odom.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  Camara camara;
  Control pid;
  Carro carro;
  VideoCapture capturar(CAP_INTELPERC);
  //VideoWriter grabadora;
  //String archivo="./Salida.avi";
  //int codec=VideoWriter::fourcc('M', 'J', 'P', 'G');
  bool color=true;
  double kp=0.4;
  double ki=0.00;
  double kd=0.5;
  double pend, ang, err, set;
  int pwm;
  double dist, dist2, d;
  pid.init(kp, ki, kd);
  Mat imagen, imD, filtrada, bordes, blancos, masc, imagenL, imgO, imgE, imgM, H, imgMM, imgMMM, im;
  vector<Vec4i> lin;
  vector<vector<Vec4i>> lin2;
  vector<Point> carril;
  //grabadora.open(archivo, codec, 30, Size(640,480), color);
  if (!capturar.isOpened())
    {
      cerr<<"Error: No se puede iniciar la cámara."<<endl;
      return -1;
    }
  set=80.0;
  carro.initUART();
  // camara.detectarDescO(camara.kpA, camara.desA, camara.kpD, camara.desD, camara.kpI, camara.desI);
  carro.setVal(1565);
  char *endptr;
  int vel=std::strtol(argv[1], &endptr, 10);
  //cout<<vel<<endl;
  for(;;)
    {
      carro.setVal(vel); //1610 velocidad mínima para arrancar
      capturar.grab();
      capturar.set(CAP_INTELPERC_IMAGE | CAP_PROP_FPS, 10);
      capturar.retrieve(imagen, CAP_INTELPERC_IMAGE);
      //      capturar.retrieve(imD, CAP_INTELPERC_DEPTH_MAP);
      //imshow("Original", imagen);
      filtrada=camara.filtro(imagen);
      //cvtColor(filtrada, imgE, COLOR_BGR2GRAY);
      //camara.detectarDescE(imgE, camara.kpE, camara.desE);
      //bool alto=camara.Alto(camara.kpA, camara.desA, camara.alto, camara.esc, imgE, camara.kpE, camara.desE, camara.esqE);
      //if (alto)
      //{
      //cout<<"Alto"<<endl;
      //carro.setVal(1500);
      //break;
      //}
      bordes=camara.borde(filtrada);
      masc=camara.RDI(bordes);
      lin=camara.detectarL(masc);
      lin2=camara.separarL(lin, masc);
      carril=camara.MinCu(lin2, imagen);
      pend=camara.m+80;
      pid.error(set-pend);
      ang=pid.tErr()+80;
      imagenL=camara.dibujarC(imagen, carril);
      imshow("Carril", imagenL);
      //      grabadora.write(imagenL);
      //      dist=camara.getDistancia(imD, Point(carril[1].x-carril[3].x, carril[1].y));
      //dist2=camara.getDistancia(imD, Point(carril[0].x-carril[2].x, carril[0].y));
      //d=dist-dist2;
      //cout<<"Distancia fin: "<<dist<<endl;
      //cout<<"Distancia inicio: "<<dist2<<endl;
      //cout<<"Distancia: "<<d<<endl;
      //      ang=90-ang;
      //      ang=180-ang;
      if (ang>125)
	{
	  ang=125;
	}
      else if (ang<35)
	{
	  ang=35;
	}
      else
	{
	  ang=ang;
	}
      //      cout<<"Actual: "<<pend<<endl;
      //      cout<<"Corregido: "<<ang<<endl;
      ang+=2000;
      pwm=(int)ang;
      carro.setVal(pwm);
      //cout<<pwm<<endl;
      if (waitKey(30)>=0)
	{
	  break;
	}
    }
  carro.setVal(1550);
  carro.cerrar();
  return 0;
}
