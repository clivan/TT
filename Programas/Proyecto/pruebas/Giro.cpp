//Librerías generales

//Librerías propias
#include "../include/cam.hpp"
//#include "../include/control.hpp"
//#include "../include/imu.hpp"
#include "../include/carro.hpp"
//#include "../include/odom.hpp"

using namespace cv;

int main(int argc, char** argv)
{
  Camara camara;
  Carro carro;
  carro.initUART();
  VideoCapture cap(CAP_INTELPERC);
  Mat imgO, imgE, imgM, H, imgMM, imgMMM, im;
  std::vector<DMatch> Bmatches;
  std::vector<Point2f> esqE;
  bool alto=false, dere=false, izq=false;
  camara.detectarDescO(camara.kpA, camara.desA, camara.kpD, camara.desD, camara.kpI, camara.desI);
  carro.setVal(1565);
  while(1)
    {
      carro.setVal(1600);
      cap.grab();
      cap.set(CAP_INTELPERC_IMAGE | CAP_PROP_FPS, 10);
      cap.retrieve(im, CAP_INTELPERC_IMAGE);
      cvtColor(im, imgE, COLOR_BGR2GRAY);
      //imshow("1", imgE);
      camara.detectarDescE(imgE, camara.kpE, camara.desE);
      alto=camara.Alto(camara.kpA, camara.desA, camara.alto, camara.esc, imgE, camara.kpE, camara.desE, camara.esqE);
      camara.alto.clear();
      camara.esc.clear();
      camara.kpA.clear();
      camara.kpE.clear();
      Bmatches.clear();
      izq=camara.Izquierda(camara.kpI, camara.desI, camara.dere, camara.esc, imgE, camara.kpE, camara.desE, camara.esqE);
      camara.izq.clear();
      camara.esc.clear();
      camara.kpI.clear();
      camara.kpE.clear();
      Bmatches.clear();
      dere=camara.Derecha(camara.kpD, camara.desD, camara.izq, camara.esc, imgE, camara.kpE, camara.desE, camara.esqE);
      camara.dere.clear();
      camara.esc.clear();
      camara.kpD.clear();
      camara.kpE.clear();
      Bmatches.clear();
      if (alto)
	{
	  imgMM=camara.dibujarO(camara.esqE, im, "Alto");
	  carro.setVal(1550);
	}
      if (izq)
	{
	  //cout<<"Izquierda"<<endl;
	  imgMMM=camara.dibujarO(camara.esqE, im, "Izquierda");
	  carro.setVal(2125);
	}
      if (dere)
	{
	  //cout<<"Derecha"<<endl;
	  imgMMM=camara.dibujarO(camara.esqE, im, "Derecha");
	  carro.setVal(2035);
	}
      if (!alto && !izq && !dere)
	{
	  //cout<<"Nada"<<endl;
	  imgMMM=im;
	  carro.setVal(2080);
	}
      imshow("Señales", imgMMM);
      if (waitKey(30)>=0)
	{
	  carro.setVal(2080);
	  break;
	}
      }
  return 0;
}
