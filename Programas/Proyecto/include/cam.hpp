#ifndef CAM_HPP
#define CAM_HPP

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/rgbd.hpp>

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

class Camara
{
private:
  double imgC;
  double mD, mI;
  Point bD, bI;
public:
  bool derB=false;
  bool izqB=false;
  bool altoB=false;
  std::vector<KeyPoint> kpA, kpD, kpI, kpE;
  Mat desA, desD, desI, desE;
  std::vector<Point2f> alto, dere, izq, esc;
  std::vector<Point2f> esqE;
  double m;
  //Funciones para adquisición de imágenes
  void mostrarImIntrin(VideoCapture &capturar);
  void mostrarDIntrin(VideoCapture &capturar);
  //Funciones para detección de carril
  Mat filtro(Mat imagen);
  Mat blanco(Mat filtrada);
  Mat borde(Mat blancos);
  Mat RDI(Mat bordes);
  std::vector<Vec4i> detectarL(Mat masc);
  std::vector<std::vector<Vec4i>> separarL(std::vector<Vec4i> lin, Mat bordes);
  std::vector<Point> MinCu(std::vector<std::vector<Vec4i>> lin2, Mat imagen);
  Mat dibujarC(Mat imagen, std::vector<Point> carril);
  //Funciones para detección de señales de tránsito
  void detectarDescO(std::vector<KeyPoint> &kpA, Mat &desA, std::vector<KeyPoint> &kpD, Mat &desD, std::vector<KeyPoint> &kpI, Mat &desI);
  void detectarDescE(Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE);
  std::vector<DMatch> match(Mat &desO, Mat &desE);
  Mat dibujarMatch(Mat imgO, std::vector<KeyPoint> &kpO, Mat imgE, std::vector<KeyPoint> &kpE, std::vector<DMatch> Bmatches);
  Mat locEscena(std::vector<Point2f> &obj, std::vector<KeyPoint> &kpO, std::vector<Point2f> &esc,  std::vector<KeyPoint> &kpE, std::vector<DMatch> Bmatches);
  std::vector<Point2f> limitesO(Size tam, Mat H);
  Mat dibujarO(std::vector<Point2f> esqE, Mat imgM, std::string acc);
  bool Alto(std::vector<KeyPoint> &kpO, Mat &desO, std::vector<Point2f> &obj, std::vector<Point2f> &esc, Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE, std::vector<Point2f> &esqE);
  bool Izquierda(std::vector<KeyPoint> &kpO, Mat &desO, std::vector<Point2f> &obj, std::vector<Point2f> &esc, Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE, std::vector<Point2f> &esqE);
  bool Derecha(std::vector<KeyPoint> &kpO, Mat &desO, std::vector<Point2f> &obj, std::vector<Point2f> &esc, Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE, std::vector<Point2f> &esqE);
  //Funciones para detección del color de un semáforo
  
  //Funciones para profundidad
  double getDist(Mat imD, Point p);
  Mat umbProf(Mat prof, int um);
  //Funciones de salida
  Mat dibujar(Mat imagen, std::vector<Point> carril, std::vector<Point2f> esqE, int tipo, double d1, Point p1, double d2, Point p2);
};

#endif
