#include "../include/cam.hpp"
#include <cmath>

void Camara::mostrarImIntrin(VideoCapture &capturar)
{
  cout<<"Parámetros de la imagen a color"<<endl;
  cout<<"Brillo: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_BRIGHTNESS)<<endl;
  cout<<"Contrase: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_CONTRAST)<<endl;
  cout<<"Saturación: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_SATURATION)<<endl;
  cout<<"Tono: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_HUE)<<endl;
  cout<<"Gama: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_GAMMA)<<endl;
  cout<<"Nitidez: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_SHARPNESS)<<endl;
  cout<<"Apertura: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_GAIN)<<endl;
  cout<<"Contraluz: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_BACKLIGHT)<<endl;
  cout<<"Anchura: "<<(int)capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_FRAME_WIDTH);
  cout<<"Altura: "<<(int)capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_FRAME_HEIGHT);
  cout<<"FPS: "<<capturar.get(CAP_INTELPERC_IMAGE_GENERATOR | CAP_PROP_FPS)<< endl;
}

void Camara::mostrarDIntrin(VideoCapture &capturar)
{
  cout<<"Valor de baja confianza: "<<capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_INTELPERC_DEPTH_LOW_CONFIDENCE_VALUE)<<endl;
  cout<<"Saturación: "<<capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_INTELPERC_DEPTH_SATURATION_VALUE)<<endl;
  cout<<"Umbral de confianza: "<<capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_INTELPERC_DEPTH_CONFIDENCE_THRESHOLD)<<endl;
  cout<<"Distancia focal: ("<<capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_INTELPERC_DEPTH_FOCAL_LENGTH_HORZ)<<", "<<capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_INTELPERC_DEPTH_FOCAL_LENGTH_VERT)<<")"<<endl;
  cout<<"Anchura: "<<(int)capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_FRAME_WIDTH)<<endl;
  cout<<"Altura: "<<(int)capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_FRAME_HEIGHT)<<endl;
  cout<<"FPS: "<<capturar.get(CAP_INTELPERC_DEPTH_GENERATOR | CAP_PROP_FPS)<<endl;
}

//Camara::filtro: Filtrado de imagen
/*
 *Aplicar filtro gaussiano a la imagen de entrada para remover el ruido.
 *Recibe: cv::Mat imagen (CV_8UC1 u CV_8UC3), imagen de entrada.
 *Retorna: cv::Mat filtrada (CV_8UC1 u CV_8UC3), imagen de salida filtrada.
*/
Mat Camara::filtro(Mat imagen)
{
  Mat filtrada;
  GaussianBlur(imagen, filtrada, Size(5, 5), 0, 0);
  return filtrada;
}

//Camara::blanco: Detectar las líneas blancas
/*
 *Convierte la imagen a HSV para detectar las líneas blancas de la imagen.
 *Recibe: cv::Mat imagen (CV_8UC3), imagen de entrada (de preferencia filtrada).
 *Retorna: cv::Mat blanco (CV_8UC1), imagen binarizada con las líneas blancas.
*/
Mat Camara::blanco(Mat filtrada)
{
  Mat ihsv, blancos;
  cvtColor(filtrada, ihsv, COLOR_BGR2HSV);
  inRange(ihsv, Scalar(0, 0, 150), Scalar(50, 50, 255), blancos);
  return blancos;
}

//Camara::bordes: Detector de bordes
/*
 *Detecta todos los bordes en la imagen filtrada.
 *Recibe: cv::Mat filtrada (CV_8UC1 u CV_8UC3), imagen de entrada.
 *Retorna: cv::Mat borde (CV_8UC1), imagen de salida con bordes.
*/
Mat Camara::borde(Mat blancos)
{
  Mat bordes;
  Canny(blancos, bordes, 50, 150, 3);
  return bordes;
}

//Camara::RDI: Región de interés
/*
 *Aplica una máscara pligonal a la imagen para reducir la región de interés.
 *Recibe: cv::Mat bordes (CV_8UC1), imagen de entrada.
 *Retorna: cv::Mat mascara, imagen de salida segmentada para la región de interés.
*/
Mat Camara::RDI(Mat bordes)
{
  Mat masc;
  Mat rdi=Mat::zeros(bordes.size(), bordes.type());
  Point p[4]={
    Point(0, 430),
    Point(120, 250),
    Point(520, 250),
    Point(640, 430)
  };
  fillConvexPoly(rdi, p, 4, Scalar(255, 0, 0));
  bitwise_and(bordes, rdi, masc);
  return masc;
}

//Camara::detectarL: Detectar líneas
/*
 *Emplea la transformada de Hough probabilística para obtener todos los segmentos de línea en la región de interés.
 *Recibe: cv::Mat masc (CV_8UC1), imagen de entrada.
 *Retorna: std::vector<cv::Vec4i> vector con los parámetros de todas las líneas detectadas
*/
std::vector<Vec4i> Camara::detectarL(Mat masc)
{
  std::vector<Vec4i> lin;
  HoughLinesP(masc, lin, 1, CV_PI/180, 20,20, 30);
  return lin;
}

//Camara::separarL: Separar las líneas a la izquierda y a la derecha del carril
/*
 *Separa las líneas a la derecha del carril y a la izquierda del carril, de acuerdo a su pendiente.
 *Recibe: std::vector<cv::Vec4i>, vector con todas las líneas detectadas con la función Camara::detectarL
 *Recibe: cv::Mat bordes (CV_8UC1), imagen binarizada con los bordes detectados en la imagen
 *Retorna: std::vector<std::vector<cv::Vec4i>> lin2, vector con dos columnas (líneas a la izquierda y a la derecha del carril).
*/
std::vector<std::vector<Vec4i>> Camara::separarL(std::vector<Vec4i> lin, Mat bordes)
{
  std::vector<std::vector<Vec4i>> lin2(2);
  size_t j=0;
  Point ini, fin;
  double pendU=0.3;
  std::vector<double> pendientes;
  std::vector<Vec4i> linsel;
  std::vector<Vec4i> derL, izqL;
  for (auto i: lin)
    {
      ini=Point(i[0], i[1]);
      fin=Point(i[2], i[3]);
      double pend=(static_cast<double>(fin.y)-static_cast<double>(ini.y))/(static_cast<double>(fin.x)-static_cast<double>(ini.x)+0.00001);
      if (abs(pend)>pendU)
	{
	  pendientes.push_back(pend);
	  linsel.push_back(i);
	}
    }
  imgC=static_cast<double>((bordes.cols/2));
  while (j<linsel.size())
    {
      ini=Point(linsel[j][0], linsel[j][1]);
      fin=Point(linsel[j][2], linsel[j][3]);
      if (pendientes[j]>0 && fin.x>imgC && ini.x>imgC)
	{
	  derL.push_back(linsel[j]);
	  derB=true;
	}
      else if (pendientes[j]<0 && fin.x<imgC && ini.x<imgC)
	{
	  izqL.push_back(linsel[j]);
	  izqB=true;
	}
      j++;
    }
  lin2[0]=derL;
  lin2[1]=izqL;
  return lin2;
}

//Camara::MinCu: Interpolación de líneas
/*
 *Se encarga de tomar todas las líneas a ambos lados del carril para interpolar una sola línea a cada lado del carril
 *Recibe: std::vector<std::vector<cv::Vec4i>>, vector con las líneas a la izquierda y derecha del carril
 *Recibe: cv::Mat imagen (CV_8UC1 o CV_8UC3), imagen con el contenido de filas y columnas del área a trabajar
 *Retorna: std::vector<cv::Point>> carril, vector con las dos líneas del carril.
*/
std::vector<Point> Camara::MinCu(std::vector<std::vector<Vec4i>> lin2, Mat imagen)
{
  std::vector<Point> carril(4);
  Point ini, fin, ini2, fin2;
  Vec4d linD, linI;
  std::vector<Point> pD, pI;
  if (derB==true)
    {
      for (auto i: lin2[0])
	{
	  ini=Point(i[0], i[1]);
	  fin=Point(i[2], i[3]);
	  pD.push_back(ini);
	  pD.push_back(fin);
	}
      if (pD.size()>0)
	{
	  fitLine(pD, linD, DIST_L2, 0, 0.01, 0.01);
	  mD=linD[1]/linD[0];
	  bD=Point(linD[2], linD[3]);
	}
    }
  if (izqB==true)
    {
      for (auto j: lin2[1])
	{
	  ini2=Point(j[0], j[1]);
	  fin2=Point(j[2], j[3]);
	  pI.push_back(ini2);
	  pI.push_back(fin2);
	}
      if (pI.size()>0)
	{
	  fitLine(pI, linI, DIST_L2, 0, 0.01, 0.01);
	  mI=linI[1]/linI[0];
	  bI=Point(linI[2], linI[3]);
	}
    }
  int iniY=imagen.rows-50;
  int finY=190;
  double iniDx=((iniY-bD.y)/mD)+bD.x;
  double finDx=((finY-bD.y)/mD)+bD.x;
  double iniIx=((iniY-bI.y)/mI)+bI.x;
  double finIx=((finY-bI.y)/mI)+bI.x;
  carril[0]=Point(iniDx, iniY);
  carril[1]=Point(finDx, finY);
  carril[2]=Point(iniIx, iniY);
  carril[3]=Point(finIx, finY);
  //double ma=(finY-iniY)/((finDx+finIx)/2)-((iniDx+iniIx)/2));
  //m=(finY-iniY)/((iniDx-iniIx)-(finDx-finIx));
  m=mI+mD;
  //m=(0.5*abs(mD)+0.5*abs(mI))/2;
  m=m*180/3.141592;
  //cout<<m<<" "<<ma<<endl;
  //cout<<m<<endl;
  return carril;
}

//Camara::dibujarC: Dibujar todos los parámetros del carril
/*
 *Se encarga de mostrar las líneas detectadas para el carril junto con el área encerrada entre ellas.
 *Recibe: cv::Mat imagen (CV_8UC3 o CV_8UC1), imagen de entrada original.
 *Recibe: std::vector<cv::Point> carril, vector con la información de las líneas a ambos lados del carril
 *Retorna: cv::Mat imagenL (CV_8UC3 o CV_8UC1), imagen de salida con la información del procesado.
*/
//Mat Camara::dibujar(Mat imagen, std::vector<Point> carril, std::vector<Point2f> esqE, string tipo, double d1, double d2)
Mat Camara::dibujarC(Mat imagen, std::vector<Point> carril)
{
  std::vector<Point> poli;
  Mat imagenL;
  imagenL=imagen.clone();
  poli.push_back(carril[2]);
  poli.push_back(carril[0]);
  poli.push_back(carril[1]);
  poli.push_back(carril[3]);
  fillConvexPoly(imagenL, poli, Scalar(0, 0, 255), CV_8U, 0);
  addWeighted(imagenL, 0.3, imagen, 1.0-0.3, 0.0, imagenL);
  line(imagenL, carril[0], carril[1], Scalar(0, 255, 0), 5, CV_8U);
  line(imagenL, carril[2], carril[3], Scalar(0, 255, 0), 5, CV_8U);
  return imagenL;
}

//Camara::detectarDescO: Encuentra los descriptores de los objetos
/*
 *Se encarga de tomar tomar las imágenes y por medio del algoritmo SURF encontrar los descriptores en las imagen de objeto.
 *Recibe: std::vector<KeyPoint> &kpO, referencia a un vector de descriptores de la señal de alto (instanciado dentro de la clase).
 *Recibe: cv::Mat &desO, referencia a la matriz de descriptores de la señal de alto (instanciado dentro de la clase.
 *Recibe: std::vector<KeyPoint> &kpD, referencia a un vector de descriptores de la señal de vuelta a la Derecha (instanciado dentro de la clase).
 *Recibe: cv::Mat &desD, referencia a la matriz de descriptores de la señal de vuelta a la Izquierda (instanciado dentro de la clase.
 *Recibe: std::vector<KeyPoint> &kpI, referencia a un vector de descriptores de la señal de vuelta a la Izquierda (instanciado dentro de la clase).
 *Recibe: cv::Mat &desO, referencia a la matriz de descriptores de la señal de vuelta a la Izquierda (instanciado dentro de la clase.
 *Función que no devuelve argumento.
*/
void Camara::detectarDescO(std::vector<KeyPoint> &kpA, Mat &desA, std::vector<KeyPoint> &kpD, Mat &desD, std::vector<KeyPoint> &kpI, Mat &desI)
{
  Mat alto=imread("../Figuras/Alto.png", IMREAD_GRAYSCALE);
  Mat dere=imread("../Figuras/Derecha.png", IMREAD_GRAYSCALE);
  Mat izq=imread("../Figuras/Izquierda.png", IMREAD_GRAYSCALE);
  Ptr<SURF> detector=SURF::create(20);
  detector->detectAndCompute(alto, noArray(), kpA, desA);
  detector->detectAndCompute(dere, noArray(), kpD, desD);
  detector->detectAndCompute(izq, noArray(), kpI, desI);
}

//Camara::detectarDescE: Encuentra los descriptores de la escena
/*
 *Se encarga de tomar tomar las imágenes y por medio del algoritmo SURF encontrar los descriptores en las imagen escena.
 *Recibe: cv::Mat imgE (CV_8UC1), imagen que contiene la escena.
 *Recibe: std::vector<KeyPoint> &kpE, referencia a un vector de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: cv::Mat &desE, referencia a la matriz de descriptores de la escena (instanciado dentro de la clase.
 *Función que no devuelve argumento.
*/
void Camara::detectarDescE(Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE)
{
  Ptr<SURF> detector=SURF::create(20); //minHess
  detector->detectAndCompute(imgE, noArray(), kpE, desE);
  }
  
//Camara::match: Empata los descriptores del objeto con los de la escena
/*
 *Toma los descriptores del objeto y los compara con los descriptores de la escena por medio del algoritmo FLANN.
 *Recibe: cv::Mat &desO, referencia a matriz de descriptores del objeto (instanciado dentro de la clase).
 *Recibe: cv::Mat &desE, referencia a matriz de descriptores de la escena (instanciado dentro de la clase).
 *Retorna: std::vector<DMatch> Bmatches, vector con todos los matches que cumplen con los requisitos dentro de la imagen.
*/
std::vector<DMatch> Camara::match(Mat &desO, Mat &desE)
{
  Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
  std::vector<std::vector<DMatch>> matches;
  matcher->knnMatch(desO, desE, matches, 2);
  std::vector<DMatch> Bmatches;
  for (int i=0; i<min(desO.rows-1, (int)matches.size()); i++)
    {
      if((matches[i][0].distance<0.75f*(matches[i][1].distance)) && ((int)matches[i].size()<=2 && (int)matches.size()>0))
      {
	Bmatches.push_back(matches[i][0]);
      }
    }
  //int n=Bmatches.size();
  //cout<<n<<endl;
  return Bmatches;
}

//Camara::dibujarMatch: muestra los matches entre el objeto y la escena de manera gráfica
/*
 *Guarda al objeto, la escena y los matches en una sola imagen.
 *Recibe: cv::Mat imgO, imagen del objeto.
 *Recibe: std::vector<KeyPoint> &kpO, referencia a vector de descriptores del objeto (instanciado dentro de la clase).
 *Recibe: cv::Mat imgE, imagen de la escena.
 *Recibe: std::vector<KeyPoint> &kpE, referencia a vector de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: std::vector<DMatch> Bmatches, vector con los matches buenos del objeto en la escena.
 *Retorna: cv:Mat imgM, imagen con el objeto, la escena y los matches.
*/
Mat Camara::dibujarMatch(Mat imgO, std::vector<KeyPoint> &kpO, Mat imgE, std::vector<KeyPoint> &kpE, std::vector<DMatch> Bmatches)
{
  Mat imgM;
  drawMatches(imgO, kpO, imgE, kpE, Bmatches, imgM, Scalar::all(-1), Scalar::all(-1), std::vector<char>(0), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  return imgM;
}

//Camara::locEscena: localiza el objeto dentro de la escena
/*
 *Relaciona las transoformaciones geométricas entre los descriptores del objeto con los descriptores de la escena.
 *Recibe: std::vector<Point2f> &obj, referencia a vector de puntos del objeto (instanciado dentro de la clase).
 *Recibe: std::vector<KeyPoint> &kpO, referencia a vector de descriptores del objeto (instanciado dentro de la clase).
 *Recibe: std::vector<Point2f> &esc, referencia a vector de puntos de la escena (instanciado dentro de la clase).
 *Recibe: std::vector<KeyPoint> &kpE, referencia a vector de puntos de la escena (instanciado dentro de la clase).
 *Recibe: std::vector<DMatch> Bmatches, vector con los matches buenos del objetod en la escena.
 *Retorna: cv::Mat H, matriz de homografía entre el objeto y la escena.
*/
Mat Camara::locEscena(std::vector<Point2f> &obj, std::vector<KeyPoint> &kpO, std::vector<Point2f> &esc,  std::vector<KeyPoint> &kpE, std::vector<DMatch> Bmatches)
{
  for (size_t i=0; i<Bmatches.size(); i++)
    {
      obj.push_back(kpO[Bmatches[i].queryIdx].pt);
      esc.push_back(kpE[Bmatches[i].trainIdx].pt);
    }
  Mat H=findHomography(obj, esc, RANSAC);
  //cout<<H<<endl;
  return H;
}

//Camara::limitesO: determina los límites del objeto y su correspondecia en la escena
/*
 *Obtiene los puntos de las esquinas del objeto y calcula la correspondencia de los puntos en la escena.
 *Recibe: cv::Mat imgO, imagen del objeto.
 *Recibe: cv::Mat H, matriz de homgrafía de la imagen.
 *Retorna: std::vector<Point2f> esqE, vector con las esquinas del objeto en la escena.
 */
std::vector<Point2f> Camara::limitesO(Size tam, Mat H)
{
  std::vector<Point2f> esqO(4);
  esqO[0]=Point2f(0, 0);
  esqO[1]=Point2f((float)tam.height, 0);
  esqO[2]=Point2f((float)tam.height, (float)tam.width);
  esqO[3]=Point2f(0, (float)tam.width);
  std::vector<Point2f> esqE(4);
  esqE[0]=Point2f(0.0, 0.0);
  esqE[1]=Point2f(0.0, 0.0);
  esqE[2]=Point2f(0.0, 0.0);
  esqE[3]=Point2f(0.0, 0.0);
  perspectiveTransform(esqO, esqE, H);
  return esqE;
}

//Camara::dibujarO, dibuja al objeto encerrado dentro de la escena.
/*
 *Combina la imagen obtenida con la función dibujarMatch para encerrar al objeto dentro de la escena.
 *Recibe: std::vector<Point2f> esqE, vector con las esquinas del objeto en la escena.
 *Recibe: cv::Mat imgO, imagen del objeto.
 *Recibe: cv::Mat imgM, imagen generada con la función dibujarMatch
 *Retorna: cv::Mat imgM, imagen con los matches y el objeto encerrado dentro de la escena.
 */
Mat Camara::dibujarO(std::vector<Point2f> esqE, Mat imgM, std::string acc)
{
  line(imgM, esqE[0], esqE[1], Scalar(255, 0, 0), 4);
  line(imgM, esqE[1], esqE[2], Scalar(255, 0, 0), 4);
  line(imgM, esqE[2], esqE[3], Scalar(255, 0, 0), 4);
  line(imgM, esqE[3], esqE[0], Scalar(255, 0, 0), 4);
  putText(imgM, acc, Point(50, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 0), 1, CV_8U);
  //circle(imgM, Point((esqE[0].x-esqE[2].x)/2, (esqE[0].y-esqE[2].y)/2), 5, Scalar(255, 0, 0), 2);
  return imgM;
}

//Camara::Alto, condensa los parámetros necesarios para buscar la señal de alto en la escena.
/*
 *Combina todas las funciones concernientes al algoritmo SURF, y les asigna los parámetros necesarios para buscar la señal de alto en la escena.
 *Recibe: std::vector<KeyPoint> &kpO, referencia a un vector de descriptores del objeto (instanciado dentro de la clase).
 *Recibe: cv::Mat &desO, referencia a la matriz de descriptores del objeto (instanciado dentro de la clase.
 *Recibe: std::vector<Point2f> &obj, referencia a vector de puntos del objeto (instanciado dentro de la clase).
 *Recibe: std::vector<Point2f> &esc, referencia a vector de puntos de la escena (instanciado dentro de la clase).
 *Recibe: cv::Mat imgE (CV_8UC1), imagen en blanco y negro de la escena.
 *Recibe: std::vector<KeyPoint> &kpE, referencia a un vector de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: cv::Mat &desE, referencia a la matriz de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: std::vector<Point2f> esqE, vector con las esquinas del objeto en la escena.
 *Retorna: bool, valor de lógica afirmativa que indica si se detectó o no el objeto dentro de la escena.
*/
bool Camara::Alto(std::vector<KeyPoint> &kpO, Mat &desO, std::vector<Point2f> &obj, std::vector<Point2f> &esc, Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE, std::vector<Point2f> &esqE)
{
  std::vector<DMatch> Bmatches=match(desO, desE);
  int n=Bmatches.size();
  if (n>7)
    {
      Mat H=locEscena(obj, kpO, esc, kpE, Bmatches);
      if (!H.empty())
      {
	esqE=limitesO(Size(145, 131), H);
      }
      //cout<<esqE<<endl;
      return true;
    }
  else
    {
      return false;
    }
  }

//Camara::Izquierda, condensa los parámetros necesarios para buscar la señal de alto en la escena.
/*
 *Combina todas las funciones concernientes al algoritmo SURF, y les asigna los parámetros necesarios para buscar la señal de alto en la escena.
 *Recibe: std::vector<KeyPoint> &kpO, referencia a un vector de descriptores del objeto (instanciado dentro de la clase).
 *Recibe: cv::Mat &desO, referencia a la matriz de descriptores del objeto (instanciado dentro de la clase.
 *Recibe: std::vector<Point2f> &obj, referencia a vector de puntos del objeto (instanciado dentro de la clase).
 *Recibe: std::vector<Point2f> &esc, referencia a vector de puntos de la escena (instanciado dentro de la clase).
 *Recibe: cv::Mat imgE (CV_8UC1), imagen en blanco y negro de la escena.
 *Recibe: std::vector<KeyPoint> &kpE, referencia a un vector de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: cv::Mat &desE, referencia a la matriz de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: std::vector<Point2f> esqE, vector con las esquinas del objeto en la escena.
 *Retorna: bool, valor de lógica afirmativa que indica si se detectó o no el objeto dentro de la escena.
*/
bool Camara::Izquierda(std::vector<KeyPoint> &kpO, Mat &desO, std::vector<Point2f> &obj, std::vector<Point2f> &esc, Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE, std::vector<Point2f> &esqE)
{
  std::vector<DMatch> Bmatches=match(desO, desE);
  int n=Bmatches.size();
  if (n>20)
    {
      Mat H=locEscena(obj, kpO, esc, kpE, Bmatches);
      if (!H.empty())
	{
	  esqE=limitesO(Size(134, 132), H);
	}
      //cout<<esqE<<endl;
      return true;
    }
  else
    {
      return false;
    }
}

//Camara::Derecha, condensa los parámetros necesarios para buscar la señal de alto en la escena.
/*
 *Combina todas las funciones concernientes al algoritmo SURF, y les asigna los parámetros necesarios para buscar la señal de alto en la escena.
 *Recibe: std::vector<KeyPoint> &kpO, referencia a un vector de descriptores del objeto (instanciado dentro de la clase).
 *Recibe: cv::Mat &desO, referencia a la matriz de descriptores del objeto (instanciado dentro de la clase.
 *Recibe: std::vector<Point2f> &obj, referencia a vector de puntos del objeto (instanciado dentro de la clase).
 *Recibe: std::vector<Point2f> &esc, referencia a vector de puntos de la escena (instanciado dentro de la clase).
 *Recibe: cv::Mat imgE (CV_8UC1), imagen en blanco y negro de la escena.
 *Recibe: std::vector<KeyPoint> &kpE, referencia a un vector de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: cv::Mat &desE, referencia a la matriz de descriptores de la escena (instanciado dentro de la clase).
 *Recibe: std::vector<Point2f> esqE, vector con las esquinas del objeto en la escena.
 *Retorna: bool, valor de lógica afirmativa que indica si se detectó o no el objeto dentro de la escena.
*/
bool Camara::Derecha(std::vector<KeyPoint> &kpO, Mat &desO, std::vector<Point2f> &obj, std::vector<Point2f> &esc, Mat imgE, std::vector<KeyPoint> &kpE, Mat &desE, std::vector<Point2f> &esqE)
{
  std::vector<DMatch> Bmatches=match(desO, desE);
  int n=Bmatches.size();
  if (n>20)
    {
      Mat H=locEscena(obj, kpO, esc, kpE, Bmatches);
      if (!H.empty())
	{
	  esqE=limitesO(Size(145, 131), H);
	}
      //cout<<esqE<<endl;
      return true;
    }
  else
    {
      return false;
    }
}

//Camara::getDistancia, obtener la distancia hasta un punto dado.
/*
 *Obtiene la distancia del punto p con la información de la imagen de profundiad.
 *Recibe: cv::Mat imD, imagen de profundidad (CV_16UC1).
 *Recibe: cv::Point p, punto de la imagen con coordenadas (x, y).
 *Retorna: double d, distancia desde el objeto al plano XY de la cámara.
*/
double Camara::getDist(Mat imD, Point p)
{
  ushort a=imD.at<ushort>(p.y, p.x);
  //double d=(double)a*13/655360;
  double d=(double)a*0.000124987;
  return d;
}

Mat Camara::umbProf(Mat prof, int umb)
{
  int i, j;
  Mat um=Mat::zeros(prof.rows, prof.cols, CV_8UC1); //CV_16UC1
  //cvtColor(prof, prof, COLOR_BGR2GRAY); //*
  for (i=0; i<prof.rows; i++)
    {
      for (j=0; j<prof.cols; j++)
	{
	  if (prof.at<ushort>(i, j)>umb) //short
	    {
	      um.at<ushort>(i, j)=0; //short
	    }
	  else
	    {
	      um.at<ushort>(i, j)=255;
	    }
	}
    }
  return um;
}

//Camara::dibujar: Dibuja todos los parámetros del procesamiento de imagen
/*
 *Se encarga de mostrar las líneas detectadas para el carril junto con el área encerrada entre ellas. Muestra los señalamientos encerrados y muestra las distancia al final del camino detectado y a los señalamientos.
 *Recibe: cv::Mat imagen (CV_8UC3 o CV_8UC1), imagen de entrada original.
 *Recibe: std::vector<cv::Point> carril, vector con la información de las líneas a ambos lados del carril
 *Recibe: std::vector<Point2f> esqE, vector con las esquinas del objeto detectado.
 *Recibe: int tipo, tipo de señalamiento detectado. 0 si ninguno, 1 si Derecha, 2 si Izquierda, 3 si Alto, 4 si Verde, 5 si Amarillo, 6 si Rojo.
 *Recibe: double d1, distancia al final del carril detectado.
 *Recibe: cv::Point p1, coordenadas del final del carril.
 *Recibe: double d2, distancia al objeto detectado.
 *Recibe: cv:Point p2, coordenadas del objeto detectado.
 *Recibe: double v, velocidad del vehículo
 *Retorna: cv::Mat imagenL (CV_8UC3), imagen de salida con la información del procesado.
*/
Mat Camara::dibujar(Mat imagen, std::vector<Point> carril, std::vector<Point2f> esqE, int tipo, double d1, Point p1, double d2, Point p2)
{
  std::vector<Point> poli;
  Mat imagenL;
  imagenL=imagen.clone();
  poli.push_back(carril[2]);
  poli.push_back(carril[0]);
  poli.push_back(carril[1]);
  poli.push_back(carril[3]);
  fillConvexPoly(imagenL, poli, Scalar(0, 0, 255), CV_8U, 0);
  addWeighted(imagenL, 0.3, imagen, 1.0-0.3, 0.0, imagenL);
  line(imagenL, carril[0], carril[1], Scalar(0, 255, 0), 5, CV_8U);
  line(imagenL, carril[2], carril[3], Scalar(0, 255, 0), 5, CV_8U);
  return imagenL;
}
