#include "../include/odom.hpp"
/*
//
void seguirCar(Mat im, Mat im2, vector<Point2f>& ps1, vector<Point2f>& ps2, vector<uchar> estado)
{
  vector<float> err;
  Size tam=Size(21, 21);
  TermCriteria tc=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
  calcOpticalFlowPyrLK(im1, im2, ps1, ps2, estado, err, tam, 3, tc, 0, 0.001);
  int inC=0;
  for(int i=0; i<estado.size(); i++)
    {
      Point2f pt=ps2.at(i-inC);
      if((estado.at(i)==0)||(pt.x<0)||(pt.y<0))
	{
	  if ((pt.x<0)||(pt.y<0))
	    {
	      estado.at(i)=0;
	    }
	  ps1.erase(ps1.begin()+(i-inC));
	  ps2.erase(ps2.begin()+(i-inC));
	  inC++;
	}
    }
}

//
void detectarCar(Mat im, vector<Point2f>& ps1)
{
  vector<KeyPoint> kp1;
  int umb=20;
  bool max=true;
  FAST(im, kp1, umb, max);
  KeyPoint::convert(kp1, ps1, vector<int>());
}
*/