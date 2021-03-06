#ifndef _MARKER_H_
#define _MARKER_H_

#include "opencv2/opencv.hpp"
#include <stdlib.h>

using namespace std;
using namespace cv;



class Marker{
 public:
  Marker();
  Marker(Mat intrinsic,Mat distceof);

  //input image
  void setTargetImage(cv::Mat image);
  
  //project threshold
  void setThreshold(float e);
  float getThreshold();

  bool Match(cv::Mat image,double meanerr=1.0,double ratio=0.9);

  void searchMatches(vector<KeyPoint>& keys1, Mat& descp1,vector<KeyPoint>& keys2, Mat& descp2,vector<DMatch>& matches);

  unsigned int hamdist2(unsigned char* a, unsigned char* b, size_t size);

  //获取平面在3d世界坐标中的中心点
  cv::Mat get3dCenter();
  cv::Mat get3dPlane();

  float getDiagonalLenth();
  float getScale();

 private:
  float mThreshold;
  int mSize;
  
  cv::Mat mIntrinsic;
  cv::Mat mDistCeof;

  std::vector<cv::KeyPoint> mvKeys1;
  cv::Mat mImage1;
  cv::Mat mDescriptors1;
  
  std::vector<cv::KeyPoint> mvKeys2;
  cv::Mat mImage2;
  cv::Mat mDescriptors2;

  cv::Mat mCenterPt;
  cv::Mat mCenterPt2d;
  cv::Mat mPlane;

 public:
  cv::Mat mHomo;
  cv::Point2f mLbPt;//left bottom
  cv::Point2f mRtPt;//right top 
  std::vector<cv::Point2f> mInPt;
  std::vector<cv::Point2f> mvKeyPt;
  std::vector<cv::Point3f> mvMapPt;
  std::vector<cv::Point2f> mvKeyOutPt;

};


#endif
