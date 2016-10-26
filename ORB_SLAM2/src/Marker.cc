#include "Marker.h"

#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

#include <vector> 

using namespace std;
using namespace cv;


extern void cvFitPlane(const CvMat* points, float* plane);

unsigned int maxD = 0;
unsigned int minD = 100;


Marker::Marker()
{
  mThreshold = 2.0;
  mSize = 0;

  //mvKeys1 = ;
  mImage1 = Mat();
  mDescriptors1 = Mat();

  //mvKeys2 = ;
  mImage2 = Mat();
  mDescriptors2 = Mat();
  
  mCenterPt = Mat();
  mPlane = Mat();

  mLbPt = Point2f(0,0);
  mRtPt = Point2f(0,0);

  //mInPt = ;
  //mvKeyPt = ;
  //mvMapPt = ;
}


Marker::Marker(Mat intrinsic,Mat distceof)
{
  mThreshold = 2.0;
  mSize = 0;

  //mvKeys1 = ;
  mImage1 = Mat();
  mDescriptors1 = Mat();

  //mvKeys2 = ;
  mImage2 = Mat();
  mDescriptors2 = Mat();
  
  mCenterPt = Mat();
  mPlane = Mat();

  mLbPt = Point2f(0,0);
  mRtPt = Point2f(0,0);

  intrinsic.copyTo(mIntrinsic);
  distceof.copyTo(mDistCeof);
}



  //input image
void Marker::setTargetImage(cv::Mat image)
{
  image.copyTo(mImage1);

  //ORB orb(300,(float)1.2,8,31,0,2,ORB::HARRIS_SCORE,31);
  //orb.operator()(mImage1, Mat(), mvKeys1, mDescriptors1, false);

  ORB orb;
  orb(mImage1, Mat(), mvKeys1, mDescriptors1);
}
  
  //project threshold
void Marker::setThreshold(float e)
{
  mThreshold = e;
}

float Marker::getThreshold()
{
  return mThreshold;
}

bool Marker::Match(cv::Mat image,double meanerr,double ratio)
{
  cout <<"match...\n";

  image.copyTo(mImage2);
  //cout<<"--image1 size:"<<mImage1.cols<<"x"<<mImage1.rows<<endl;
  //cout<<"--image2 size:"<<mImage2.cols<<"x"<<mImage2.rows<<endl;

  //orb extract
  ORB orb;
  orb(mImage2, Mat(), mvKeys2, mDescriptors2);
  //cout <<"-- orb1 features:"<<mvKeys1.size()<<endl;
  //cout <<"-- orb2 features:"<<mvKeys2.size()<<endl;

  // find matches
  BruteForceMatcher<HammingLUT> matcher;
  vector<DMatch> matches;

  if(0 == mDescriptors1.cols || 0 == mDescriptors2.cols){
    cout <<"image invalid.\n";
    return false;
  }

  //match
  matcher.match(mDescriptors1, mDescriptors2, matches);

  double max_dist = 0; double min_dist = 100;
  for( int i = 0; i < mDescriptors1.rows; i++ ){ 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
  }

  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );

  vector< DMatch > good_matches;
  for( int i = 0; i < matches.size(); i++ ) { 
    if( matches[i].distance < 0.5*max_dist )  { 
      good_matches.push_back( matches[i]); 
    }
  }

  printf("-- matchs num: %d\n", matches.size());
  printf("-- good matchs num: %d\n", good_matches.size());

  if(good_matches.size() < 10) {
    printf("not enough matches,exit.\n");
    return false;
  }

  vector<Point2f> pt1,pt2;
  for(int i = 0; i < (int)good_matches.size(); i++){
    pt1.push_back(mvKeys1[good_matches[i].queryIdx].pt);
    pt2.push_back(mvKeys2[good_matches[i].trainIdx].pt);
  }

  //homo = findHomography(pt1, pt2, Mat(), CV_RANSAC, 5);
  Mat homo = findHomography(pt1, pt2, CV_RANSAC);
  homo.copyTo(mHomo);
  /*
  printf("homo\n"
	 "%f %f %f\n"
	 "%f %f %f\n"
	 "%f %f %f\n",
	 homo.at<double>(0,0), homo.at<double>(0,1), homo.at<double>(0,2),
	 homo.at<double>(1,0), homo.at<double>(1,1), homo.at<double>(1,2),
	 homo.at<double>(2,0),homo.at<double>(2,1),homo.at<double>(2,2));
  */


  //投影.误差测量
  vector<Point2f> reproj;
  reproj.resize(pt1.size());
  perspectiveTransform(pt1, reproj, homo);

  Mat diff;
  diff = Mat(reproj) - Mat(pt2);

  int inlier = 0;
  double err_sum = 0;
  for(int i = 0; i < diff.rows; i++){
    float* ptr = diff.ptr<float>(i);
    float err = ptr[0]*ptr[0] + ptr[1]*ptr[1];
    if(err < 25.f){
      inlier++;
      err_sum += sqrt(err);
    }
  }

  double _meanerr = err_sum / inlier;
  double _ratio = inlier / (float)(diff.rows);
  printf("-- inlier num: %d\n", inlier);
  printf("-- ratio %f\n", _ratio);
  printf("-- mean reprojection error: %f\n",_meanerr);

  if(_meanerr > meanerr || _ratio < ratio){
    return false;
  }

  //将目标四个角点投影到另外一副图像
  cout<<"-- project 4 corners.\n";
  std::vector<Point2f> obj_corners(5);
  obj_corners[0] = cvPoint(0,0); 
  obj_corners[1] = cvPoint( mImage1.cols, 0 );
  obj_corners[2] = cvPoint( mImage1.cols, mImage1.rows );
  obj_corners[3] = cvPoint( 0, mImage1.rows );
  obj_corners[4] = cvPoint( mImage1.cols/2, mImage1.rows/2 );
  std::vector<Point2f> scene_corners(5);
  perspectiveTransform( obj_corners, scene_corners, homo);

  //检查边界
  /* 3-----2
   * |  5  |
   * 0-----1
  */
  int w = image.cols;
  int h = image.rows;

  //左边界
  scene_corners[0].x = scene_corners[0].x>0 ? scene_corners[0].x : 0;
  scene_corners[3].x = scene_corners[3].x>0 ? scene_corners[3].x : 0;
  //右边界
  scene_corners[1].x = scene_corners[1].x<w ? scene_corners[1].x : w;
  scene_corners[2].x = scene_corners[2].x<w ? scene_corners[2].x : w;
  //上边界
  scene_corners[2].y = scene_corners[2].y<h ? scene_corners[2].y : h;
  scene_corners[3].y = scene_corners[3].y<h ? scene_corners[3].y : h;
  //下边界
  scene_corners[0].y = scene_corners[0].y>0 ? scene_corners[0].y : 0;
  scene_corners[1].y = scene_corners[1].y>0 ? scene_corners[1].y : 0;
  
  //cout<<"get leftBottom and rightTop points.\n";
  float left,right,top,bottom=0;
  for(int i=0;i<4;i++) {
    for(int j=1;i<4;i++) {
      left   = (scene_corners[j].x < scene_corners[i].x) ? scene_corners[j].x : scene_corners[i].x;
      right  = (scene_corners[j].x > scene_corners[i].x) ? scene_corners[j].x : scene_corners[i].x;
      top    = (scene_corners[j].y > scene_corners[i].y) ? scene_corners[j].y : scene_corners[i].y;
      bottom = (scene_corners[j].y < scene_corners[i].y) ? scene_corners[j].y : scene_corners[i].y;
    }
  }

  mInPt.swap(scene_corners);
  //cout<<"size:"<<mInPt.size()<<endl;

  mLbPt = Point2f(left,bottom);
  mRtPt = Point2f(right,top);
  cout<<"-- lb:"<<left<<","<<bottom<<endl;
  cout<<"-- rt:"<<right<<","<<top<<endl;

  return true;
}


cv::Mat Marker::get3dCenter()
{
  CvMat*points_mat = cvCreateMat(mvMapPt.size(), 3, CV_32FC1);
  float sumx = 0;
  float sumy = 0;
  float sumz = 0;
  int m = mvMapPt.size();
  for (int i=0;i < m; ++i) {  
    float x = mvMapPt[i].x;
    float y = mvMapPt[i].y;
    float z = mvMapPt[i].z;

    cout<<mvMapPt[i]<<endl;

    sumx += x;
    sumy += y;
    sumz += z;
  }  
  //目标中心
  float center[3];
  center[0] = sumx/(float)m;
  center[1] = sumy/(float)m;
  center[2] = sumz/(float)m;
  cv::Mat centerMat(1,3,CV_32FC1,center);
  centerMat.copyTo(mCenterPt);

  return mCenterPt;
}

cv::Mat Marker::get3dPlane()
{
  CvMat*points_mat = cvCreateMat(mvMapPt.size(), 3, CV_32FC1);
  for (int i=0;i < mvMapPt.size(); ++i) {  
    points_mat->data.fl[i*3+0] = mvMapPt[i].x;
    points_mat->data.fl[i*3 + 1] =  mvMapPt[i].y;
    points_mat->data.fl[i*3 + 2] =  mvMapPt[i].z;

    //points_mat->data.fl[i*3+0] = mvMapPt[i].x * 1000.0f;
    //points_mat->data.fl[i*3 + 1] =  mvMapPt[i].y * 1000.0f;
    //points_mat->data.fl[i*3 + 2] =  mvMapPt[i].z * 1000.0f;
  }  

  //最小二乘法拟合平面
  float plane[4] = { 0 };
  cvFitPlane(points_mat, plane);

  //normalize
  float r = 1/(fabs(plane[3])+0.000001);
  plane[0] =  plane[0]*r;
  plane[1] =  plane[1]*r;
  plane[2] =  plane[2]*r;
  plane[3] = 1.0;

  cv::Mat planeMat(1,4,CV_32FC1,plane);
  planeMat.copyTo(mPlane);
  //cout<<"plane:"<<planeMat<<endl;

  return mPlane;
}


void Marker::searchMatches(vector<KeyPoint>& keys1, Mat& descp1,vector<KeyPoint>& keys2, Mat& descp2,vector<DMatch>& matches)
{
  for( int i = 0; i < (int)keys2.size(); i++) {
    unsigned int min_dist = INT_MAX;
    unsigned int sec_dist = INT_MAX;
    int min_idx = -1, sec_idx = -1;
    unsigned char* query_feat = descp2.ptr(i);
    for( int j = 0; j < (int)keys1.size(); j++) {
      unsigned char* train_feat = descp1.ptr(j);
      unsigned int dist = hamdist2(query_feat, train_feat, 32);

      if(dist < min_dist){
	sec_dist = min_dist;
	sec_idx = min_idx;
	min_dist = dist;
	min_idx = j;
      }else if(dist < sec_dist){
	sec_dist = dist;
	sec_idx = j;
      }

      if( dist <= minD ) minD  = dist;
      if( dist > maxD ) maxD  = dist;
    }


    if(min_dist <= (unsigned int)(sec_dist * 0.8) && min_dist <=50){
      //if(min_dist <= 50)
      matches.push_back(DMatch(i, min_idx, 0, (float)min_dist));
    }
  }

  printf("-- Max dist : %d \n", maxD );
  printf("-- Min dist : %d \n", minD );

}

unsigned int Marker::hamdist2(unsigned char* a, unsigned char* b, size_t size)
{
  HammingLUT lut;

  unsigned int result;
  result = lut((a), (b), size);
  return result;
}

//获取较长的对角线
float Marker::getDiagonalLenth()
{
  float d02 = (mInPt[0].x-mInPt[2].x)*(mInPt[0].x-mInPt[2].x) + (mInPt[0].y-mInPt[2].y)*(mInPt[0].y-mInPt[2].y);
  float d13 = (mInPt[1].x-mInPt[3].x)*(mInPt[1].x-mInPt[3].x) + (mInPt[1].y-mInPt[3].y)*(mInPt[1].y-mInPt[3].y);

  float d = d02 > d13 ? d02 : d13;

  return (float)pow(d,0.5);
}

//获取3d坐标与2d坐标的尺度
float Marker::getScale()
{
  float sumx = 0;
  float sumy = 0;
  float sumz = 0;
  float suma = 0;
  float sumb = 0;

  int m = mvMapPt.size();

  for (int i=0;i < m; ++i) {  
    float a = mvKeyPt[i].x;
    float b = mvKeyPt[i].y;

    float x = mvMapPt[i].x;
    float y = mvMapPt[i].y;
    float z = mvMapPt[i].z;


    suma += a;
    sumb += b;

    sumx += x;
    sumy += y;
    sumz += z;
  }  

  //2d中心
  float center2d[2];
  center2d[0] = suma/(float)m;
  center2d[1] = sumb/(float)m;

  //3d中心
  float center3d[3];
  center3d[0] = sumx/(float)m;
  center3d[1] = sumy/(float)m;
  center3d[2] = sumz/(float)m;

  float sum2 = 0;
  float sum3 = 0;
  for (int i=0;i < m; ++i) {  
    float a = mvKeyPt[i].x - center2d[0];
    float b = mvKeyPt[i].y - center2d[1];
    float d2 = a*a + b*b;
    d2 = (float)pow(d2,0.5);

    float x = mvMapPt[i].x - center3d[0];
    float y = mvMapPt[i].y - center3d[1];
    float z = mvMapPt[i].z - center3d[2];
    float d3 = x*x + y*y +z*z;
    d3 = (float)pow(d3,0.5);

    sum2 += d2;
    sum3 += d3;
  }  

  return sum3/sum2;
}
