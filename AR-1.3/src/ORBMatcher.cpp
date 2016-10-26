#include "ORBMatcher.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

#include <vector> 

using namespace std;
using namespace cv;


unsigned int maxD = 0;
unsigned int minD = 100;

unsigned int hamdist(unsigned int x, unsigned int y)
{
  unsigned int dist = 0, val = x ^ y;

  // Count the number of set bits

  while(val)
    {
      ++dist;
      val &= val - 1;
    }

  return dist;
}

unsigned int hamdist2(unsigned char* a, unsigned char* b, size_t size)
{
  HammingLUT lut;

  unsigned int result;
  result = lut((a), (b), size);
  return result;
}

void naive_nn_search(vector<KeyPoint>& keys1, Mat& descp1,
		     vector<KeyPoint>& keys2, Mat& descp2,
		     vector<DMatch>& matches)
{
  for( int i = 0; i < (int)keys2.size(); i++){
    unsigned int min_dist = INT_MAX;
    int min_idx = -1;
    unsigned char* query_feat = descp2.ptr(i);
    for( int j = 0; j < (int)keys1.size(); j++){
      unsigned char* train_feat = descp1.ptr(j);
      unsigned int dist = hamdist2(query_feat, train_feat, 32);

      if(dist < min_dist){
	min_dist = dist;
	min_idx = j;
      }
    }

    //if(min_dist <= (unsigned int)(second_dist * 0.8)){

    if(min_dist <= 50){
      matches.push_back(DMatch(i, min_idx, 0, (float)min_dist));
    }
  }
}

void naive_nn_search2(vector<KeyPoint>& keys1, Mat& descp1,
		      vector<KeyPoint>& keys2, Mat& descp2,
		      vector<DMatch>& matches)
{
  for( int i = 0; i < (int)keys2.size(); i++){
    unsigned int min_dist = INT_MAX;
    unsigned int sec_dist = INT_MAX;
    int min_idx = -1, sec_idx = -1;
    unsigned char* query_feat = descp2.ptr(i);
    for( int j = 0; j < (int)keys1.size(); j++){
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
}

ORBMatcher::ORBMatcher()
{
  //TODOso

}

int ORBMatcher::Match(cv::Mat img1,cv::Mat img2)
{
  if (img1.empty() || img2.empty()){
    printf("error reading images.\n ");
    return -1;
  }

  //GaussianBlur(img1, img1, Size(5, 5), 0);
  //GaussianBlur(img2, img2, Size(5, 5), 0);

  //ORB orb1(3000, ORB::CommonParams(1.2, 8));
  //ORB orb1(int nfeatures = 3000, float scaleFactor = 1.2f, int nlevels = 8, int edgeThreshold = 31,
  //int firstLevel = 0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31 );
  //ORB orb2(int nfeatures = 100, float scaleFactor = 1.2f, int nlevels = 8, int edgeThreshold = 31,
  //ORB orb2(100, ORB::CommonParams(1.2, 1));
  ORB orb1(300,(float)1.2,8,31,0,2,ORB::HARRIS_SCORE,31);
  ORB orb2(300,(float)1.2,8,31,0,2,ORB::HARRIS_SCORE,31);

  vector<KeyPoint> keys1, keys2;
  Mat descriptors1, descriptors2;

  orb1.operator()(img1, Mat(), keys1, descriptors1, false);
  printf("tem feat num: %d\n", keys1.size());

  int64 st, et;
  st = cvGetTickCount();
  orb2(img2, Mat(), keys2, descriptors2, false);
  et = cvGetTickCount();
  printf("orb2 extraction time: %f\n", (et-st)/(double)cvGetTickFrequency()/1000.);
  printf("query feat num: %d\n", keys2.size());

  // find matches
  vector<DMatch> matches;
  st = cvGetTickCount();
  naive_nn_search2(keys1, descriptors1, keys2, descriptors2, matches);
  printf("-- Max dist : %d \n", maxD );
  printf("-- Min dist : %d \n", minD );

  std::vector< DMatch > good_matches;
  for( int i = 0; i < matches.size(); i++ ) { 
    //if( matches[i].distance <= 2*minD )  { 
    if( matches[i].distance <= 0.5*maxD )  { 
      good_matches.push_back( matches[i]); 
    }
  }

  et = cvGetTickCount();

  keys1.size();
  keys1[0].pt.x;
  keys1[0].pt.y;
  keys1[0].angle;
  keys1[0].size;


  printf("match time: %f\n", (et-st)/(double)cvGetTickFrequency()/1000.);
  printf("matchs num: %d\n", matches.size());
  printf("good matchs num: %d\n", good_matches.size());

  if(good_matches.size() < 15){
    printf("not enough matches,exit.\n");
    return -1;
  }


  Mat showImg;
  //drawMatches(img2, keys2, img1, keys1, matches, showImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255));
  //drawMatches(img1, keys1, img2, keys2, good_matches, showImg,Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  

  vector<Point2f> pt1,pt2;
  for(int i = 0; i < (int)good_matches.size(); i++){
    pt2.push_back(Point2f(keys2[good_matches[i].queryIdx].pt.x, keys2[good_matches[i].queryIdx].pt.y));
    pt1.push_back(Point2f(keys1[good_matches[i].trainIdx].pt.x, keys1[good_matches[i].trainIdx].pt.y));
  }

  st = cvGetTickCount();
  //homo = findHomography(pt1, pt2, Mat(), CV_RANSAC, 5);
  Mat homo = findHomography(pt1, pt2, CV_RANSAC);
  m_homo = homo;
  et = cvGetTickCount();
  printf("ransac time: %f\n", (et-st)/(double)cvGetTickFrequency()/1000.);

  printf("homo\n"
	 "%f %f %f\n"
	 "%f %f %f\n"
	 "%f %f %f\n",
	 homo.at<double>(0,0), homo.at<double>(0,1), homo.at<double>(0,2),
	 homo.at<double>(1,0), homo.at<double>(1,1), homo.at<double>(1,2),
	 homo.at<double>(2,0),homo.at<double>(2,1),homo.at<double>(2,2));


  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); 
  obj_corners[1] = cvPoint( img1.cols, 0 );
  obj_corners[2] = cvPoint( img1.cols, img1.rows );
  obj_corners[3] = cvPoint( 0, img1.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, homo);

#if 0
  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( showImg, scene_corners[0] + Point2f( img1.cols, 0), scene_corners[1] + Point2f( img1.cols, 0), Scalar(255,  0, 0), 4 );
  line( showImg, scene_corners[1] + Point2f( img1.cols, 0), scene_corners[2] + Point2f( img1.cols, 0), Scalar( 0, 255, 0), 4 );
  line( showImg, scene_corners[2] + Point2f( img1.cols, 0), scene_corners[3] + Point2f( img1.cols, 0), Scalar( 0, 0, 255), 4 );
  line( showImg, scene_corners[3] + Point2f( img1.cols, 0), scene_corners[0] + Point2f( img1.cols, 0), Scalar( 255, 255, 0), 4 );
  

  string winName = "Matches";
  namedWindow( winName, 1 );
  imshow( winName, showImg );
  waitKey(0);
#endif


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
  printf("inlier num: %d\n", inlier);
  printf("ratio %f\n", inlier / (float)(diff.rows));
  printf("mean reprojection error: %f\n", err_sum / inlier);

  return 0;
}
