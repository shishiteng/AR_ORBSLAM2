/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<stdlib.h>
#include<sys/time.h>

#include<mutex>

#include "Marker.h"

//#define cout cout<<setw(-50)<<"["<<__FUNCTION__<<":"<<__LINE__<<"]"
//#define cerr cerr<<setw(-50)<<"["<<__FUNCTION__<<":"<<__LINE__<<"]"

using namespace std;
using namespace cv;

#define PI 3.141593

/*************************旋转矩阵、四元数、欧拉角*****************************/
//旋转矩阵得到四元数
cv::Mat Matrix2Quaternion(cv::Mat matrix) {
  float qx, qy, qz, qw;

  // 计算矩阵轨迹
  float a[4][4] = { 0 };
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      a[i][j] = matrix.at<float>(i, j);

  // I removed + 1.0f; see discussion with Ethan
  float trace = a[0][0] + a[1][1] + a[2][2];
  if (trace > 0) {
    // I changed M_EPSILON to 0
    float s = 0.5f / sqrtf(trace + 1.0f);
    qw = 0.25f / s;
    qx = (a[2][1] - a[1][2]) * s;
    qy = (a[0][2] - a[2][0]) * s;
    qz = (a[1][0] - a[0][1]) * s;
  } else {
    if (a[0][0] > a[1][1] && a[0][0] > a[2][2]) {
      float s = 2.0f * sqrtf(1.0f + a[0][0] - a[1][1] - a[2][2]);
      qw = (a[2][1] - a[1][2]) / s;
      qx = 0.25f * s;
      qy = (a[0][1] + a[1][0]) / s;
      qz = (a[0][2] + a[2][0]) / s;
    } else if (a[1][1] > a[2][2]) {
      float s = 2.0f * sqrtf(1.0f + a[1][1] - a[0][0] - a[2][2]);
      qw = (a[0][2] - a[2][0]) / s;
      qx = (a[0][1] + a[1][0]) / s;
      qy = 0.25f * s;
      qz = (a[1][2] + a[2][1]) / s;
    } else {
      float s = 2.0f * sqrtf(1.0f + a[2][2] - a[0][0] - a[1][1]);
      qw = (a[1][0] - a[0][1]) / s;
      qx = (a[0][2] + a[2][0]) / s;
      qy = (a[1][2] + a[2][1]) / s;
      qz = 0.25f * s;
    }
  }

  float q[] = { qw, qx, qy, qz };
  //cout<< "\n quaternion:"<<cv::Mat(4,1,CV_32FC1,q).t()<<endl;
  return cv::Mat(4, 1, CV_32FC1, q).clone();
}

//四元数得到欧拉角
cv::Mat Quaternion2Euler(float *q) {
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  float ret[3] = { 0 };
  //cv::Mat ret(3,1,CV_32FC1);

  float test = x * y + z * w;
  if (test > 0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = PI / 2;
    ret[0] = 0.0f;
    return cv::Mat(3, 1, CV_32FC1, ret).clone();
  }
  if (test < -0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = -PI / 2;
    ret[0] = 0.0f;
    return cv::Mat(3, 1, CV_32FC1, ret).clone();
  }
  float sqx = x * x;
  float sqy = y * y;
  float sqz = z * z;
  ret[2] = atan2f(2.0f * y * w - 2.0f * x * z,
		  1.0f - 2.0f * sqy - 2.0f * sqz);
  ret[1] = asin(2.0f * test);
  ret[0] = atan2f(2.0f * x * w - 2.0f * z * y,
		  1.0f - 2.0f * sqx - 2.0f * sqz);

  ret[0] *= 180 / PI;
  ret[1] *= 180 / PI;
  ret[2] *= 180 / PI;

  return cv::Mat(3, 1, CV_32FC1, ret).clone();
}

//四元数得到欧拉角
cv::Mat Quaternion2Euler(cv::Mat q) {
  float w = q.at<float>(0);
  float x = q.at<float>(1);
  float y = q.at<float>(2);
  float z = q.at<float>(3);

  float ret[3] = { 0 };
  //cv::Mat ret(3,1,CV_32FC1);

  float test = x * y + z * w;
  if (test > 0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = PI / 2;
    ret[0] = 0.0f;
    return cv::Mat(3, 1, CV_32FC1, ret).clone();
  }
  if (test < -0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = -PI / 2;
    ret[0] = 0.0f;
    return cv::Mat(3, 1, CV_32FC1, ret).clone();
  }
  float sqx = x * x;
  float sqy = y * y;
  float sqz = z * z;

  ret[0] = atan2f(2.0f * (y * w - x * z), 1.0f - 2.0f * (sqy + sqz));
  ret[1] = asin(2.0f * test);
  ret[2] = atan2f(2.0f * (x * w - y * z), 1.0f - 2.0f * (sqx + sqz));

  return cv::Mat(3, 1, CV_32FC1, ret).clone();
}

cv::Mat Matrix2Euler(cv::Mat matrix) {
  cv::Mat q = Matrix2Quaternion(matrix);
  cv::Mat angle = Quaternion2Euler(q);
  return angle.clone();

  float m[4][4] = { 0 };
  for (int a = 0; a < 4; a++)
    for (int b = 0; b < 4; b++)
      m[a][b] = matrix.at<float>(a, b);

  float a[3];
  a[0] = atan2f(m[2][1], m[2][2]) * 180 / PI;
  a[1] = atan2f(-m[2][0], sqrtf(m[2][1] * m[2][1] + m[2][2] * m[2][2]))
    * 180/PI;
  a[2] = atan2f(m[1][0], m[0][0]) * 180 / PI;
  return cv::Mat(3, 1, CV_32FC1, a).clone();
}

// 由欧拉角创建四元数
cv::Mat Euler2Quaternion(float *angle) {
  float heading = angle[0];
  float attitude = angle[1];
  float bank = angle[2];

  float c1 = cos(heading / 2);
  float s1 = sin(heading / 2);
  float c2 = cos(attitude / 2);
  float s2 = sin(attitude / 2);
  float c3 = cos(bank / 2);
  float s3 = sin(bank / 2);
  float c1c2 = c1 * c2;
  float s1s2 = s1 * s2;
  float w = c1c2 * c3 - s1s2 * s3;
  float x = c1c2 * s3 + s1s2 * c3;
  float y = s1 * c2 * c3 + c1 * s2 * s3;
  float z = c1 * s2 * c3 - s1 * c2 * s3;
  float q[4] = { w, x, y, z };
  cv::Mat ret(4, 1, CV_32FC1, q);
  return ret.clone();

}

cv::Mat Euler2Quaternion(cv::Mat Angle) {
  //angle:roll pitch yaw
  //    q:w x y z
  float angle[3];
  angle[0] = Angle.at<float>(0);
  angle[1] = Angle.at<float>(1);
  angle[2] = Angle.at<float>(2);

  return Euler2Quaternion(angle).clone();
}

// 由旋转四元数推导出矩阵
cv::Mat Quaternion2Matrix(cv::Mat q) {
  float w = q.at<float>(0);
  float x = q.at<float>(1);
  float y = q.at<float>(2);
  float z = q.at<float>(3);

  float xx = x * x;
  float yy = y * y;
  float zz = z * z;
  float xy = x * y;
  float wz = w * z;
  float wy = w * y;
  float xz = x * z;
  float yz = y * z;
  float wx = w * x;

  float ret[4][4];
  ret[0][0] = 1.0f - 2 * (yy + zz);
  ret[0][1] = 2 * (xy - wz);
  ret[0][2] = 2 * (wy + xz);
  ret[0][3] = 0.0f;

  ret[1][0] = 2 * (xy + wz);
  ret[1][1] = 1.0f - 2 * (xx + zz);
  ret[1][2] = 2 * (yz - wx);
  ret[1][3] = 0.0f;

  ret[2][0] = 2 * (xz - wy);
  ret[2][1] = 2 * (yz + wx);
  ret[2][2] = 1.0f - 2 * (xx + yy);
  ret[2][3] = 0.0f;

  ret[3][0] = 0.0f;
  ret[3][1] = 0.0f;
  ret[3][2] = 0.0f;
  ret[3][3] = 1.0f;

  return cv::Mat(4, 4, CV_32FC1, ret).clone();
}

//欧拉角转旋转矩阵
cv::Mat Euler2Matrix(float *angle) {
  cv::Mat q = Euler2Quaternion(angle);
  return Quaternion2Matrix(q).clone();
}

//欧拉角转旋转矩阵
cv::Mat Euler2Matrix(cv::Mat angle) {
  cv::Mat q = Euler2Quaternion(angle);
  return Quaternion2Matrix(q).clone();
}

/*********************************************************************/

//return us
unsigned int micros() {
  struct timeval dwTime;
  gettimeofday(&dwTime, NULL);
  return (1000000 * dwTime.tv_sec + dwTime.tv_usec);
}

void ComparePose(cv::Mat *pInitCampose, cv::Mat *pCurCampose,
		 imu_data_t *pInitImuData, imu_data_t *pCurImuData) {

}

float getTriangleArea(Point2f pt1, Point2f pt2, Point2f pt3) {
  float area, s, a, b, c;

  a = sqrtf(
	    (pt1.x - pt2.x) * (pt1.x - pt2.x)
	    + (pt1.y - pt2.y) * (pt1.y - pt2.y));
  b = sqrtf(
	    (pt3.x - pt2.x) * (pt3.x - pt2.x)
	    + (pt3.y - pt2.y) * (pt3.y - pt2.y));
  c = sqrtf(
	    (pt1.x - pt3.x) * (pt1.x - pt3.x)
	    + (pt1.y - pt3.y) * (pt1.y - pt3.y));
  s = (a + b + c) / 2;
  area = sqrtf(s * (s - a) * (s - b) * (s - c));

  return area;
}

//在四边形内部
bool isInQuadrilateral(Point2f pt, Point2f pt1, Point2f pt2, Point2f pt3,
		       Point2f pt4) {
  float s123 = getTriangleArea(pt1, pt2, pt3);
  float s134 = getTriangleArea(pt1, pt3, pt4);

  float s12 = getTriangleArea(pt, pt1, pt2);
  float s23 = getTriangleArea(pt, pt2, pt3);
  float s34 = getTriangleArea(pt, pt3, pt4);
  float s41 = getTriangleArea(pt, pt4, pt1);

  float s = s123 + s134;
  float _s = s12 + s23 + s34 + s41;

  /*
    cout<<"pt:"<<pt<<" |"<<pt1<<" "<<pt2<<" "<<pt3<<" "<<pt4<<endl;
    cout<<"real area:"<<s<<endl;
    cout<<"  my area:"<<_s<<endl;
    cout <<"       e:"<<e<<endl;
    cout <<"       r:"<<_s/s<<endl;
  */

  float ratio = _s / s;
  if (ratio > 0.99 && ratio < 1.01)
    return true;

  return false;
}

void cvFitPlane(const CvMat* points, float* plane) {
  // Estimate geometric centroid.  
  int nrows = points->rows;
  int ncols = points->cols;
  int type = points->type;
  CvMat* centroid = cvCreateMat(1, ncols, type);
  cvSet(centroid, cvScalar(0));
  for (int c = 0; c < ncols; c++) {
    for (int r = 0; r < nrows; r++) {
      centroid->data.fl[c] += points->data.fl[ncols * r + c];
    }
    centroid->data.fl[c] /= nrows;
  }
  // Subtract geometric centroid from each point.  
  CvMat* points2 = cvCreateMat(nrows, ncols, type);
  for (int r = 0; r < nrows; r++)
    for (int c = 0; c < ncols; c++)
      points2->data.fl[ncols * r + c] = points->data.fl[ncols * r + c]
	- centroid->data.fl[c];
  // Evaluate SVD of covariance matrix.  
  CvMat* A = cvCreateMat(ncols, ncols, type);
  CvMat* V = cvCreateMat(ncols, ncols, type);
  CvMat* W = cvCreateMat(ncols, ncols, type);
  /*
    CvMat* A = cvCreateMat(nrows, nrows, type);  
    CvMat* W = cvCreateMat(nrows, ncols, type);  
    CvMat* V = cvCreateMat(ncols, ncols, type);  
  */
  cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
  cvSVD(A, W, NULL, V, CV_SVD_V_T);
  // Assign plane coefficients by singular vector corresponding to smallest singular value.  
  plane[ncols] = 0;
  for (int c = 0; c < ncols; c++) {
    plane[c] = V->data.fl[ncols * (ncols - 1) + c];
    plane[ncols] += plane[c] * centroid->data.fl[c];
  }
  // Release allocated resources.  
  cvReleaseMat(&centroid);
  cvReleaseMat(&points2);
  cvReleaseMat(&A);
  cvReleaseMat(&W);
  cvReleaseMat(&V);
}

namespace ORB_SLAM2 {

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
  mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
  mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys),
  mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
  // Load camera parameters from settings file

  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  float fps = fSettings["Camera.fps"];
  if (fps == 0)
    fps = 30;

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  cout << endl << "Camera Parameters: " << endl;
  cout << "- fx: " << fx << endl;
  cout << "- fy: " << fy << endl;
  cout << "- cx: " << cx << endl;
  cout << "- cy: " << cy << endl;
  cout << "- k1: " << DistCoef.at<float>(0) << endl;
  cout << "- k2: " << DistCoef.at<float>(1) << endl;
  if (DistCoef.rows == 5)
    cout << "- k3: " << DistCoef.at<float>(4) << endl;
  cout << "- p1: " << DistCoef.at<float>(2) << endl;
  cout << "- p2: " << DistCoef.at<float>(3) << endl;
  cout << "- fps: " << fps << endl;

  int nRGB = fSettings["Camera.RGB"];
  mbRGB = nRGB;

  if (mbRGB)
    cout << "- color order: RGB (ignored if grayscale)" << endl;
  else
    cout << "- color order: BGR (ignored if grayscale)" << endl;

  // Load ORB parameters

  int nFeatures = fSettings["ORBextractor.nFeatures"];
  float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
  int nLevels = fSettings["ORBextractor.nLevels"];
  int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
  int fMinThFAST = fSettings["ORBextractor.minThFAST"];

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels,
					fIniThFAST, fMinThFAST);

  if (sensor == System::STEREO)
    mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels,
					   fIniThFAST, fMinThFAST);

  if (sensor == System::MONOCULAR)
    mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor,
					 nLevels, fIniThFAST, fMinThFAST);

  cout << endl << "ORB Extractor Parameters: " << endl;
  cout << "- Number of Features: " << nFeatures << endl;
  cout << "- Scale Levels: " << nLevels << endl;
  cout << "- Scale Factor: " << fScaleFactor << endl;
  cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
  cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

  if (sensor == System::STEREO || sensor == System::RGBD) {
    mThDepth = mbf * (float) fSettings["ThDepth"] / fx;
    cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth
	 << endl;
  }

  if (sensor == System::RGBD) {
    mDepthMapFactor = fSettings["DepthMapFactor"];
    if (mDepthMapFactor == 0)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer) {
  mpViewer = pViewer;
}

void Tracking::SetImu(Imu *pImu) {
  mpImu = pImu;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft,
				  const cv::Mat &imRectRight, const double &timestamp) {
  mImGray = imRectLeft;
  cv::Mat imGrayRight = imRectRight;

  if (mImGray.channels() == 3) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, CV_RGB2GRAY);
      cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
    } else {
      cvtColor(mImGray, mImGray, CV_BGR2GRAY);
      cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
    }
  } else if (mImGray.channels() == 4) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
      cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
    } else {
      cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
      cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
    }
  }

  mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
			mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
  Track();

  return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD,
				const double &timestamp) {
  mImGray = imRGB;
  cv::Mat imDepth = imD;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
  }

  if (mDepthMapFactor != 1 || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft,
			mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

  Track();

  return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) 
{
  mImGray = im;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
  }

  if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
    mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
  else
    mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

  //Imu data
  if (NULL != mpImu) {
    // Get IMU Mutex -> IMU data cannot be changed
    unique_lock < mutex > lock(mpImu->mMutexImu);
    memcpy(&(mCurrentFrame.mImuData), &(mpImu->mData), sizeof(imu_data_t));
    fprintf(stdout, "    P/R/Y/P:%3d %3d %3d\n",
	    mCurrentFrame.mImuData.pitch, mCurrentFrame.mImuData.roll,
	    mCurrentFrame.mImuData.yaw);
  }

  cout << "Add IMU data.\n";
  AddImuData();

  cout << "Track.\n";
  Track();

  //对比campose计算出的旋转和imu计算出的旋转
  cv::Mat *pInitCampose = &mInitialFrame.mTcw;
  imu_data_t *pInitImuData = &mInitialFrame.mImuData;
  cv::Mat *pCurCampose = &mCurrentFrame.mTcw;
  imu_data_t *pCurImuData = &mCurrentFrame.mImuData;
  ComparePose(pInitCampose, pCurCampose, pInitImuData, pCurImuData);

  return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocularEx(const cv::Mat &im, const double &timestamp, Marker &marker) 	
{
  mImGray = im;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
  }

  if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
    mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor,
			  mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
  else
    mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft,
			  mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

  Track();

  //sst
  if (mState == OK) {
    int m = 0;
    int n = 0;
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;
    double err_sum = 0;
    std::vector<cv::Mat> goodPt;
    for (std::vector<cv::KeyPoint>::iterator it = mCurrentFrame.mvKeys.begin(); it != mCurrentFrame.mvKeys.end();it++) {
      cv::KeyPoint key = *it;
      //第一次筛选
      if (!(key.pt.x >= marker.mLbPt.x && key.pt.x <= marker.mRtPt.x && key.pt.y >= marker.mLbPt.y && key.pt.y <= marker.mRtPt.y)) {
	c++;
	continue;
      }

      //判断是否在四点约束的范围内
      if (isInQuadrilateral(key.pt, marker.mInPt[0], marker.mInPt[1], marker.mInPt[2], marker.mInPt[3])) {
	n++;
	int a = it - mCurrentFrame.mvKeys.begin();
	if (NULL != mCurrentFrame.mvpMapPoints[a]) {
	  if (mCurrentFrame.mvpMapPoints[a]->isBad()) {
	    a++;
	    continue;
	  }
	  float e = fabs(mCurrentFrame.mvpMapPoints[a]->mTrackProjX - key.pt.x) + fabs(mCurrentFrame.mvpMapPoints[a]->mTrackProjY - key.pt.y);
	  err_sum += e;
	  if (e > marker.getThreshold()) {
	    b++;
	    continue;
	  }
	  m++;

	  cv::Point3f pt3d;
	  pt3d.x = mCurrentFrame.mvpMapPoints[a]->GetWorldPos().at<float>(0);
	  pt3d.y = mCurrentFrame.mvpMapPoints[a]->GetWorldPos().at<float>(1);
	  pt3d.z = mCurrentFrame.mvpMapPoints[a]->GetWorldPos().at<float>(2);

	  //cout<<"-------map point:"<<mCurrentFrame.mvpMapPoints[a]->GetWorldPos()<<endl;
	  //cout<<"-----------keypt:"<<key.pt<<endl;
	  //cout<<"------------pt3d:"<<pt3d<<endl;

	  marker.mvKeyPt.push_back(key.pt);
	  marker.mvMapPt.push_back(pt3d);
	}
      } else {
	d++;
	marker.mvKeyOutPt.push_back(key.pt);
      }
    }
    cout << "  key point:" << n << endl;
    cout << "  map point:" << m << endl;
    cout << "  mean error:" << err_sum / n << "  threshold:" << marker.getThreshold() << endl;
    cout << "  bad :" << a << endl;
    cout << "  filt:" << b << endl;
    cout << "  out :" << c << endl;
    cout << "  not :" << d << endl;

  }

  return mCurrentFrame.mTcw.clone();
}

void Tracking::Track() {

  if (mState == NO_IMAGES_YET) {
    cout << "no images yet.\n";
    mState = NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  // Get Map Mutex -> Map cannot be changed
  unique_lock < mutex > lock(mpMap->mMutexMapUpdate);

  if (mState == NOT_INITIALIZED) {
    //cout<<"System is not initialized,monocular init."<<endl;
    if (mSensor == System::STEREO || mSensor == System::RGBD)
      StereoInitialization();
    else {
      MonocularInitialization();
      //UpdatePoseWithIMU();
    }

    mpFrameDrawer->Update(this);

    if (mState != OK)
      return;
  } else {
    //cout<<"System is initialized,track frame."<<endl;
    // System is initialized. Track Frame.
    bool bOK;

    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if (!mbOnlyTracking) {
      //cout<<"Local mapping is activated."<<endl;
      // Local Mapping is activated. This is the normal behaviour, unless
      // you explicitly activate the "only tracking" mode.
      //cout<<"System is initialized,track frame."<<endl;
      if (mState == OK) {
	// Local Mapping might have changed some MapPoints tracked in last frame
	CheckReplacedInLastFrame();

	if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
	  cout << "No motion model,track reference key frame." << endl;
	  bOK = TrackReferenceKeyFrame();
	} else {
	  cout << "Track with motion model." << endl;
	  bOK = TrackWithMotionModel();
	  if (!bOK) {
	    cout << "Track reference key frame." << endl;
	    bOK = TrackReferenceKeyFrame();
	  }
	}
      } else {
	cout << "Relocalization." << endl;
	bOK = Relocalization();
      }
    } 

    mCurrentFrame.mpReferenceKF = mpReferenceKF;

    //IMU fusion
    if (mState == OK) {
      if (!mCurrentFrame.mTcw.empty()) {
	//UpdatePoseWithIMU();
      }
    }

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (!mbOnlyTracking) {
      if (bOK) {
	cout << "Have an initial pose estimation,track localmap." << endl;
	bOK = TrackLocalMap();
      }
    }

    if (bOK)
      mState = OK;
    else
      mState = LOST;

    // Update drawer
    mpFrameDrawer->Update(this);

    // If tracking were good, check if we insert a keyframe
    if (bOK) {
      cout << "Tracking good,check if need to insert a keyframe." << endl;
      // Update motion model
      if (!mLastFrame.mTcw.empty()) {
	cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
	mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
	mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
	mVelocity = mCurrentFrame.mTcw * LastTwc;
	cout << "Update motion model." << endl;
      } else
	mVelocity = cv::Mat();

      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
      if (mpMapDrawer->mbDrawImuAxis) {
	mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcwIMU);
      }
      cout << "Set current camera pose." << endl;
      cout << mCurrentFrame.mTcw << endl;

      // Clean temporal point matches
      for (int i = 0; i < mCurrentFrame.N; i++) {
	MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
	if (pMP)
	  if (pMP->Observations() < 1) {
	    mCurrentFrame.mvbOutlier[i] = false;
	    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
	  }
      }

      // Delete temporal MapPoints
      for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++) {
	MapPoint* pMP = *lit;
	delete pMP;
      }
      mlpTemporalPoints.clear();

      // Check if we need to insert a new keyframe
      if (NeedNewKeyFrame()) {
	cout << "Need new key frame,create one." << endl;
	CreateNewKeyFrame();
      }

      // We allow points with high innovation (considererd outliers by the Huber Function)
      // pass to the new keyframe, so that bundle adjustment will finally decide
      // if they are outliers or not. We don't want next frame to estimate its position
      // with those points so we discard them in the frame.
      for (int i = 0; i < mCurrentFrame.N; i++) {
	if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
	  mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
      }
    }

    // Reset if the camera get lost soon after initialization
    if (mState == LOST) {
      if (mpMap->KeyFramesInMap() <= 5) {
	cout << "Track lost soon after initialisation, reseting..." << endl;
	mpSystem->Reset();
	return;
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    mLastFrame = Frame(mCurrentFrame);
  }

  // Store frame pose information to retrieve the complete camera trajectory afterwards.	
  if (!mCurrentFrame.mTcw.empty()) {
    cout << "Store frame pose to retrieve the camera trajectory" << endl;
    cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
    cout << Tcr << endl;
    mlRelativeFramePoses.push_back(Tcr);
    mlpReferences.push_back(mpReferenceKF);
    mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    mlbLost.push_back(mState == LOST);
  } else {
    // This can happen if tracking is lost
    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    mlpReferences.push_back(mlpReferences.back());
    mlFrameTimes.push_back(mlFrameTimes.back());
    mlbLost.push_back(mState == LOST);
  }

  //cout<<"-------------------------------------------------"<<endl;

}

void Tracking::StereoInitialization() {
  if (mCurrentFrame.N > 500) {
    // Set Frame pose to the origin
    mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

    // Create KeyFrame
    KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    // Insert KeyFrame in the map
    mpMap->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to KeyFrame
    for (int i = 0; i < mCurrentFrame.N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
	cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
	MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpMap);
	pNewMP->AddObservation(pKFini, i);
	pKFini->AddMapPoint(pNewMP, i);
	pNewMP->ComputeDistinctiveDescriptors();
	pNewMP->UpdateNormalAndDepth();
	mpMap->AddMapPoint(pNewMP);

	mCurrentFrame.mvpMapPoints[i] = pNewMP;
      }
    }

    cout << "New map created with " << mpMap->MapPointsInMap() << " points"
	 << endl;

    mpLocalMapper->InsertKeyFrame(pKFini);

    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;

    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

    mState = OK;
  }
}

void Tracking::MonocularInitialization() {
  //cerr<<"		MonocularInitialization"<<endl;
  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() > 100) {
      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
	mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      if (mpInitializer)
	delete mpInitializer;

      mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

      return;
    }
  } else {
    // Try to initialize
    if ((int) mCurrentFrame.mvKeys.size() <= 100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame,
						   mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches < 100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      return;
    }
    //cout<<"[mono_init]Matching for map initialization,find enough correspondences."<<endl;

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    //1.Computes in parallel a fundamental matrix and a homography
    //2.Selects a model and tries to recover the motion and the structure from motion
    if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw,
				  mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
	if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
	  mvIniMatches[i] = -1;
	  nmatches--;
	}
      }

      // Set Frame Poses
      mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
      cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
      Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
      tcw.copyTo(Tcw.rowRange(0, 3).col(3));
      mCurrentFrame.SetPose(Tcw);

      //sst
      //UpdatePoseWithIMU();

      cout << "------------------------Track-------------------------" << endl;
      cout << "System is not initialized,monocular init." << endl;
      cout << "Find enough matched points,can recover motion structure." << endl;
      cout << "currentFramePose:" << endl;
      cout << Tcw << endl;
      CreateInitialMapMonocular();
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
  KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  // Insert KFs in the map
  cout << "[CreateInitialMap]Add key frame." << endl;
  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and associate to keyframes
  cout << "[CreateInitialMap]Create map points and asscoiate to keyframes."
       << endl;
  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0)
      continue;

    //Create MapPoint.
    cv::Mat worldPos(mvIniP3D[i]);

    MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    //Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    //Add to Map
    mpMap->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  // Bundle Adjustment
  cout << "[CreateInitialMap]New Map created with " << mpMap->MapPointsInMap()
       << " points" << endl;

  Optimizer::GlobalBundleAdjustemnt(mpMap, 20);
  cout << "[CreateInitialMap]Global BA optimizer." << endl;

  // Set median depth to 1
  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth = 1.0f / medianDepth;
  cout << "[CreateInitialMap]Compute scene median depth:" << medianDepth
       << "  normalize:" << invMedianDepth << endl;

  if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
    cout << "Wrong initialization, reseting..." << endl;
    Reset();
    return;
  }

  // Scale initial baseline
  cv::Mat Tc2w = pKFcur->GetPose();
  Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
  pKFcur->SetPose(Tc2w);
  cout << "[CreateInitialMap]Scale initial baseline." << endl;

  // Scale points
  vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
  for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
      MapPoint* pMP = vpAllMapPoints[iMP];
      cout << "[CreateInitialMap]world pose:" << pMP->GetWorldPos()
	   << "  " << iMP << endl;
      pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      cout << "[CreateInitialMap]set world pose:" << pMP->GetWorldPos()
	   << "  " << iMP << endl;
    }
  }
  cout << "[CreateInitialMap]Scale points:" << vpAllMapPoints.size() << endl;

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);
  cout << "[CreateInitialMap]Insert key frame." << endl;

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  mLastFrame = Frame(mCurrentFrame);

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

  cout << "[CreateInitialMap]Set current camera pose:" << endl;
  cout << pKFcur->GetPose() << endl;

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i = 0; i < mLastFrame.N; i++) {
    MapPoint* pMP = mLastFrame.mvpMapPoints[i];

    if (pMP) {
      MapPoint* pRep = pMP->GetReplaced();
      if (pRep) {
	mLastFrame.mvpMapPoints[i] = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  // Compute Bag of Words vector
  mCurrentFrame.ComputeBoW();
  cout << "[TrackReferenceKeyFrame]Compute Bag of Words vector.\n";

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.7, true);
  vector<MapPoint*> vpMapPointMatches;

  int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame,
				     vpMapPointMatches);
  cout << "[TrackReferenceKeyFrame]ORB match...\n";

  if (nmatches < 15)
    return false;

  mCurrentFrame.mvpMapPoints = vpMapPointMatches;
  mCurrentFrame.SetPose(mLastFrame.mTcw);
  cout << "[TrackReferenceKeyFrame]set pose:\n";
  //UpdatePoseWithIMU();
  

  Optimizer::PoseOptimization(&mCurrentFrame);
  cout << "[TrackReferenceKeyFrame]Pose optimize:\n";
  cout << mCurrentFrame.mK << endl;

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
	MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

	mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
	mCurrentFrame.mvbOutlier[i] = false;
	pMP->mbTrackInView = false;
	pMP->mnLastFrameSeen = mCurrentFrame.mnId;
	nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
	nmatchesMap++;
    }
  }

  cout << "[TrackReferenceKeyFrame]Discard outliers.\n";

  return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  KeyFrame* pRef = mLastFrame.mpReferenceKF;
  cv::Mat Tlr = mlRelativeFramePoses.back();

  mLastFrame.SetPose(Tlr * pRef->GetPose());

  if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR)
    return;

  // Create "visual odometry" MapPoints
  // We sort points according to their measured depth by the stereo/RGB-D sensor
  vector<pair<float, int> > vDepthIdx;
  vDepthIdx.reserve(mLastFrame.N);
  for (int i = 0; i < mLastFrame.N; i++) {
    float z = mLastFrame.mvDepth[i];
    if (z > 0) {
      vDepthIdx.push_back(make_pair(z, i));
    }
  }

  if (vDepthIdx.empty())
    return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // We insert all close points (depth<mThDepth)
  // If less than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;

    bool bCreateNew = false;

    MapPoint* pMP = mLastFrame.mvpMapPoints[i];
    if (!pMP)
      bCreateNew = true;
    else if (pMP->Observations() < 1) {
      bCreateNew = true;
    }

    if (bCreateNew) {
      cv::Mat x3D = mLastFrame.UnprojectStereo(i);
      MapPoint* pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

      mLastFrame.mvpMapPoints[i] = pNewMP;

      mlpTemporalPoints.push_back(pNewMP);
      nPoints++;
    } else {
      nPoints++;
    }

    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
      break;
  }
}

bool Tracking::TrackWithMotionModel() {
  ORBmatcher matcher(0.9, true);

  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points
  UpdateLastFrame();
  cout << "[TrackWithMotionModel]Update last frame pose according to its reference keyframe." << endl;
  cout << "[TrackWithMotionModel]set pose:" << endl;
  mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
  cout << mVelocity << endl << mLastFrame.mTcw << endl << mCurrentFrame.mTcw << endl;
  //UpdatePoseWithIMU();
  

  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(),
       static_cast<MapPoint*>(NULL));

  // Project points seen in previous frame
  int th;
  if (mSensor != System::STEREO)
    th = 15;
  else
    th = 7;
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th,
					    mSensor == System::MONOCULAR);

  // If few matches, uses a wider window search
  if (nmatches < 20) {
    fill(mCurrentFrame.mvpMapPoints.begin(),
	 mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th,
					  mSensor == System::MONOCULAR);
  }

  if (nmatches < 20)
    return false;

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);
  cout << "[TrackWithMotionModel]Pose optimize:" << endl;
  cout << mCurrentFrame.mTcw << endl;

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
	MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

	mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
	mCurrentFrame.mvbOutlier[i] = false;
	pMP->mbTrackInView = false;
	pMP->mnLastFrameSeen = mCurrentFrame.mnId;
	nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
	nmatchesMap++;
    }
  }

  if (mbOnlyTracking) {
    mbVO = nmatchesMap < 10;
    return nmatches > 20;
  }

  cout << "[TrackWithMotionModel]Discard outliers." << endl;

  return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap() {
  // We have an estimation of the camera pose and some map points tracked in the frame.
  // We retrieve the local map and try to find matches to points in the local map.

  UpdateLocalMap();

  SearchLocalPoints();

  // Optimize Pose
  Optimizer::PoseOptimization(&mCurrentFrame);
  mnMatchesInliers = 0;

  // Update MapPoints Statistics
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
	mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
	if (!mbOnlyTracking) {
	  if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
	    mnMatchesInliers++;
	} else
	  mnMatchesInliers++;
      } else if (mSensor == System::STEREO)
	mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

    }
  }

  cout << "TrackLocalMap:\n";
  cout << "  current frame id:" << mCurrentFrame.mnId << endl;
  cout << "  last frame id:" << mnLastRelocFrameId << " maxframes:"
       << mMaxFrames << endl;
  cout << "  matches inliers:" << mnMatchesInliers << endl;

  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames
      && mnMatchesInliers < 50)
    return false;

  if (mnMatchesInliers < 30)
    return false;
  else
    return true;
}

bool Tracking::NeedNewKeyFrame() {
  if (mbOnlyTracking)
    return false;

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

  const int nKFs = mpMap->KeyFramesInMap();

  // Do not insert keyframes if not enough frames have passed from last relocalisation
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames
      && nKFs > mMaxFrames)
    return false;

  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs <= 2)
    nMinObs = 2;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  // Local Mapping accept keyframes?
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
  // "total matches = matches to map + visual odometry matches"
  // Visual odometry matches will become MapPoints if we insert a keyframe.
  // This ratio measures how many MapPoints we could create if we insert a keyframe.
  int nMap = 0;
  int nTotal = 0;
  if (mSensor != System::MONOCULAR) {
    for (int i = 0; i < mCurrentFrame.N; i++) {
      if (mCurrentFrame.mvDepth[i] > 0
	  && mCurrentFrame.mvDepth[i] < mThDepth) {
	nTotal++;
	if (mCurrentFrame.mvpMapPoints[i])
	  if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
	    nMap++;
      }
    }
  } else {
    // There are no visual odometry matches in the monocular case
    nMap = 1;
    nTotal = 1;
  }

  const float ratioMap = (float) nMap / fmax(1.0f, nTotal);

  // Thresholds
  float thRefRatio = 0.75f;
  if (nKFs < 2)
    thRefRatio = 0.4f;

  if (mSensor == System::MONOCULAR)
    thRefRatio = 0.9f;

  float thMapRatio = 0.35f;
  if (mnMatchesInliers > 300)
    thMapRatio = 0.20f;

  // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames
		    && bLocalMappingIdle);
  //Condition 1c: tracking is weak
  const bool c1c = mSensor != System::MONOCULAR
    && (mnMatchesInliers < nRefMatches * 0.25 || ratioMap < 0.3f);
  // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
  const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio
		    || ratioMap < thMapRatio) && mnMatchesInliers > 15);

  if ((c1a || c1b || c1c) && c2) {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      if (mSensor != System::MONOCULAR) {
	if (mpLocalMapper->KeyframesInQueue() < 3)
	  return true;
	else
	  return false;
      } else
	return false;
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame() {
  if (!mpLocalMapper->SetNotStop(true))
    return;

  KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  if (mSensor != System::MONOCULAR) {
    mCurrentFrame.UpdatePoseMatrices();

    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    vector<pair<float, int> > vDepthIdx;
    vDepthIdx.reserve(mCurrentFrame.N);
    for (int i = 0; i < mCurrentFrame.N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
	vDepthIdx.push_back(make_pair(z, i));
      }
    }

    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());

      int nPoints = 0;
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
	int i = vDepthIdx[j].second;

	bool bCreateNew = false;

	MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
	if (!pMP)
	  bCreateNew = true;
	else if (pMP->Observations() < 1) {
	  bCreateNew = true;
	  mCurrentFrame.mvpMapPoints[i] =
	    static_cast<MapPoint*>(NULL);
	}

	if (bCreateNew) {
	  cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
	  MapPoint* pNewMP = new MapPoint(x3D, pKF, mpMap);
	  pNewMP->AddObservation(pKF, i);
	  pKF->AddMapPoint(pNewMP, i);
	  pNewMP->ComputeDistinctiveDescriptors();
	  pNewMP->UpdateNormalAndDepth();
	  mpMap->AddMapPoint(pNewMP);

	  mCurrentFrame.mvpMapPoints[i] = pNewMP;
	  nPoints++;
	} else {
	  nPoints++;
	}

	if (vDepthIdx[j].first > mThDepth && nPoints > 100)
	  break;
      }
    }
  }

  mpLocalMapper->InsertKeyFrame(pKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  for (vector<MapPoint*>::iterator vit = mCurrentFrame.mvpMapPoints.begin(),
	 vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++) {
    MapPoint* pMP = *vit;
    if (pMP) {
      if (pMP->isBad()) {
	*vit = static_cast<MapPoint*>(NULL);
      } else {
	pMP->IncreaseVisible();
	pMP->mnLastFrameSeen = mCurrentFrame.mnId;
	pMP->mbTrackInView = false;
      }
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(), vend =
	 mvpLocalMapPoints.end(); vit != vend; vit++) {
    MapPoint* pMP = *vit;
    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
      continue;
    if (pMP->isBad())
      continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }

  if (nToMatch > 0) {
    ORBmatcher matcher(0.8);
    int th = 1;
    if (mSensor == System::RGBD)
      th = 3;
    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
      th = 5;
    matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();

  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(),
	 itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
    KeyFrame* pKF = *itKF;
    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for (vector<MapPoint*>::const_iterator itMP = vpMPs.begin(), itEndMP =
	   vpMPs.end(); itMP != itEndMP; itMP++) {
      MapPoint* pMP = *itMP;
      if (!pMP)
	continue;
      if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
	continue;
      if (!pMP->isBad()) {
	mvpLocalMapPoints.push_back(pMP);
	pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
      }
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  map<KeyFrame*, int> keyframeCounter;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
      if (!pMP->isBad()) {
	const map<KeyFrame*, size_t> observations =
	  pMP->GetObservations();
	for (map<KeyFrame*, size_t>::const_iterator it =
	       observations.begin(), itend = observations.end();
	     it != itend; it++)
	  keyframeCounter[it->first]++;
      } else {
	mCurrentFrame.mvpMapPoints[i] = NULL;
      }
    }
  }

  if (keyframeCounter.empty())
    return;

  int max = 0;
  KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
  for (map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(),
	 itEnd = keyframeCounter.end(); it != itEnd; it++) {
    KeyFrame* pKF = it->first;

    if (pKF->isBad())
      continue;

    if (it->second > max) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(it->first);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }

  // Include also some not-already-included keyframes that are neighbors to already-included keyframes
  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(),
	 itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80)
      break;

    KeyFrame* pKF = *itKF;

    const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(),
	   itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF;
	 itNeighKF++) {
      KeyFrame* pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
	if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
	  mvpLocalKeyFrames.push_back(pNeighKF);
	  pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
	  break;
	}
      }
    }

    const set<KeyFrame*> spChilds = pKF->GetChilds();
    for (set<KeyFrame*>::const_iterator sit = spChilds.begin(), send =
	   spChilds.end(); sit != send; sit++) {
      KeyFrame* pChildKF = *sit;
      if (!pChildKF->isBad()) {
	if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
	  mvpLocalKeyFrames.push_back(pChildKF);
	  pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
	  break;
	}
      }
    }

    KeyFrame* pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
	mvpLocalKeyFrames.push_back(pParent);
	pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
	break;
      }
    }

  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  // Compute Bag of Words Vector
  cout << "[Relocalization]Compute Bag of Words Vector." << endl;
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
  vector<KeyFrame*> vpCandidateKFs =
    mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
  cout << "[Relocalization]Query KeyFrame Database for keyframe candidates."
       << endl;

  if (vpCandidateKFs.empty())
    return false;

  const int nKFs = vpCandidateKFs.size();

  // We perform first an ORB matching with each candidate
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.75, true);

  cout << "[Relocalization]If enough matches are found,setup a PnP solver."
       << endl;

  vector<PnPsolver*> vpPnPsolvers;
  vpPnPsolvers.resize(nKFs);

  vector<vector<MapPoint*> > vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame* pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame,
					 vvpMapPointMatches[i]);
      if (nmatches < 15) {
	vbDiscarded[i] = true;
	continue;
      } else {
	PnPsolver* pSolver = new PnPsolver(mCurrentFrame,
					   vvpMapPointMatches[i]);
	pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
	vpPnPsolvers[i] = pSolver;
	nCandidates++;
      }
    }
  }

  cout
    << "[Relocalization]Alternatively perform some iterations of P4P RANSAC."
    << endl;

  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  ORBmatcher matcher2(0.9, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i])
	continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      PnPsolver* pSolver = vpPnPsolvers[i];
      cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
	vbDiscarded[i] = true;
	nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (!Tcw.empty()) {
	Tcw.copyTo(mCurrentFrame.mTcw);

	set<MapPoint*> sFound;

	const int np = vbInliers.size();

	for (int j = 0; j < np; j++) {
	  if (vbInliers[j]) {
	    mCurrentFrame.mvpMapPoints[j] =
	      vvpMapPointMatches[i][j];
	    sFound.insert(vvpMapPointMatches[i][j]);
	  } else
	    mCurrentFrame.mvpMapPoints[j] = NULL;
	}

	int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

	if (nGood < 10)
	  continue;

	for (int io = 0; io < mCurrentFrame.N; io++)
	  if (mCurrentFrame.mvbOutlier[io])
	    mCurrentFrame.mvpMapPoints[io] =
	      static_cast<MapPoint*>(NULL);

	// If few inliers, search by projection in a coarse window and optimize again
	if (nGood < 50) {
	  int nadditional = matcher2.SearchByProjection(mCurrentFrame,
							vpCandidateKFs[i], sFound, 10, 100);

	  if (nadditional + nGood >= 50) {
	    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

	    // If many inliers but still not enough, search by projection again in a narrower window
	    // the camera has been already optimized with many points
	    if (nGood > 30 && nGood < 50) {
	      sFound.clear();
	      for (int ip = 0; ip < mCurrentFrame.N; ip++)
		if (mCurrentFrame.mvpMapPoints[ip])
		  sFound.insert(
				mCurrentFrame.mvpMapPoints[ip]);
	      nadditional = matcher2.SearchByProjection(
							mCurrentFrame, vpCandidateKFs[i], sFound, 3,
							64);

	      // Final optimization
	      if (nGood + nadditional >= 50) {
		nGood = Optimizer::PoseOptimization(
						    &mCurrentFrame);

		for (int io = 0; io < mCurrentFrame.N; io++)
		  if (mCurrentFrame.mvbOutlier[io])
		    mCurrentFrame.mvpMapPoints[io] = NULL;
	      }
	    }
	  }
	}

	// If the pose is supported by enough inliers stop ransacs and continue
	if (nGood >= 50) {
	  bMatch = true;
	  break;
	}
      }
    }
  }

  if (!bMatch) {
    return false;
  } else {
    cout
      << "[Relocalization]found a camera pose supported by enough inliers."
      << endl;
    mnLastRelocFrameId = mCurrentFrame.mnId;
    return true;
  }

}

void Tracking::Reset() {
  cout << "System Reseting" << endl;
  if (mpSystem->UseViewer()) {

    mpViewer->RequestStop();

    while (!mpViewer->isStopped()) {
      cout << ".";
      usleep(3000);
    }
  }
  // Reset Local Mapping
  cout << "Reseting Local Mapper...";
  mpLocalMapper->RequestReset();
  cout << " done" << endl;

  // Reset Loop Closing
  cout << "Reseting Loop Closing...";
  mpLoopClosing->RequestReset();
  cout << " done" << endl;

  // Clear BoW Database
  cout << "Reseting Database...";
  mpKeyFrameDB->clear();
  cout << " done" << endl;

  // Clear Map (this erase MapPoints and KeyFrames)
  mpMap->clear();

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = NO_IMAGES_YET;

  if (mpInitializer) {
    delete mpInitializer;
    mpInitializer = static_cast<Initializer*>(NULL);
  }

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();

  mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) {
  mbOnlyTracking = flag;
}

Map *Tracking::getMap() {
  return mpMap;
}

void Tracking::AddImuData() {
  // Get IMU Mutex -> IMU data cannot be changed
  //unique_lock<mutex> lock(mpImu->mMutexImu);

  //偏离角度
  float pitch = (float) (mCurrentFrame.mImuData.pitch);
  float roll = (float) (mCurrentFrame.mImuData.roll);
  float yaw = (float) (mCurrentFrame.mImuData.yaw);
  cout << "  p/r/y:" << pitch << "  " << roll << "  " << yaw << endl;

  float data[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, };
  mCurrentFrame.mTcwIMU = cv::Mat(4, 4, CV_32FC1, data);

  //heading-pitch-bank:四元数转欧拉角的顺序
  //yaw     pitch roll
  //float angle[] = {yaw/57.3f, pitch/57.3f, roll/57.3f};
  //float angle[] = {-yaw/57.3f, -roll/57.3f, pitch/57.3f};//right
  float angle[] = { -yaw / 57.3f, roll / 57.3f, -pitch / 57.3f };
  cv::Mat R2 = Euler2Matrix(angle);
  R2.copyTo(mCurrentFrame.mTcwIMU);

  cout << "  Euler:\n" << cv::Mat(3, 1, CV_32FC1, angle).t() * 57.3f << endl;
  cout << "  Angle:" << cv::Mat(1, 3, CV_32FC1, angle) << endl;
  cout << "  Euler2Matrix:\n" << R2 << endl;
  cout << " Matrix2Eular:\n"  << Matrix2Euler(mCurrentFrame.mTcwIMU).t() * 57.3f << endl;

  float y[] = { 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };
  cv::Mat y_ = cv::Mat(4, 4, CV_32FC1, y);
  mCurrentFrame.mTcwIMU = y_ * mCurrentFrame.mTcwIMU;
  //cout<<"mTcwIMU:\n"<<mCurrentFrame.mTcwIMU<<endl;
}

void Tracking::UpdatePoseWithIMU() {
  cout << "Update pose with imu:" << micros() << endl;

  if (mState == OK) {
    cv::Mat initTcw =	mInitialFrame.mTcw.rowRange(0, 3).colRange(0, 3).clone();
    cv::Mat initTcwIMU =mInitialFrame.mTcwIMU.rowRange(0, 3).colRange(0, 3).clone();
    cv::Mat curTcw =	mCurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3).clone();
    cv::Mat curTcwIMU = mCurrentFrame.mTcwIMU.rowRange(0, 3).colRange(0, 3).clone();
    cout << "  initTcw:\n" << initTcw << endl;
    cout << "  curTcw:\n" << curTcw << endl;
    cout << "  initTcwImu:\n" << initTcwIMU << endl;
    cout << "  curTcwImu:\n" << curTcwIMU << endl;

    cv::Mat R1, R2;
    cv::Rodrigues(initTcw, R1);
    cv::Rodrigues(curTcw, R2);
    cout << "--------------------------" << endl;
    cout << " t init:" << R1.t() * 57.3f << endl;
    cout << " t current:" << R2.t() * 57.3f << endl;
    cout << " t :" << (R2 - R1).t() * 57.3f << endl;

    cv::Mat R3, R4;
    cv::Rodrigues(initTcwIMU, R3);
    cv::Rodrigues(curTcwIMU, R4);
    //R3 = Matrix2Euler(mInitialFrame.mTcwIMU);
    //R4 = Matrix2Euler(mCurrentFrame.mTcwIMU);
    cout << " t initIMU:" << R3.t() * 57.3f << endl;
    cout << " t curIMU:" << R4.t() * 57.3f << endl;
    cout << " t IMU:" << (R4 - R3).t() * 57.3f << endl;
    cout << "--------------------------" << endl;

    
    //
    cv::Mat T1 = (R2 - R1).t() * 57.3f;
    cv::Mat T2 = (R4 - R3).t() * 57.3f;
    //fprintf(stderr,"\n[slam:%3.0f %3.0f %3.0f]\n",T1.at<float>(0),T1.at<float>(1),T1.at<float>(2));
    //fprintf(stderr,  "[ imu:%3.0f %3.0f %3.0f]\n\n",T2.at<float>(0),T2.at<float>(1),T2.at<float>(2));

    cv::Mat Rtrans = initTcwIMU.inv() * curTcwIMU;
    cv::Mat newR = Rtrans * initTcw;

    //先平移，再旋转？
    cv::Mat T = mCurrentFrame.mTcw.rowRange(0,3).col(3);
    cv::Mat newT = newR.t() * T;
    cv::Mat Ttrans = (newT - T)*1000.f;
    fprintf(stderr,  "[slam :%5.3f %5.3f %5.3f]\n",T.at<float>(0),T.at<float>(1),T.at<float>(2));
    fprintf(stderr,  "[ imu :%5.3f %5.3f %5.3f]\n",newT.at<float>(0),newT.at<float>(1),newT.at<float>(2));
    fprintf(stderr,  "[trans:%3.0f %3.0f %3.0f]\n\n",Ttrans.at<float>(0),Ttrans.at<float>(1),Ttrans.at<float>(2));

    cv::Mat R5;
    cv::Rodrigues(Rtrans, R5);
    cout<<" t trans:"<<R5<<endl;

    newR.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    newT.copyTo(mCurrentFrame.mTcw.rowRange(0,3).col(3));
    cout << mCurrentFrame.mTcw << endl;
  }

  cout << "  " << micros() << endl;
}

} //namespace ORB_SLAM

