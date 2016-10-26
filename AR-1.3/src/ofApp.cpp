#include "ofApp.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Marker.h"

#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

#define INIT     0
#define TRACKING 1
#define LOST     2 

#define UP       357
#define DOWN     359
#define LEFT     356
#define RIGHT    358
#define PAGEUP   360
#define PAGEDOWN 361

//marker对角线长度185mm
//#define MARKERLEN 185
#define MARKERLEN 330
#define STEP 5

void cvFitPlane(const CvMat* points, float* plane){  
  // Estimate geometric centroid.  
  int nrows = points->rows;  
  int ncols = points->cols;  
  int type = points->type;  
  CvMat* centroid = cvCreateMat(1, ncols, type);  
  cvSet(centroid, cvScalar(0));  
  for (int c = 0; c<ncols; c++) {  
    for (int r = 0; r < nrows; r++) {  
      centroid->data.fl[c] += points->data.fl[ncols*r + c];  
    }  
    centroid->data.fl[c] /= nrows;  
  }  
  // Subtract geometric centroid from each point.  
  CvMat* points2 = cvCreateMat(nrows, ncols, type);  
  for (int r = 0; r<nrows; r++)  
    for (int c = 0; c<ncols; c++)  
      points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];  
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
  for (int c = 0; c<ncols; c++){  
    plane[c] = V->data.fl[ncols*(ncols - 1) + c];  
    plane[ncols] += plane[c] * centroid->data.fl[c];  
  }  
  // Release allocated resources.  
  cvReleaseMat(&centroid);  
  cvReleaseMat(&points2);  
  cvReleaseMat(&A);  
  cvReleaseMat(&W);  
  cvReleaseMat(&V);  
} 

void ofApp::computeIntersection(Mat ptIn,Mat& ptOut,Mat plane)
{
  if( !plane.empty() ) {
    float a = plane.at<float>(0);
    float b = plane.at<float>(1);
    float c = plane.at<float>(2);
    float d = plane.at<float>(3);
    float x0 = ptIn.at<float>(0);
    float y0 = ptIn.at<float>(1);
    float z0 = ptIn.at<float>(2);

    float t = -(a*x0 + b*y0 + c*z0 +d)/(a*a + b*b +c*c);
    float x = a*t + x0;
    float y = b*t + y0;
    float z = c*t + z0;
    float p[]={x,y,z,1};
    Mat pt(4,1,CV_32FC1,p);
    pt.copyTo(ptOut);
  }

  //点到点的移动
  ptIn.copyTo(ptOut);
}

void ofApp::computeMoveAngle(cv::Point3f pt1, cv::Point3f pt2,float &angle)
{
  //直线在xz平面投影是否与z轴垂直
  if(fabs(pt1.x-pt2.x) < 0.0000001) {
    if((pt1.z-pt2.z) < 0.0000001)
      angle = 180;
    else 
      angle = 0;
    return;
  }

  //确定在xz平面的投影直线,y=ax+b    
  float a = (pt1.z-pt2.z)/(pt1.x-pt2.x);
  float b = pt1.z-a*pt1.x;
  if (m_bDebug) {
    cout<<" pt1:"<<pt1<<endl;
    cout<<" pt2:"<<pt2<<endl;
    cout<<" move line:y="<<a<<"x+"<<b<<endl;
  }
  angle = -atanf(1/a);
  angle *= 180/PI;
  if(pt1.z < pt2.z)
    angle += 180;
}

cv::Mat fitPlane(std::vector<cv::Mat> vPt)
{
  CvMat*points_mat = cvCreateMat(vPt.size(), 3, CV_32FC1);
  for (int i=0;i < vPt.size(); ++i) {  
    points_mat->data.fl[i*3 + 0] = vPt[i].at<float>(0);
    points_mat->data.fl[i*3 + 1] = vPt[i].at<float>(1);
    points_mat->data.fl[i*3 + 2] = vPt[i].at<float>(2);

    //points_mat->data.fl[i*3+0] = mvMapPt[i].x * 1000.0f;
    //points_mat->data.fl[i*3 + 1] =  mvMapPt[i].y * 1000.0f;
    //points_mat->data.fl[i*3 + 2] =  mvMapPt[i].z * 1000.0f;
  }  

  //最小二乘法拟合平面
  float plane[4] = { 0 };
  cvFitPlane(points_mat, plane);
  cerr<<"pt mat:"<<Mat(points_mat)<<endl;
  cerr<<"plane:";
  for(int i=0;i<4;i++)
    cerr<<"plane:"<<plane[i]<<"  ";
  cerr<<endl;

  //normalize
  float r = 1/(fabs(plane[3])+0.000001);
  plane[0] =  plane[0]*r;
  plane[1] =  plane[1]*r;
  plane[2] =  plane[2]*r;
  plane[3] = 1.0;

  cv::Mat result(1,4,CV_32FC1,plane);
  cerr<<"result:"<<result<<endl;
  
  return result;
}


//--------------------------------------------------------------
void ofApp::setup(){
  cout<<"setup"<<endl;

//m_bDebug = false;
  m_bDetected = false;
  m_bAdd = false;
  m_bMove = false;
  m_bAutoMove = false;
  m_nMoveIndex = 0;
  m_scale = 0.001;
  m_LoopCount = 0;

  m_bShow1 = true;
  m_bShow2 = true;

  float o[] = {0,0,0,1};
  m_pt2 = Mat(4,1,CV_32FC1,o);
  m_pt3 = Mat(4,1,CV_32FC1,o);

  //
  //pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

  //slam state:0-init 1-tracking 2-lost
  m_state = INIT;
  //1.orb-slam2 init
  m_pSLAM = new ORB_SLAM2::System("data/ORBvoc.txt","data/logitech.yaml",ORB_SLAM2::System::MONOCULAR,false);
  //
	
  ofSetLogLevel(OF_LOG_VERBOSE);
  ofBackground(50, 0);
  ofSetWindowShape(640, 480);

  // we need GL_TEXTURE_2D for our models coords.
  ofDisableArbTex(); 

  //videw capture
  m_video.setDeviceID(0);
  m_video.initGrabber(640, 480);

  //camera paras
  float camera_matrix[] = 
    {
      512.469f, 0.0f, 246.672f,
      0.0f, 499.080f, 215.485f,
      0.0f, 0.0f, 1.0f
    };
  float dist_coeff[] = {0.012652f, 0.438642f, -0.030902f, -0.045261f};

  //Mat构造函数不会拷贝数据，只会将指针指向数据区域，所以对于局部变量内存，需要clone
  m_camera_matrix = Mat(3, 3, CV_32FC1, camera_matrix).clone();
  m_dist_coeff = Mat(1, 4, CV_32FC1, dist_coeff).clone();

  //load target image
  //char *str = "/home/sst/work/data/imagelib/lena.jpg";
  char *str = "/home/sst/work/data/image/zazhi.jpg";
  m_target = imread(str);
  if(m_target.empty())
    cerr<<"load image faild:"<<str<<endl;

  //load 3d model
  //m_model.loadModel("squirrel/NewSquirrel.3ds", 20);

  m_model.loadModel("astroBoy_walk.dae");
  //m_model.loadModel("R2D2 by abrock/R2D2.DAE");
  m_model.setRotation(0,180,0,1,0);
  m_model.setLoopStateForAllAnimations(OF_LOOP_NORMAL);
  m_model.playAllAnimations();

  //m_model2.loadModel("dwarf.x");
  //m_model2.loadModel("Dragon/Dragon.3ds");
  m_model2.loadModel("Flag/RedFlagModel.DAE");
  //m_model2.setScale(0.001,0.001,0.001);
  m_model2.setRotation(0,180,0,1,0);
  m_model2.setRotation(0,-90,1,0,0);
  m_model2.setLoopStateForAllAnimations(OF_LOOP_NORMAL);
  m_model2.playAllAnimations();
  
}

//--------------------------------------------------------------
void ofApp::update()
{
  m_video.update();


  
  if (m_video.isFrameNew()) {
    bool good_format = true;
    int width = m_video.getWidth();
    int height = m_video.getHeight();
    m_img_gray.create(height, width, CV_8UC1);

    ofPixelFormat format = m_video.getPixelFormat();
    switch (format) {
    case OF_PIXELS_RGB:
      m_img_color.create(height, width, CV_8UC3);
      memcpy(m_img_color.data, (void*)m_video.getPixels(), height*width*3);
      cvtColor(m_img_color, m_img_gray, CV_RGB2GRAY);
      break;
    case OF_PIXELS_RGBA:
      m_img_color.create(height, width, CV_8UC4);
      memcpy(m_img_color.data, (void*)m_video.getPixels(), height*width*4);
      cvtColor(m_img_color, m_img_gray, CV_RGBA2GRAY);
      break;
    case OF_PIXELS_BGRA:
      m_img_color.create(height, width, CV_8UC4);
      memcpy(m_img_color.data, (void *)m_video.getPixels(), height*width*4);
      cvtColor(m_img_color, m_img_gray, CV_BGRA2GRAY);
      break;
    default:
      good_format = false;
      cout<<"Unsupported video format!"<<endl;
      break;
    }
    if (!good_format) 
      return;

    //track
    cv::Mat im(m_img_color);
    if(im.empty())
      return;
    //m_camPose = m_pSLAM->TrackMonocular(im,30);
    //m_camPose = m_pSLAM->TrackMonocular(im,30,m_target);

    
    //输入：四个点的卡片，反映射的误差
    //输出：keyPoints
    //过程：
    /*1.检测卡片：已知卡片与frame做特征匹配，如果找到卡片，返回frame中四个顶点坐标
     *2.提取keypoint：在orbslam中，四顶点连接成四边形,判断点是否在四边形内,求四边形中的keypoint和MapPoint
     */


    //检测目标,track
    if(!m_bDetected){
      Marker marker(m_camera_matrix,m_dist_coeff);
      marker.setTargetImage(m_target);
      marker.setThreshold(3);
      if(marker.Match(im,1.5,0.9)){
	cout<<"track with marker...\n";
	cout<<"--marker size:"<<marker.mInPt.size()<<endl;
	cout<<"--marker input:"<<marker.mInPt<<endl;
	m_camPose = m_pSLAM->TrackMonocularEx(im,30,marker);
	cout<<"--KeyPt size:"<<marker.mvKeyPt.size()<<endl;
	if(marker.mvKeyPt.size() <= 20)
	  return;

	//拟合平面，计算原点
	cv::Mat matPlane = marker.get3dPlane();
	cv::Mat matCenter = marker.get3dCenter();

	matPlane.copyTo(m_movePlane);

	float o[4];
	o[0] = matCenter.at<float>(0);
	o[1] = matCenter.at<float>(1);
	o[2] = matCenter.at<float>(2);
	o[3] = 1.0;
	//内存必须拷贝到m_pt,如果直接赋值，在draw中将得不到数据
	Mat pt = Mat(4,1,CV_32F,o);
	pt.copyTo(m_pt);
	pt.copyTo(m_pt1);

	o[0] = matPlane.at<float>(0);
	o[1] = matPlane.at<float>(1);
	o[2] = matPlane.at<float>(2);
	o[3] = matPlane.at<float>(3);
	Mat direction = Mat(4,1,CV_32F,o);
	direction.copyTo(m_direction);

	float w2ps = marker.getScale();
	float ns = m_model.getNormalizedScale();
	float dlen= marker.getDiagonalLenth();
	float scale = 1/ns*dlen/8000;
	aiVector3D scene_min, scene_max;
	m_model.getBoundingBoxWithMinVector(&scene_min, &scene_max);
	cout<<scene_min.x<<" "<<scene_min.y<<" "<<scene_min.z<<endl;
	cout<<scene_max.x<<" "<<scene_max.y<<" "<<scene_max.z<<endl;

	float dx = scene_max.x - scene_min.x;
	float dy = scene_max.y - scene_min.y;
	float dz = scene_max.z - scene_min.z;
	float d = dx > dy ? dx : dy;
	d = d > dz ? d : dz;
	
	//scale = d/dlen*0.05;
	scale = d/dlen*0.03;

	cout<<"target pos:"<<m_pt<<endl;
	cout<<"target pos1:"<<m_pt1<<endl;
	cout<<"target plane:"<<matPlane<<endl;
	cout<<"target direction:"<<m_direction<<endl;
	cout<<"world 2 pixel scale:"<<w2ps<<endl;
	cout<<"scale d:"<<d<<endl;
	cout<<"normalize scale:"<<ns<<endl;
	cout<<"diaglen:"<<dlen<<endl;
	cerr<<"scale:"<<scale<<endl;

	m_scale = scale;
	m_model.setScale(scale,scale,scale);
	//m_model.setScale(w2ps,w2ps,w2ps);
	//m_model.setScale(0.001f,0.001f,0.001f);
	//m_model.setRotation(0,180,0,1,0);


	//debug,save image
	line(im, marker.mInPt[0], marker.mInPt[1], Scalar(255,  0, 0), 1 );
	line(im, marker.mInPt[1], marker.mInPt[2], Scalar(0,  255, 0), 1 );
	line(im, marker.mInPt[2], marker.mInPt[3], Scalar(0,  0, 255), 1 );
	line(im, marker.mInPt[3], marker.mInPt[0], Scalar(255,255, 0), 1 );
	circle(im,marker.mInPt[4], 2, Scalar(0,0,255),2.5);

	//计算slam与实际世界的尺度 w2ps dlen 185mm
	mWorldScale = w2ps*dlen/MARKERLEN;
	cerr<<" world scale:"<<mWorldScale<<endl;

	//void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0);
	for(int i=0;i<marker.mvKeyPt.size();i++){
	  Point2f pt = marker.mvKeyPt[i];
	  circle(im, pt, 2,Scalar(255,0,0),2);
	}

	for(int i=0;i<marker.mvKeyOutPt.size();i++){
	  Point2f pt = marker.mvKeyOutPt[i];
	  circle(im, pt, 2,Scalar(0,255,0),2);
	}
	  
	imwrite("tmp.jpg",im);
	
	m_bDetected = true;
	return;
      }
    }

    //未检测到目标,或者已经检测到,处于跟踪状态
    m_camPose = m_pSLAM->TrackMonocular(im,0);
    m_model.update();
    m_model2.update();

    //Loop count
    m_LoopCount = m_pSLAM->getLoopDetectCount();

    //KeyFrame count
    //MapPoint count    
    m_pSLAM->getTrackResult(m_TrackResult);

    if(!m_camPose.empty()){
      m_state = TRACKING;
      mTrajectory.push_back(m_camPose);
      
      //更新运动模型的位置
      if(m_bAdd && m_bAutoMove) {
	if(m_nMoveIndex < m_vTrajectory.size()) {
          //cout<<m_nMoveIndex<<":"<<m_vTrajectory[m_nMoveIndex]<<endl;
	  float r = m_vTrajectory[m_nMoveIndex].at<float>(4);
	  m_model.setRotation(1,r,0,1,0);
	  m_pt = m_vTrajectory[m_nMoveIndex++].rowRange(0,4);
	}
      }

    }else{
      if(m_state == TRACKING)
	m_state = LOST;
    }
  }
}


//--------------------------------------------------------------
void ofApp::draw(){

  //ofSetColor(255);
  float view_width = ofGetViewportWidth();
  float view_height = ofGetViewportHeight();
  m_video.draw(0, 0, view_width, view_height);

  //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //run state
  string strState = "initializing";
  string strMarker = "not detected";
  if(TRACKING == m_state)
    strState = "tracking";
  else if(LOST == m_state)
    strState = "lost";
  if(m_bDetected)
    strMarker = "detected";


  ofSetColor(0, 255, 0);
  ofDrawBitmapString("fps: "+ofToString(ofGetFrameRate(), 2), 10, 15);
  ofDrawBitmapString("state: "+strState, 10, 30);
  ofDrawBitmapString("marker: "+strMarker, 10, 45);
  ofDrawBitmapString("loopCount: "+ofToString(m_LoopCount,2), 10, 60);
  ofDrawBitmapString("kfs: "+ofToString(m_TrackResult.nKFs,2), 10, 75);
  ofDrawBitmapString("mps: "+ofToString(m_TrackResult.nMPs,2), 10, 90);
  ofDrawBitmapString("matches: "+ofToString(m_TrackResult.nTracked,2), 10, 105);
  ofDrawBitmapString("matches VO: "+ofToString(m_TrackResult.nTrackedVO,2), 10, 120);
  ofSetColor(255, 255, 255 );

  //1.Set camera matrix to the opengl projection matrix;
  intrinsicMatrix2ProjectionMatrix(m_camera_matrix, 640, 480, 0.01f, 100.0f, m_projection_matrix);
  //cout<<"project matrix:"<<m_projection_matrix<<endl;

  //2.set matrix mode
  ofSetMatrixMode(OF_MATRIX_PROJECTION);

  //3.这一段去掉就不显示坐标系了
  static float reflect[] = 
    {
      1,  0,  0, 0,
      0,  -1,  0, 0,
      0,  0,  1, 0,
      0,  0,  0, 1
    };
  ofLoadMatrix(reflect);

  //左右手坐标系问题,需要绕x轴旋转
  //ofRotateX(180.0);
  
  //OpenGL默认为右乘
  ofMultMatrix(m_projection_matrix);

  //Reset model view matrix to identity;
  ofSetMatrixMode(OF_MATRIX_MODELVIEW);
  ofLoadIdentityMatrix();

  //没有这一段代码，正方体颜色为纯白
  //Set opengl parameters
  ofEnableBlendMode(OF_BLENDMODE_ALPHA);
  ofEnableDepthTest();
  glShadeModel(GL_SMOOTH); //some model / light stuff
  //m_light.enable();
  ofEnableSeparateSpecularLight();


  //draw
  if( !m_camPose.empty()) {
    float mm[16];
    Mat Pose,R,T;
    m_camPose.copyTo(Pose);
    R = Pose.rowRange(0,3).colRange(0,3);
    T = Pose.rowRange(0,3).col(3);
    cout << R<<"  "<<T<<endl;
    extrinsicMatrix2ModelViewMatrix(R,T,mm);
    cout<<"his matrix:"<<Mat(4,4,CV_32FC1,mm)<<endl;
    ofLoadMatrix(mm);

    //画坐标系:世界坐标系
    ofGetCurrentRenderer()->setLineWidth(3.0f);
    if(m_bDebug)
      ofGetCurrentRenderer()->get3dGraphics().drawAxis(0.8f);
    
    float o[] = {0,0,0,1};
    Mat pt = Mat(4,1,CV_32F,o);

    //orbslam的世界坐标系原点
    cout<<"cam pose:"<<m_camPose<<endl;
    float x0 = pt.at<float>(0,0);
    float y0 = pt.at<float>(0,1);
    float z0 = pt.at<float>(0,2);

    ofSetColor(255,255,0);
    //ofDrawCircle(x0, y0, z0, 0.01f);


    //目标位置
    if(m_bDetected) {
      //中心点
      ofSetColor(0,255,255);
      m_pt.copyTo(pt);
      cout<<"target center point:"<<pt<<endl;
      float xC = pt.at<float>(0);
      float yC = pt.at<float>(1);
      float zC = pt.at<float>(2);
      //draw model1    
      ofSetColor(255,255,255,255);
      m_model.setPosition(xC,yC,zC);
      if(m_bShow1)
	m_model.drawFaces();

      if(m_bDebug) {
	cout<<"world point:"<<m_pt<<endl;
	ofDrawCircle(xC, yC, zC, 0.01f);

	//画法向量pp
	//pt = m_direction;
	m_movePlane.copyTo(pt);
	cout<<"target plane direction:"<<pt<<endl;
	float xD = pt.at<float>(0);
	float yD = pt.at<float>(1);
	float zD = pt.at<float>(2);
	ofSetColor(0,200,0);
	ofDrawLine(xC,yC,zC,xC+xD,yC+yD,zC+zD);

	//画目标平面，y轴上的点为中心
	m_pt.copyTo(pt);
	pt.at<float>(0) = 0;
	pt.at<float>(2) = 0;
	myDrawPlane(pt,m_movePlane,300,300,0.5);

	//model1的旋转
	ofPoint ofpt = m_model.getRotationAxis(0);
	cout<<" model rotation:"<<ofpt.x<<","<<ofpt.y<<","<<ofpt.z<<endl;
      }
    }

    if(m_bAdd) {
      //draw model2
      ofSetColor(255,255,255);
      float x = m_pt2.at<float>(0);
      float y = m_pt2.at<float>(1);
      float z = m_pt2.at<float>(2);
      cout<<"mpt22:\n"<<m_pt2<<endl;
      m_model2.setPosition(x,y,z);
      if(m_bShow2)
	m_model2.drawFaces();

      //draw automove trajectory
      if(m_bDebug) {
	//画预设的移动轨迹
	ofSetColor(128,0,0);
	float x,y,z,x0,y0,z0;
	Mat lastPt;
	m_pt1.copyTo(lastPt);
	for(int i = 0;i<m_vPathPts.size(); i++) {
	  x = m_vPathPts[i].at<float>(0);
	  y = m_vPathPts[i].at<float>(1);
	  z = m_vPathPts[i].at<float>(2);
	  x0 = lastPt.at<float>(0);
	  y0 = lastPt.at<float>(1);
	  z0 = lastPt.at<float>(2);
	  //draw
	  ofDrawLine(x0,y0,z0,x,y,z);
	  //lastPt = m_vPathPts[i];
	  m_vPathPts[i].copyTo(lastPt);
	}

	//画过m_pt2的平面法向量
	m_pt2.copyTo(pt);
	float a = pt.at<float>(0);
	float b = pt.at<float>(1);
	float c = pt.at<float>(2);
	x = m_movePlane.at<float>(0) + a;
	y = m_movePlane.at<float>(1) + b;
	z = m_movePlane.at<float>(2) + c;
	ofSetColor(0,0,128);
	ofDrawLine(x,y,z,a,b,c);

	//model2的旋转
	ofPoint ofpt = m_model2.getRotationAxis(0);
	cout<<" model2 rotation:"<<ofpt.x<<","<<ofpt.y<<","<<ofpt.z<<endl;
	cout<<" model2 angle0:"<<m_model2.getRotationAngle(0)<<endl;
	cout<<" model2 angle1:"<<m_model2.getRotationAngle(1)<<endl;
	cout<<" model2 angle2:"<<m_model2.getRotationAngle(2)<<endl;
      }
    }

    //marker point
    if(m_bDebug) {
      for(int i=0;i<m_vMarkerPt.size();i++) {
	int c = i+99;
	ofSetColor(c*9%255,c*99%255,c*999%255);

	float x = m_vMarkerPt[i].at<float>(0);
	float y = m_vMarkerPt[i].at<float>(1);
	float z = m_vMarkerPt[i].at<float>(2);
	ofDrawBox(x, y, z, 0.05f);
	ofSetColor(255,255,255);
      }
    }

    ofSetColor(255,255,255);
  }


  m_bMove = false;

  //Reset parameters
  ofDisableDepthTest();
  //m_light.disable();
  ofDisableLighting();
  ofDisableSeparateSpecularLight();

  ofSetMatrixMode(OF_MATRIX_MODELVIEW);
  ofLoadIdentityMatrix();
  ofSetMatrixMode(OF_MATRIX_PROJECTION);
  ofLoadIdentityMatrix();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
  cerr<<"key:"<<key<<endl;
  if(m_camPose.empty())
    return;

  if (key == 'o') {
    string name = ofGetTimestampString();
    imwrite(name + ".jpg", m_img_gray);
    cout << "Frame " << name <<" has been saved!"<<endl;
  } else if (key == 'q') {
    cout << "quit..."<<endl;
    //m_pSlam->Shutdown();
		
  } else if(key == 'i') {
    cerr<<"key 'i' pressed:add model2.\n";
    Mat Rwc(3,3,CV_32F);
    Mat twc(3,1,CV_32F);
    Rwc = m_camPose.rowRange(0,3).colRange(0,3).t();
    twc = Rwc*m_camPose.rowRange(0,3).col(3);
    m_pt2 = -twc;
    cerr<<"--pt2:\n"<<m_pt2<<endl;
    m_bAdd = true;
  }else if(key == 'a') {
    cerr<<"key 'a' pressed:move mode1 to the plane.\n";
    cerr<<"--old pt1:\n"<<m_pt<<endl;
    Mat Rwc(3,3,CV_32F);
    Mat twc(3,1,CV_32F);
    //R正交，R.t()=R.inv()
    //RxT,在世界坐标系下，先平移再旋转
    Rwc = m_camPose.rowRange(0,3).colRange(0,3).t();
    twc = Rwc*m_camPose.rowRange(0,3).col(3);
    Mat pt = -twc;
    computeIntersection(pt,m_pt,m_movePlane);
    m_pt.copyTo(m_pt1);
    cerr<<"--new pt1:\n"<<m_pt<<endl;
  } else if(key == 'b') {
    cerr<<"key 'b' pressed:add model2 in the plane.\n";
    Mat Rwc(3,3,CV_32F);
    Mat twc(3,1,CV_32F);
    Rwc = m_camPose.rowRange(0,3).colRange(0,3).t();
    twc = Rwc*m_camPose.rowRange(0,3).col(3);
    m_pt2 = -twc;
    //computeIntersection(pt,pt2,m_movePlane);
    //pt2.copyTo(m_pt2);
    Mat pt;
    computeIntersection(m_pt2,pt,m_movePlane);
    cerr<<"--pt:\n"<<m_pt2<<endl;
    cerr<<"--pt2:\n"<<pt<<endl;
    cerr<<"--plane:\n"<<m_movePlane<<endl;
    pt.copyTo(m_pt2);
    m_model2.setScale(m_scale,m_scale,m_scale);
    m_bAdd = true;
  } else if(key == 'e') {
    cerr<<"key 'e' pressed:set path.\n";
    Mat Rwc(3,3,CV_32F);
    Mat twc(3,1,CV_32F);
    Rwc = m_camPose.rowRange(0,3).colRange(0,3).t();
    twc = Rwc*m_camPose.rowRange(0,3).col(3);
    Mat pt = -twc;
    Mat pt2;
    cerr<<"--cuurent pt:\n"<<pt<<endl;
    computeIntersection(pt,pt2,m_movePlane);
    cerr<<"--project pt:\n"<<pt2<<endl;
    m_vPathPts.push_back(pt2.clone());
  }else if(key == 'r') {
    cerr<<"key 'r' pressed:set path.\n";
    Mat Rwc(3,3,CV_32F);
    Mat twc(3,1,CV_32F);
    Rwc = m_camPose.rowRange(0,3).colRange(0,3).t();
    twc = Rwc*m_camPose.rowRange(0,3).col(3);
    Mat pt = -twc;
    m_vPathPts.push_back(pt.clone());
  }else if(key == 'c') {
    cerr<<"key 'c' pressed:move model1 to the starting position.\n";
    cerr<<"camera pose:\n"<<m_camPose<<endl;
    m_pt1.copyTo(m_pt);
    m_model.setRotation(1,0,0,1,0);
    //m_pt = m_pt1.clone();
  }else if(key == 's') {
    cerr<<"key 's' pressed:set marker.\n";
    Mat Rwc(3,3,CV_32F);
    Mat twc(3,1,CV_32F);
    Rwc = m_camPose.rowRange(0,3).colRange(0,3).t();
    twc = Rwc*m_camPose.rowRange(0,3).col(3);
    Mat pt = -twc;
    m_vMarkerPt.push_back(pt);
  }
  //plane:ax+by+cz+d=0
  //UP:z+ DOWN:z- LEFT:x- RIGHT:x+
  else if(key == UP) {
    cerr<<"move up:"<<endl;
    cerr<<"  from:"<<m_pt<<endl;
    float pt[4] = {0};
    pt[0] = m_pt.at<float>(0);
    pt[1] = m_pt.at<float>(1);
    pt[2] = m_pt.at<float>(2);
    pt[3] = 1;

    float plane[4] = {0};
    plane[0] = m_movePlane.at<float>(0);
    plane[1] = m_movePlane.at<float>(1);
    plane[2] = m_movePlane.at<float>(2);
    plane[3] = m_movePlane.at<float>(3);

    float y,x=0.0000001,z = 0.01;
    if (fabs(plane[1]) <= x)
      return;

    pt[0] += x;
    pt[2] += z;
    pt[3] = 1;
    pt[1] = -(plane[0]*pt[0] + plane[2]*pt[2] + plane[3])/plane[1];

    Mat newpt(4,1,CV_32FC1,pt);
    newpt.copyTo(m_pt);
    cerr<<"  to:"<<m_pt<<endl;
  } else if(key == DOWN) {
    cerr<<"move down:"<<endl;
    cerr<<"  from:"<<m_pt<<endl;
    float pt[4] = {0};
    pt[0] = m_pt.at<float>(0);
    pt[1] = m_pt.at<float>(1);
    pt[2] = m_pt.at<float>(2);
    pt[3] = 1;

    float plane[4] = {0};
    plane[0] = m_movePlane.at<float>(0);
    plane[1] = m_movePlane.at<float>(1);
    plane[2] = m_movePlane.at<float>(2);
    plane[3] = m_movePlane.at<float>(3);

    float y,x=0.0000001,z = -0.01;
    if (fabs(plane[1]) <= x)
      return;

    pt[0] += x;
    pt[2] += z;
    pt[3] = 1;
    pt[1] = -(plane[0]*pt[0] + plane[2]*pt[2] + plane[3])/plane[1];

    Mat newpt(4,1,CV_32FC1,pt);
    newpt.copyTo(m_pt);
    cerr<<"  to:"<<m_pt<<endl;
  } else if(key == LEFT) {
    cerr<<"move left:"<<endl;
    cerr<<"  from:"<<m_pt<<endl;
    float pt[4] = {0};
    pt[0] = m_pt.at<float>(0);
    pt[1] = m_pt.at<float>(1);
    pt[2] = m_pt.at<float>(2);
    pt[3] = 1;

    float plane[4] = {0};
    plane[0] = m_movePlane.at<float>(0);
    plane[1] = m_movePlane.at<float>(1);
    plane[2] = m_movePlane.at<float>(2);
    plane[3] = m_movePlane.at<float>(3);

    float z=0.0000001,x=-0.01;
    if (fabs(plane[1]) <= z)
      return;

    pt[0] += x;
    pt[2] += z;
    pt[3] = 1;
    pt[1] = -(plane[0]*pt[0] + plane[2]*pt[2] + plane[3])/plane[1];

    Mat newpt(4,1,CV_32FC1,pt);
    newpt.copyTo(m_pt);
    cerr<<"  to:"<<m_pt<<endl;
  } else if(key == RIGHT) {
    cerr<<"move right:"<<endl;
    cerr<<"  from:"<<m_pt<<endl;
    float pt[4] = {0};
    pt[0] = m_pt.at<float>(0);
    pt[1] = m_pt.at<float>(1);
    pt[2] = m_pt.at<float>(2);
    pt[3] = 1;

    float plane[4] = {0};
    plane[0] = m_movePlane.at<float>(0);
    plane[1] = m_movePlane.at<float>(1);
    plane[2] = m_movePlane.at<float>(2);
    plane[3] = m_movePlane.at<float>(3);

    float y,z=0.0000001,x=0.01;
    if (fabs(plane[1]) <= z)
      return;

    pt[0] += x;
    pt[2] += z;
    pt[3] = 1;
    pt[1] = -(plane[0]*pt[0] + plane[2]*pt[2] + plane[3])/plane[1];

    Mat newpt(4,1,CV_32FC1,pt);
    newpt.copyTo(m_pt);
    cerr<<"  to:"<<m_pt<<endl;
  } else if(key == 'm') {
    //最后要移动到m_pt2
    m_vPathPts.push_back(m_pt2); 
    //清空上次运行的轨迹
    m_vTrajectory.clear();  
    m_nMoveIndex = 0;

    //一次移动2mm
    float d = mWorldScale*STEP;
    Mat lastPt = m_pt;

    for(int i = 0;i<m_vPathPts.size(); i++) {
      float x,y,z,x0,y0,z0,t;
      x = m_vPathPts[i].at<float>(0);
      y = m_vPathPts[i].at<float>(1);
      z = m_vPathPts[i].at<float>(2);
      x0 = lastPt.at<float>(0);
      y0 = lastPt.at<float>(1);
      z0 = lastPt.at<float>(2);

      float d2 = (x-x0)*(x-x0) + (y-y0)*(y-y0) + (z-z0)*(z-z0);
      d2 = (float)pow(d2,0.5);
      int step = d2/d;

      //移动分量
      float dx = (x-x0)/(float)step;
      float dy = (y-y0)/(float)step;
      float dz = (z-z0)/(float)step;
      //绕y轴旋转分量
      float r = 0;
      Point3f pt1(x0,y0,z0);
      Point3f pt2(x,y,z);
      computeMoveAngle(pt1,pt2,r);
      cout<<"move angle:"<<r<<endl;
      
      for(int j=0;j<step;j++) {
	float p[4];
	p[0] = x0 + dx*(float)j;
	p[1] = y0 + dy*(float)j;
	p[2] = z0 + dz*(float)j;
	p[3] = 1;
	p[4] = r;
	Mat pt(5,1,CV_32FC1,p);
	m_vTrajectory.push_back(pt.clone());//此处必须用clone
	cout<<"t1   "<<pt<<endl;
      }
      lastPt = m_vPathPts[i];
    }

    //将mpt2从路径中删除
    m_vPathPts.pop_back();

    m_bAutoMove = true;
    cerr<<"move trajectory complete.\n";
  } else if(key == 'p') {
    float A = m_movePlane.at<float>(0);
    float B = m_movePlane.at<float>(1);
    float C = m_movePlane.at<float>(2);
    float D = m_movePlane.at<float>(3);
    //Ax+By+Cz+D=0
    //Ax+B(y-y0)+C+D=0
    float y0 = 0.5;
    float s = D+B*y0;
    float o[4] = {A/s,B/s,C/s,1};
    Mat plane(4,1,CV_32FC1,o);
    plane.copyTo(m_movePlane);
  } else if(key == 'n') {
    float A = m_movePlane.at<float>(0);
    float B = m_movePlane.at<float>(1);
    float C = m_movePlane.at<float>(2);
    float D = m_movePlane.at<float>(3);
    //Ax+By+Cz+D=0
    //Ax+B(y-y0)+C+D=0
    //float y0 = -0.5;
    float y0 = -mWorldScale*700;
    float s = D+B*y0;
    float o[4] = {A/s,B/s,C/s,1};
    Mat plane(4,1,CV_32FC1,o);
    plane.copyTo(m_movePlane);
  } else if(key == 'd') {
    m_bDebug = !m_bDebug;
  } else if(key == '+') {
    float scale = 1.2;
    ofPoint p = m_model.getScale();
    m_model.setScale(p.x*scale,p.y*scale,p.z*scale);
    
    p = m_model2.getScale();
    m_model2.setScale(p.x*scale,p.y*scale,p.z*scale);
  } else if(key == '-') {
    float scale = 0.8;
    ofPoint p = m_model.getScale();
    m_model.setScale(p.x*scale,p.y*scale,p.z*scale);
    
    p = m_model2.getScale();
    m_model2.setScale(p.x*scale,p.y*scale,p.z*scale);
  } else if(key == '1') {
    m_bShow1 = !m_bShow1;
  } else if(key == '2') {
    m_bShow2 = !m_bShow2;
  } else if(key == PAGEDOWN ) {
    //m_pt.at<float>(1) -= 0.01;
    m_pt2.at<float>(1) += 0.01;
    //
    for(int i=0;i<m_vPathPts.size();i++) {
      m_vPathPts[i].at<float>(2) -= 0.01;
    } 
  }else if(key == PAGEUP ) {
    //m_pt.at<float>(1) -= 0.01;
    m_pt2.at<float>(1) -= 0.01;
    //
    for(int i=0;i<m_vPathPts.size();i++) {
      m_vPathPts[i].at<float>(2) -= 0.01;
    }
  }

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

/*
  void ofApp::setSlam(ORB_SLAM2::System* pSLAM)
  {
  m_pSlam = pSLAM;
  }

  ORB_SLAM2::System ofApp::getSlam()
  {
  return *m_pSlam;
  }
*/


//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::intrinsicMatrix2ProjectionMatrix(cv::Mat& camera_matrix, float width, float height, float near_plane, float far_plane, float* projection_matrix)
{
  float f_x = camera_matrix.at<float>(0,0);
  float f_y = camera_matrix.at<float>(1,1);

  float c_x = camera_matrix.at<float>(0,2);
  float c_y = camera_matrix.at<float>(1,2);
	
  projection_matrix[0] = 2*f_x/width;
  projection_matrix[1] = 0.0f;
  projection_matrix[2] = 0.0f;
  projection_matrix[3] = 0.0f;

  projection_matrix[4] = 0.0f;
  projection_matrix[5] = 2*f_y/height;
  projection_matrix[6] = 0.0f;
  projection_matrix[7] = 0.0f;

  projection_matrix[8] = 1.0f - 2*c_x/width;
  projection_matrix[9] = 2*c_y/height - 1.0f;
  projection_matrix[10] = -(far_plane + near_plane)/(far_plane - near_plane);
  projection_matrix[11] = -1.0f;

  projection_matrix[12] = 0.0f;
  projection_matrix[13] = 0.0f;
  projection_matrix[14] = -2.0f*far_plane*near_plane/(far_plane - near_plane);
  projection_matrix[15] = 0.0f;
}

void ofApp::extrinsicMatrix2ModelViewMatrix(cv::Mat& rotation, cv::Mat& translation, float* model_view_matrix)
{
  //绕X轴旋转180度，从OpenCV坐标系变换为OpenGL坐标系
  static float d[] = 
    {
      1, 0, 0,
      0, -1, 0,
      0, 0, -1
    };
  Mat_<float> rx(3, 3, d);

  rotation = rx*rotation;
  translation = rx*translation;

  model_view_matrix[0] = rotation.at<float>(0,0);
  model_view_matrix[1] = rotation.at<float>(1,0);
  model_view_matrix[2] = rotation.at<float>(2,0);
  model_view_matrix[3] = 0.0f;

  model_view_matrix[4] = rotation.at<float>(0,1);
  model_view_matrix[5] = rotation.at<float>(1,1);
  model_view_matrix[6] = rotation.at<float>(2,1);
  model_view_matrix[7] = 0.0f;

  model_view_matrix[8] = rotation.at<float>(0,2);
  model_view_matrix[9] = rotation.at<float>(1,2);
  model_view_matrix[10] = rotation.at<float>(2,2);
  model_view_matrix[11] = 0.0f;

  model_view_matrix[12] = translation.at<float>(0, 0);
  model_view_matrix[13] = translation.at<float>(1, 0);
  model_view_matrix[14] = translation.at<float>(2, 0);
  model_view_matrix[15] = 1.0f;
}




void ofApp::myDrawPlane(Mat centerPt,Mat plane,int rows,int cols,float scale)
{
  ofGetCurrentRenderer()->setLineWidth(2.5f);
  ofSetColor(0,100,0);

  for(int i=0;i<rows;i++) {
    Mat lPt,rPt,inPt;

    //left
    centerPt.copyTo(inPt);
    inPt.at<float>(0) -= scale*cols;
    inPt.at<float>(2) += scale*i;
    computeIntersection(inPt,lPt,plane);

    //right
    centerPt.copyTo(inPt);
    inPt.at<float>(0) += scale*cols;
    inPt.at<float>(2) += scale*i;
    computeIntersection(inPt,rPt,plane);
    
    float x0 = lPt.at<float>(0);
    float y0 = lPt.at<float>(1);
    float z0 = lPt.at<float>(2);
    float x1 = rPt.at<float>(0);
    float y1 = rPt.at<float>(1);
    float z1 = rPt.at<float>(2);
    
    ofDrawLine(x0,y0,z0,x1,y1,z1);

    centerPt.copyTo(inPt);
    inPt.at<float>(0) -= scale*cols;
    inPt.at<float>(2) -= scale*i;
    computeIntersection(inPt,lPt,plane);

    centerPt.copyTo(inPt);
    inPt.at<float>(0) += scale*cols;
    inPt.at<float>(2) -= scale*i;
    computeIntersection(inPt,rPt,plane);

    x0 = lPt.at<float>(0);
    y0 = lPt.at<float>(1);
    z0 = lPt.at<float>(2);
    x1 = rPt.at<float>(0);
    y1 = rPt.at<float>(1);
    z1 = rPt.at<float>(2);
    
    ofDrawLine(x0,y0,z0,x1,y1,z1);
  }

  for(int i=0;i<cols;i++) {
    Mat tPt,bPt,inPt;

    //top
    centerPt.copyTo(inPt);
    inPt.at<float>(0) -= scale*i;
    inPt.at<float>(2) += scale*rows;
    computeIntersection(inPt,tPt,plane);

    //bottom
    centerPt.copyTo(inPt);
    inPt.at<float>(0) -= scale*i;
    inPt.at<float>(2) -= scale*rows;
    computeIntersection(inPt,bPt,plane);
    
    float x0 = tPt.at<float>(0);
    float y0 = tPt.at<float>(1);
    float z0 = tPt.at<float>(2);
    float x1 = bPt.at<float>(0);
    float y1 = bPt.at<float>(1);
    float z1 = bPt.at<float>(2);
    
    ofDrawLine(x0,y0,z0,x1,y1,z1);

    //top
    centerPt.copyTo(inPt);
    inPt.at<float>(0) += scale*i;
    inPt.at<float>(2) += scale*rows;
    computeIntersection(inPt,tPt,plane);

    //bottom
    centerPt.copyTo(inPt);
    inPt.at<float>(0) += scale*i;
    inPt.at<float>(2) -= scale*rows;
    computeIntersection(inPt,bPt,plane);
    
    x0 = tPt.at<float>(0);
    y0 = tPt.at<float>(1);
    z0 = tPt.at<float>(2);
    x1 = bPt.at<float>(0);
    y1 = bPt.at<float>(1);
    z1 = bPt.at<float>(2);
    
    ofDrawLine(x0,y0,z0,x1,y1,z1);
  }
}


 void ofApp::enableDebug()
 {
   m_bDebug = true;
 }
