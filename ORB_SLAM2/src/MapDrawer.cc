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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <GL/glut.h>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mbDrawImuAxis = false;
}

void MapDrawer::DrawCoordinateEx()
{
//glPushMatrix();
//glRotatef(180, 1, 0, 0);
  glClearColor(1.0, 1.0, 1.0, 0.0);
  glBegin(GL_LINES); 
  glLineWidth(10);

  GLfloat o[3] = { 0.0f, 0.0f, 0.0f };
  GLfloat x[3] = { 1.0f, 0.0f, 0.0f };
  GLfloat y[3] = { 0.0f, 1.0f, 0.0f };
  GLfloat z[3] = { 0.0f, 0.0f, 1.0f };



  glColor3f(255,0,0);
  glVertex3fv(&o[0]);     
  glVertex3fv(&x[0]);     
  glVertex3fv(&o[0]);     

  glColor3f(0,255,0);
  glVertex3fv(&o[0]);
  glVertex3fv(&y[0]);
  glVertex3fv(&o[0]);

  glColor3f(0,0,255);
  glVertex3fv(&o[0]);
  glVertex3fv(&z[0]);

  glEnd();

//glPopMatrix();
}

void MapDrawer::DrawCoordinate()
{
	glBegin(GL_LINES); 
	glColor3f(0.0,0.0,128.0);

	float vertex_list[][3] = 
	{ 
		-0.5f, -0.5f, -0.5f, 
		0.5f, -0.5f, -0.5f, 
		-0.5f, 0.5f, -0.5f, 
		0.5f, 0.5f, -0.5f, 
		-0.5f, -0.5f, 0.5f, 
		0.5f, -0.5f, 0.5f, 
		-0.5f, 0.5f, 0.5f, 
		0.5f, 0.5f, 0.5f, 
	}; 
	
	GLint index_list[][2] = 
	{ 
		{0, 1},    
		{2, 3},    
		{4, 5},    
		{6, 7},    
		{0, 2},    
		{1, 3},    
		{4, 6},    
		{5, 7},
		{0, 4},
		{1, 5},
		{7, 3},
		{2, 6}
	};

	//3 lines
    for(int i=0; i<12; ++i)
    {
        for(int j=0; j<2; ++j)
        {
            glVertex3fv(vertex_list[index_list[i][j]]);     
        }
    }


	glEnd();
}


void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}


void MapDrawer::DrawTargetCenter()
{
  glPointSize(mPointSize*2);
  glBegin(GL_POINTS);
  glColor3f(0.5,0.5,1.0);

  float x = mpMap->mTargetCenter[0];
  float y = mpMap->mTargetCenter[1];
  float z = mpMap->mTargetCenter[2];

  glVertex3f(x,y,z);

  glEnd();
}

void MapDrawer::DrawTargetPlane()
{
  glLineWidth(5);
  glBegin(GL_LINES);
  glColor3f(0.0,1.0,1.0);

  float a = mpMap->mTargetPlane[0];
  float b = mpMap->mTargetPlane[1];
  float c = mpMap->mTargetPlane[2];
  //float d = mpMap->mTargetPlane[3];

  glVertex3f(0,0,0);
  glVertex3f(a,b,c);

  glEnd();
}

void MapDrawer::DrawTargetPlane2()
{
  glLineWidth(5);
  glBegin(GL_LINES);
  glColor3f(1.0,0.0,0.0);

  float a = mpMap->mTargetPlane[0];
  float b = mpMap->mTargetPlane[1];
  float c = mpMap->mTargetPlane[2];
  //float d = mpMap->mTargetPlane[3];

  float x = mpMap->mTargetCenter[0];
  float y = mpMap->mTargetCenter[1];
  float z = mpMap->mTargetCenter[2];

  glVertex3f(x,y,z);
  //glVertex3f(a+x,b+y,-(c+z));
  glVertex3f(a-x,b-y,-(c-z));

  glEnd();
}


void MapDrawer::DrawTargetDirection()
{
  glLineWidth(5);
  glBegin(GL_LINES);
  glColor3f(0.0,1.0,0.0);

  //float a = mpMap->mTargetPlane[0];
  float b = mpMap->mTargetPlane[1];
  float c = mpMap->mTargetPlane[2];
  float d = mpMap->mTargetPlane[3];

  float x = mpMap->mTargetCenter[0];
  float y = mpMap->mTargetCenter[1];
  float z = mpMap->mTargetCenter[2];

  float vectorA[] = {x,y,z+d/c};
  float vectorB[] = {x,y+d/b,z};
  float vectorC[] = {x+d/b,y,z};

  cv::Mat A(1,3,CV_32FC1,vectorA);
  cv::Mat B(1,3,CV_32FC1,vectorB);
  cv::Mat C(1,3,CV_32FC1,vectorC);

  cv::Mat D = A.cross(B);
  float xx = D.at<float>(0,1);
  float yy = D.at<float>(0,2);
  float zz = D.at<float>(0,3);

  //glVertex3f(0,0,-d/c);
  //glVertex3f(0,-d/b,0);
  //glVertex3f(-d/a,0,0);
  //glVertex3f(0,0,-d/c);

  glVertex3f(x,y,z);
  glVertex3f(xx,yy,zz);

  glEnd();
}


void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::AddMarkerCamera(pangolin::OpenGlMatrix Twc)
{
  mvMarkerCamera.push_back(Twc);
}

void MapDrawer::DrawMarkerCamera()
{
  glPointSize(mPointSize*15);
  glColor3f(1.0f,0.0f,0.0f);

  for(size_t i=0, iend=mvMarkerCamera.size(); i<iend;i++)
  {
    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(mvMarkerCamera[i].m);
#else
        glMultMatrixd(mvMarkerCamera[i].m);
#endif

    glBegin(GL_POINTS);
    if(0 == i)
      glColor3f(1.0f,0.3f,0.0f);
    else if(1 == i)
      glColor3f(0.0f,0.3f,1.0f);
    else if(2 == i)
      glColor3f(1.0f,0.6f,0.0f);
    else if(3 == i)
      glColor3f(0.0f,0.6f,1.0f);
    else
      glColor3f(1.0f,0.0f,0.0f);

    glVertex3f(0,0,0);
    glEnd();

    glPopMatrix();
  }

}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
