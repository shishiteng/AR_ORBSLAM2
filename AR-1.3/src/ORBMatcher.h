#ifndef __ORBMATCHER_H__
#define __ORBMATCHER_H__

#include <opencv2/core/core.hpp>

class ORBMatcher
{
public:
  ORBMatcher();
  
  int Match(cv::Mat img1,cv::Mat img2);
  void GetHomography();
  
//private:
  cv::Mat m_homo;
};


#endif
