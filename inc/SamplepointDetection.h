#ifndef __SamplepointDetection_h__
#define __SamplepointDetection_h__

#include <vector>
#include <string>

#include "ObstacleDetection.h"
#include "Samplepoint.h"

//OPENCV Stuff
#include "opencv2/core.hpp"



class SamplepointDetection: public ObstacleDetection
{
  public:

    struct sort_distances {
      bool operator ()(std::pair<int,float> const& lhs, std::pair<int,float> const& rhs) {
        return lhs.second < rhs.second;
      }
    };

  // constructor / destructor
  SamplepointDetection();
  ~SamplepointDetection();

  void init(cv::Mat const&, cv::Mat const&, float, float);

  // getter
  std::vector<Samplepoint>  getSamplepointVec() const;
  cv::Mat_<float>           getCenterPoint() const;
  std::pair<int,int>        getRange() const;
  std::vector<Samplepoint>  getFoundObstacles() const;
  cv::Mat_<float>           getPointcloud() const;

  // setter
  void setRange(float, float); 

  /* virtual */ void print_on(std::ostream&) const;
  /* virtual */ void build(cv::Mat const&, int, int);
  /* virtual */ void detectObstacles();

  private:
    std::string              mTag;
    cv::Mat                  mDMap;
    std::vector<Samplepoint> mSPVec;
    std::vector<float>       mDistanceVec;
    cv::Mat_<float>          mCenterVec;
    cv::Point                mImageCenter;
    cv::Mat                  mQ_32F;
    std::vector<Samplepoint> mFoundObstacles;
    int                      mObstacleCounter;
    std::vector<cv::Mat>     mFoundPoints;
};

#endif