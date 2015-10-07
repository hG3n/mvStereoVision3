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
    // constructor / destructor
    SamplepointDetection();
    ~SamplepointDetection();

    void init(cv::Mat const&, cv::Mat const&);

    std::vector<Samplepoint> getSamplepointVec() const;
    cv::Mat_<float>          getCenterPoint() const;

    /* virtual */ void print_on(std::ostream&) const;
    /* virtual */ void build(cv::Mat const&, int, int);
    /* virtual */ void detectObstacles();

  private:
    std::string              mTag;
    std::vector<Samplepoint> mSPVec;
    std::vector<float>       mDistanceVec;
    cv::Mat_<float>          mCenterVec;
    cv::Point                mImageCenter;
    cv::Mat                  mQ_32F;
};

#endif