#ifndef __SamplepointDetection_h__
#define __SamplepointDetection_h__

#include "ObstacleDetection.h"

#include <vector>
#include <string>
#include "Samplepoint.h"
#include "opencv2/core.hpp"

class SamplepointDetection: public ObstacleDetection
{
  public:
    // constructor / destructor
    SamplepointDetection();
    ~SamplepointDetection();

    void init(cv::Mat const&);

    /* virtual */ void build(cv::Mat const&, int, int);
    /* virtual */ void detectObstacles();
    /* virtual */ void print_on(std::ostream&) const;

  private:
    std::string              mTag;
    std::vector<Samplepoint> mSPVector;
    cv::Mat                  mSPMatrix;

};

#endif