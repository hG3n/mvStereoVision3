#include "view.h"

void View::drawObstacleGrid(cv::Mat &stream, int binning)
{
  if(binning == 0) 
  {
    //vertical lines
    cv::line(stream, cv::Point(250,0), cv::Point(250,stream.rows), cv::Scalar(0,0,255), 1);
    cv::line(stream, cv::Point(502,0), cv::Point(502,stream.rows), cv::Scalar(0,0,255), 1);
    //horizontal lines
    cv::line(stream, cv::Point(0,160), cv::Point(stream.cols, 160), cv::Scalar(0,0,255), 1);
    cv::line(stream, cv::Point(0,320), cv::Point(stream.cols, 320), cv::Scalar(0,0,255), 1);
  }
  else
  {
    //vertical lines
    cv::line(stream, cv::Point(250/2,0), cv::Point(250/2,stream.rows), cv::Scalar(0,0,255), 1);
    cv::line(stream, cv::Point(502/2,0), cv::Point(502/2,stream.rows), cv::Scalar(0,0,255), 1);
    //horizontal lines
    cv::line(stream, cv::Point(0,160/2), cv::Point(stream.cols, 160/2), cv::Scalar(0,0,255), 1);
    cv::line(stream, cv::Point(0,320/2), cv::Point(stream.cols, 320/2), cv::Scalar(0,0,255), 1);
  }
}

void View::drawSubimageGrid(cv::Mat &stream, int binning)
{
  if (binning == 0)
  {
    // vertical lines
    cv::line(stream, cv::Point(83,0), cv::Point(83,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(166,0), cv::Point(166,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(333,0), cv::Point(333,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(416,0), cv::Point(416,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(583,0), cv::Point(583,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(666,0), cv::Point(666,stream.rows), cv::Scalar(100,0,0), 1);
    // horizontal lines
    cv::line(stream, cv::Point(0,53), cv::Point(stream.cols,53), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,106), cv::Point(stream.cols,106), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,213), cv::Point(stream.cols,213), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,266), cv::Point(stream.cols,266), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,373), cv::Point(stream.cols,373), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,426), cv::Point(stream.cols,426), cv::Scalar(100,0,0), 1);
  }
  else
  {
    // vertical lines
    cv::line(stream, cv::Point(83/2,0), cv::Point(83/2,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(166/2,0), cv::Point(166/2,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(333/2,0), cv::Point(333/2,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(416/2,0), cv::Point(416/2,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(583/2,0), cv::Point(583/2,stream.rows), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(666/2,0), cv::Point(666/2,stream.rows), cv::Scalar(100,0,0), 1);
    // horizontal lines
    cv::line(stream, cv::Point(0,53/2), cv::Point(stream.cols,53/2), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,106/2), cv::Point(stream.cols,106/2), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,213/2), cv::Point(stream.cols,213/2), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,266/2), cv::Point(stream.cols,266/2), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,373/2), cv::Point(stream.cols,373/2), cv::Scalar(100,0,0), 1);
    cv::line(stream, cv::Point(0,426/2), cv::Point(stream.cols,426/2), cv::Scalar(100,0,0), 1);
  }

}