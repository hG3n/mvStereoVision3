#include "view.h"

void View::drawObstacleGrid(cv::Mat &stream, int binning)
{
  int x = stream.cols/3;
  int y = stream.rows/3;

  //vertical lines
  cv::line(stream, cv::Point(x,0),   cv::Point(x,stream.rows),   cv::Scalar(0,0,255), 1);
  cv::line(stream, cv::Point(2*x,0), cv::Point(2*x,stream.rows), cv::Scalar(0,0,255), 1);

  //horizontal lines
  cv::line(stream, cv::Point(0,y),   cv::Point(stream.cols, y), cv::Scalar(0,0,255), 1);
  cv::line(stream, cv::Point(0,y*2), cv::Point(stream.cols, y*2), cv::Scalar(0,0,255), 1);
}

void View::drawSubimageGrid(cv::Mat &stream, int binning)
{
  int x = stream.cols/9;
  int y = stream.rows/9;
  
  // vertical lines
  cv::line(stream, cv::Point(x  ,0), cv::Point(x  ,stream.rows), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(x*2,0), cv::Point(x*2,stream.rows), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(x*4,0), cv::Point(x*4,stream.rows), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(x*5,0), cv::Point(x*5,stream.rows), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(x*7,0), cv::Point(x*7,stream.rows), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(x*8,0), cv::Point(x*8,stream.rows), cv::Scalar(100,0,0), 1);

  // horizontal lines
  cv::line(stream, cv::Point(0,y  ), cv::Point(stream.cols,y  ), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(0,y*2), cv::Point(stream.cols,y*2), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(0,y*4), cv::Point(stream.cols,y*4), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(0,y*5), cv::Point(stream.cols,y*5), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(0,y*7), cv::Point(stream.cols,y*7), cv::Scalar(100,0,0), 1);
  cv::line(stream, cv::Point(0,y*8), cv::Point(stream.cols,y*8), cv::Scalar(100,0,0), 1);
}