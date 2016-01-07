#ifndef __PLY_H__
#define __PLY_H__

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

#include "opencv2/core.hpp"
#include "easylogging++.h"
#include "utility.h"

class ply {

public:

  enum MODE{PLAIN, WITH_COLOR, WITH_COLOR_SHADING};

  ply();
  ply(std::string const&, std::string const&);
  ply(std::string const&, std::string const&, cv::Mat const&);
  ~ply();

  // methods
  void init();
  bool write(std::string const&, std::vector<cv::Mat> const&, int);

  // print method
  void print_on(std::ostream&) const;

private:
  std::string     mTag;
  std::string     mAuthor;
  std::string     mObjectName;
  std::ofstream   mPcl;
  cv::Mat         mDMap;
};

std::ostream& operator<<(std::ostream&, ply const&);

#endif //__PLY_H__