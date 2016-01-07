#include "ply.h"

ply::ply():
  mTag("PLY WRITER\t"),
  mAuthor(),
  mObjectName(),
  mPcl(),
  mDMap()
{
  LOG(INFO) << mTag << "initialized" << std::endl;
}

ply::ply(std::string const& author, std::string const& object_name):
  mTag("PLY WRITER\t"),
  mAuthor(author),
  mObjectName(object_name),
  mPcl(),
  mDMap()
{
  LOG(INFO) << mTag << "initialized" << std::endl;
}

ply::ply(std::string const& author, std::string const& object_name, cv::Mat const& disparity_map):
  mTag("PLY WRITER\t"),
  mAuthor(author),
  mObjectName(object_name),
  mPcl(),
  mDMap(disparity_map)
{
  LOG(INFO) << mTag << "initialized" << std::endl;
}

ply::~ply()
{
  LOG(INFO) << mTag << "destroyed!" << std::endl;
}

// -----------------------------------------------------------------------------
// --- methods -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void ply::init()
{}

bool ply::write(std::string const& filename, std::vector<cv::Mat> const& to_write, int mode)
{
  std::cout << mDMap.rows << std::endl;
  std::cout << mDMap.cols << std::endl;

  switch(mode){
    case MODE::PLAIN:
    {
      // create filestream
      mPcl.open(filename);

      // write header
      mPcl << "ply\nformat ascii 1.0\ncomment author: "<< mAuthor 
           <<"\ncomment object:" << mObjectName << "\n";
      mPcl << "element vertex " << std::to_string(to_write.size()) <<"\n";
      mPcl << "property float x\nproperty float y\nproperty float z\nend_header\n";

      // write vertex list
      for (int i = 0; i < to_write.size(); ++i) {
        cv::Mat_<float> temp = to_write[i];
        mPcl << temp(0) << " " 
             << temp(1) << " "
             << temp(2) << "\n";
      }      
      return true;
    }
    break;
    case MODE::WITH_COLOR:
    {
      if(mDMap.rows == 0 || mDMap.cols == 0)
        return false;

      // create filestream
      mPcl.open(filename);

      std::cout << "with color called" << std::endl;
      // get min max range of 
      auto minmax = Utility::calcMinMaxDisparity(mDMap);

      // write header
      mPcl << "ply\nformat ascii 1.0\ncomment author: "<< mAuthor 
           <<"\ncomment object:" << mObjectName << "\n";
      mPcl << "element vertex " << std::to_string(to_write.size()) <<"\n";
      mPcl << "property float x\n" <<
              "property float y\n" <<
              "property float z\n";
      mPcl << "property uchar red\n" <<
              "property uchar green\n" <<
              "property uchar blue\n";
      mPcl << "end_header\n";

      // write vertex list
      for (int i = 0; i < to_write.size(); ++i) {
        cv::Mat_<float> temp = to_write[i];       
        mPcl << temp(0) << " " 
             << temp(1) << " "
             << temp(2) << " ";
        // write greyscale colors
        mPcl << int((temp(2) - minmax.first)/(minmax.second - minmax.first) * 255.0) << " "
             << int((temp(2) - minmax.first)/(minmax.second - minmax.first) * 255.0) << " "
             << int((temp(2) - minmax.first)/(minmax.second - minmax.first) * 255.0) << "\n";
      }
      return true;
    }
    break;
    case MODE::WITH_COLOR_SHADING:
    {
      if(mDMap.rows == 0 || mDMap.cols == 0)
      return false;

      // create filestream
      mPcl.open(filename);

      std::cout << "with color shading called" << std::endl;

      // get min max range of 
      auto minmax = Utility::calcMinMaxDisparity(mDMap);

      // write header
      mPcl << "ply\nformat ascii 1.0\ncomment author: "<< mAuthor 
           << "\ncomment object:" << mObjectName << "\n";
      mPcl << "element vertex " << std::to_string(to_write.size()) <<"\n";
      mPcl << "property float x\n" <<
              "property float y\n" <<
              "property float z\n";
      mPcl << "property uchar red\n" <<
              "property uchar green\n" <<
              "property uchar blue\n";
      mPcl << "end_header\n";

      // write vertex list
      for (int i = 0; i < to_write.size(); ++i) {
        cv::Mat_<float> temp = to_write[i];
        mPcl << temp(0) << " " 
             << temp(1) << " "
             << temp(2) << "\n";
      }
      return true;
    }
    break;
  }
}

void ply::print_on(std::ostream& out) const
{
  out << mTag << "Name: " << mTag;
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& output, ply const& ply)
{
  ply.print_on(output);
  return output;
}