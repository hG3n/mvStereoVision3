#include "ObstacleDetection.h"
#include "utility.h"

ObstacleDetection::ObstacleDetection():
mTag("OBSTACLE DETECTION\t"),
mRange(),
mRangeDisparity()
{
  LOG(INFO) << mTag << "created" << std::endl;
}

ObstacleDetection::~ObstacleDetection()
{
  LOG(INFO) << mTag << "destroyed!" << std::endl;
}

// -----------------------------------------------------------------------------
// --- functions ---------------------------------------------------------------
// -----------------------------------------------------------------------------
void ObstacleDetection::print_on(std::ostream& out) const
{
  out << "Name: " << mTag;
}

// -----------------------------------------------------------------------------
// --- getter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::pair<float,float> ObstacleDetection::getRange() const
{
  return mRange;
}

// -----------------------------------------------------------------------------
// --- setter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ObstacleDetection::setRange(std::pair<float,float> const& newRange)
{
  mRange = newRange;
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& output, ObstacleDetection const& o)
{
  o.print_on(output);
  return output;
}