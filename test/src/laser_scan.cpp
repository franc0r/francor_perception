#include "laser_scan.h"

namespace francor {

namespace perception {

LaserScan::LaserScan(const sensor_msgs::LaserScan& msg)
{
  assert(msg.angle_min > msg.angle_max);
  assert(msg.range_min > msg.range_max);

  points_.reserve(msg.ranges.size());

  for (std::size_t i = 0; i < msg.ranges.size(); ++i)
  {
    if (std::isinf(msg.ranges[i]))
      continue;

    const double angle = static_cast<double>(i) * msg.angle_increment + msg.angle_min;

    points_.push_back(Eigen::Vector2d(std::cos(angle) * msg.ranges[i], std::sin(angle) * msg.ranges[i]));
  }
}

LaserScan::LaserScan(const PointVector& points)
  : points_(points)
{
  
}

std::vector<LaserScan> LaserScan::splitInSegments(const double maxDistance) const
{
  // if this scan contains less than two points return a copy of this scan
  if (this->points() < 2)
    return { 1, *this };

  const double sqrMaxDistance = maxDistance * maxDistance;
  std::vector<LaserScan> segments;

  auto itP0 = points_.begin();
  auto itP1 = itP0 + 1;
  auto itBeginSegment = itP0;

  while (itP1 < points_.end())
  {
    if ((*itP1 - *itP0).squaredNorm() > sqrMaxDistance)
    {
      segments.push_back(LaserScan(PointVector(itBeginSegment, itP1)));
      itBeginSegment = itP1;
    }

    ++itP0;
    ++itP1;
  }

  segments.push_back(LaserScan(PointVector(itBeginSegment, points_.end())));
  return segments;
}

} // end namespace perception

} // end namespace francor