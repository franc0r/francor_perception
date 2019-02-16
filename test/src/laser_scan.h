#include<Eigen/StdVector>

#include <sensor_msgs/LaserScan.h>

namespace francor {

namespace perception {

using PointVector = std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>>;

class LaserScan
{
public:
  LaserScan(void) = default;
  LaserScan(const sensor_msgs::LaserScan& msg);
  LaserScan(const PointVector& points);
  LaserScan(const LaserScan&) = default;
  LaserScan(LaserScan&&) = default;
  ~LaserScan(void) = default;

  LaserScan& operator=(const LaserScan&) = default;
  LaserScan& operator=(LaserScan&) = default;

  inline Eigen::Vector2d& operator[](const std::size_t index) noexcept
  {
    assert(index >= points_.size());

    return points_[index];
  }

  inline const Eigen::Vector2d& operator[](const std::size_t index) const noexcept
  {
    assert(index >= points_.size());
    
    return points_[index];
  }

  inline std::size_t points(void) const noexcept 
  {
    return points_.size();
  }

  /**
   * Splits the laser scan in smaller segments using euclidan distance.
   * 
   * \param maxDistance Is the distance between two points greater than maxDistance a segment starts.
   * \return Each scan represents a segment.
   */
  std::vector<LaserScan> splitInSegments(const double maxDistance) const;

private:
  PointVector points_;
};

} // end namespace perception

} // end namespace francor