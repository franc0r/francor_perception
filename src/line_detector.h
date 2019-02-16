#include "line_segment.h"
#include "laser_scan.h"

#include <random>

namespace francor {

namespace perception {

class LineDetector
{
public:
  LineDetector(void);

  void process(const LaserScan& scan, std::vector<LineSegment>& lines);

// TODO: find a way to test the methods below if they are private.
  int findNextLine(const LaserScan& scan, const std::size_t begin, const std::size_t end, LineSegment& segment);
  LineSegment fittingLine(const LaserScan& scan, const std::vector<std::size_t>& indices) const;
  void setupForScan(const LaserScan& scan);

private:
  std::random_device rd_;
  std::mt19937 gen_;

  std::vector<bool> point_mask_;
};

} // end namespace perception

} // end namespace francor