#include "line_detector.h"

namespace francor {

namespace perception {

LineDetector::LineDetector(void)
  : gen_(rd_())
{
  
}

void LineDetector::process(const LaserScan& scan, std::vector<LineSegment>& lines)
{
  if (scan.points() < 10)
    return;
//  std::cout << __PRETTY_FUNCTION__ << std::endl;
  constexpr size_t windowSize = 200;
  int associatedPoints = 0;
  LineSegment line;

  this->setupForScan(scan);

  do
  {
//    std::cout << "search for line" << std::endl;
    const int points = this->findNextLine(scan, 0, scan.points() - 1, line);

    if (points == 0)
      break;

//    std::cout << "found line with " << points << std::endl;
    associatedPoints += points;
//    std::cout << "associated points = " << associatedPoints << " and scan contains " << scan.points() << " points" << std::endl;
    lines.push_back(line);
  }
  while (associatedPoints < (scan.points() * 95) / 100); // 95%
}

void LineDetector::setupForScan(const LaserScan& scan)
{
  point_mask_.clear();
  point_mask_.resize(scan.points(), false);
}

int LineDetector::findNextLine(const LaserScan& scan, const std::size_t begin, const std::size_t end, LineSegment& segment)
{
//  std::cout << __PRETTY_FUNCTION__ << std::endl;
  constexpr double epsilon = 0.05;
  constexpr int maxSkipped = 10;
  constexpr int maxIterations = 200;
  std::uniform_int_distribution<> dis(begin, end);
  int idxP0;
  int idxP1;
  std::size_t best = 0;
  std::vector<std::size_t> toMask;

  for (int iteration = 0; iteration < maxIterations; ++iteration)
  {
    //std::cout << "iteration = " << iteration << std::endl;
    do { idxP0 = dis(gen_); } while (point_mask_[idxP0]);
    do { idxP1 = dis(gen_); } while (point_mask_[idxP1] || idxP0 == idxP1);
//    std::cout << "got " << idxP0 << " and " << idxP1 << " as index" << std::endl;
    Line line((scan[idxP1] - scan[idxP0]).normalized(), scan[idxP0]);

    std::vector<std::size_t> linePointIndices;
    int skippedCount = 0;
    bool validLine = true;

    // wake in postive direction
    for (std::size_t i = std::min(idxP0, idxP1); i < scan.points(); ++i)
    {
//      std::cout << "line.distanceTo(scan[" << i << "] = " << line.distanceTo(scan[i]) << std::endl;
      if (!point_mask_[i] && line.distanceTo(scan[i]) <= epsilon)
      {
        linePointIndices.push_back(i);
        skippedCount = 0;  
      }
      else
      {
        if (++skippedCount >= maxSkipped)
        {
//          std::cout << "skipped count reached maximum" << std::endl;
          // if the second point wasn't reached the line is invalid
          if (i < std::max(idxP0, idxP1))
            validLine = false;

          break;
        }
      }
    }

    // wake in negative direction
    skippedCount = 0;

    for (int i = std::min(idxP0, idxP1) - 1; i >= 0; --i)
    {
//      std::cout << "line.distanceTo(scan[" << i << "] = " << line.distanceTo(scan[i]) << std::endl;
      if (!point_mask_[i] && line.distanceTo(scan[i]) <= epsilon)
      {
        linePointIndices.push_back(i);
        skippedCount = 0;  
      }
      else
      {
        if (++skippedCount >= maxSkipped)
          break;
      }
    }

    // found a valid line
//    std::cout << "found " << linePointIndices.size() << " points" << std::endl;
    if (validLine && linePointIndices.size() > best)
    {
//      std::cout << "line is valid" << std::endl;
      segment = this->fittingLine(scan, linePointIndices);
      best = linePointIndices.size();
      toMask = std::move(linePointIndices);
    }
  }

  // mask used points
  for (const auto idx : toMask)
    point_mask_[idx] = true;
  
//  return best;
  return (toMask.size() >= 10 ? best : 0);
}

LineSegment LineDetector::fittingLine(const LaserScan& scan, const std::vector<std::size_t>& indices) const
{
  // a line needs minium two points
  if (indices.size() < 2)
    return { };

  // calculate the average point
  Eigen::Vector2d avg(0.0, 0.0);

  for (const auto idx : indices)
    avg += scan[idx];

  avg /=  static_cast<double>(indices.size());

  // calculate the sum (x - avg.x) * (y - avg.y) and (x - avg.x) * (x - avg.x)
  double sumXY = 0.0;
  double sumX  = 0.0;

  for (const auto idx : indices)
  {
    sumXY += (scan[idx].x() - avg.x()) * (scan[idx].y() - avg.y());
    sumX  += (scan[idx].x() - avg.x()) * (scan[idx].x() - avg.x());
  }

  // get the start and end point of the segment
  const std::size_t begin = *std::min_element(indices.begin(), indices.end());
  const std::size_t end   = *std::max_element(indices.begin(), indices.end());

  // m is infinity --> segment too
  if (sumX == 0.0)
    return { };

  // construct line segment and return it
  const double m = sumXY / sumX;
  const double t = avg.y() - m * avg.x();
  const Line line(m, t);
  const Eigen::Vector2d n(line.n());
  const Eigen::Vector2d p0(line.intersectionPoint(Line(n, scan[begin])));
  const Eigen::Vector2d p1(line.intersectionPoint(Line(n, scan[end  ])));

  return { p0, p1 };
}

} // end namespace perception

} // end namespace francor