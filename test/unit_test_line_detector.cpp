#include <gtest/gtest.h>

#include "line_detector.h"

class LineDetectorTest : public ::testing::Test
{
public:
  LineDetectorTest(void)
  {

  }

  void SetUp(void)
  {

  }

  void TearDown(void)
  {

  }

  virtual ~LineDetectorTest(void)
  {

  }

protected:
  francor::perception::LineDetector detector_;
};

TEST_F(LineDetectorTest, FittingLine)
{
  // that points should result in a line with m = 1.0 and t = 0.0.
  const francor::perception::PointVector points = { Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0),
                                                    Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(3.0, 3.0),
                                                    Eigen::Vector2d(4.0, 4.0), Eigen::Vector2d(5.0, 5.0) };
  const francor::perception::LaserScan scan(points);
  const std::vector<std::size_t> indices = { 0, 1, 2, 3, 4, 5 };

  francor::perception::LineSegment line(detector_.fittingLine(scan, indices));

  EXPECT_NEAR(line.line().m(), 1.0, 0.001);
  EXPECT_NEAR(line.line().t(), 0.0, 0.001);

  EXPECT_NEAR(line.p0().x(), 0.0, 0.001);
  EXPECT_NEAR(line.p0().y(), 0.0, 0.001);
  EXPECT_NEAR(line.p1().x(), 5.0, 0.001);
  EXPECT_NEAR(line.p1().y(), 5.0, 0.001);
}

TEST_F(LineDetectorTest, FindNextLineSingle)
{
  // that points should result in a line with m = 1.0 and t = 0.0.
  const francor::perception::PointVector points = { Eigen::Vector2d( 0.0,  0.0), Eigen::Vector2d( 1.0,  1.0),
                                                    Eigen::Vector2d( 2.0,  2.0), Eigen::Vector2d( 3.0,  3.0),
                                                    Eigen::Vector2d( 4.0,  4.0), Eigen::Vector2d( 5.0,  5.0),
                                                    Eigen::Vector2d( 6.0,  6.0), Eigen::Vector2d( 7.0,  7.0),
                                                    Eigen::Vector2d( 8.0,  8.0), Eigen::Vector2d( 9.0,  9.0),
                                                    Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d(11.0, 11.0) };
  const francor::perception::LaserScan scan(points);
  francor::perception::LineSegment line;

  detector_.setupForScan(scan);
  const int foundPoints = detector_.findNextLine(scan, 0, points.size() - 1, line);

  EXPECT_EQ(foundPoints, points.size());

  EXPECT_NEAR(line.line().m(), 1.0, 0.001);
  EXPECT_NEAR(line.line().t(), 0.0, 0.001);
  EXPECT_NEAR(line.p0().x(), 0.0, 0.001);
  EXPECT_NEAR(line.p0().y(), 0.0, 0.001);
  EXPECT_NEAR(line.p1().x(), 11.0, 0.001);
  EXPECT_NEAR(line.p1().y(), 11.0, 0.001);
}

TEST_F(LineDetectorTest, FindNextLineMulti)
{
  // that points should result in two lines with m = 1.0, t = 0.0 and m = 1.0, t = -20.
  const francor::perception::PointVector points = { Eigen::Vector2d( 0.0,  0.0), Eigen::Vector2d( 1.0,  1.0),
                                                    Eigen::Vector2d( 2.0,  2.0), Eigen::Vector2d( 3.0,  3.0),
                                                    Eigen::Vector2d( 4.0,  4.0), Eigen::Vector2d( 5.0,  5.0),
                                                    Eigen::Vector2d( 6.0,  6.0), Eigen::Vector2d( 7.0,  7.0),
                                                    Eigen::Vector2d( 8.0,  8.0), Eigen::Vector2d( 9.0,  9.0),
                                                    Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d(11.0, 11.0),
                                                    Eigen::Vector2d(20.0,  0.0), Eigen::Vector2d(21.0,  1.0),
                                                    Eigen::Vector2d(22.0,  2.0), Eigen::Vector2d(23.0,  3.0),
                                                    Eigen::Vector2d(24.0,  4.0), Eigen::Vector2d(25.0,  5.0),
                                                    Eigen::Vector2d(26.0,  6.0), Eigen::Vector2d(27.0,  7.0),
                                                    Eigen::Vector2d(28.0,  8.0), Eigen::Vector2d(29.0,  9.0) };
  const francor::perception::LaserScan scan(points);
  francor::perception::LineSegment line;

  detector_.setupForScan(scan);
  int foundPoints = detector_.findNextLine(scan, 0, points.size() - 1, line);

  if (foundPoints == 12)
  {
    EXPECT_NEAR(line.line().m(), 1.0, 0.001);
    EXPECT_NEAR(line.line().t(), 0.0, 0.001);

    EXPECT_NEAR(line.p0().x(), 0.0, 0.001);
    EXPECT_NEAR(line.p0().y(), 0.0, 0.001);
    EXPECT_NEAR(line.p1().x(), 11.0, 0.001);
    EXPECT_NEAR(line.p1().y(), 11.0, 0.001);
  }
  else if (foundPoints == 10)
  {
    EXPECT_NEAR(line.line().m(),   1.0, 0.001);
    EXPECT_NEAR(line.line().t(), -20.0, 0.001);

    EXPECT_NEAR(line.p0().x(), 20.0, 0.001);
    EXPECT_NEAR(line.p0().y(),  0.0, 0.001);
    EXPECT_NEAR(line.p1().x(), 29.0, 0.001);
    EXPECT_NEAR(line.p1().y(),  9.0, 0.001);
  }
  else
  {
    EXPECT_TRUE(false);
  }
}

TEST_F(LineDetectorTest, Process)
{
  // that points should result in two lines with m = 1.0, t = 0.0 and m = 1.0, t = -20.
  const francor::perception::PointVector points = { Eigen::Vector2d( 0.0,  0.0), Eigen::Vector2d( 1.0,  1.0),
                                                    Eigen::Vector2d( 2.0,  2.0), Eigen::Vector2d( 3.0,  3.0),
                                                    Eigen::Vector2d( 4.0,  4.0), Eigen::Vector2d( 5.0,  5.0),
                                                    Eigen::Vector2d( 6.0,  6.0), Eigen::Vector2d( 7.0,  7.0),
                                                    Eigen::Vector2d( 8.0,  8.0), Eigen::Vector2d( 9.0,  9.0),
                                                    Eigen::Vector2d(10.0, 10.0), Eigen::Vector2d(11.0, 11.0),
                                                    Eigen::Vector2d(20.0,  0.0), Eigen::Vector2d(21.0,  1.0),
                                                    Eigen::Vector2d(22.0,  2.0), Eigen::Vector2d(23.0,  3.0),
                                                    Eigen::Vector2d(24.0,  4.0), Eigen::Vector2d(25.0,  5.0),
                                                    Eigen::Vector2d(26.0,  6.0), Eigen::Vector2d(27.0,  7.0),
                                                    Eigen::Vector2d(28.0,  8.0), Eigen::Vector2d(29.0,  9.0) };
  const francor::perception::LaserScan scan(points);
  std::vector<francor::perception::LineSegment> lines;

  detector_.process(scan, lines);

  ASSERT_EQ(lines.size(), 2);

  EXPECT_NEAR(lines[0].line().m(), 1.0, 0.001);
  EXPECT_NEAR(lines[1].line().m(), 1.0, 0.001);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}