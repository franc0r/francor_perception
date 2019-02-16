#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "line_detector.h"

francor::perception::LineDetector detector_;
ros::Publisher pubLines_;

void publishLines(const std::vector<francor::perception::LineSegment>& lines, const std_msgs::Header& header)
{
  if (pubLines_.getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker msg;

  msg.header = header;
  msg.id = 1;
  msg.type = msg.LINE_LIST;
  msg.scale.x = 0.1;
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.color.a = 0.2;

  msg.points.resize(lines.size() * 2);

  for (std::size_t i = 0; i < lines.size(); ++i)
  {
    msg.points[i * 2 + 0].x = lines[i].p0().x();
    msg.points[i * 2 + 0].y = lines[i].p0().y();
    msg.points[i * 2 + 1].x = lines[i].p1().x();
    msg.points[i * 2 + 1].y = lines[i].p1().y();
  }

  pubLines_.publish(msg);
}

void callbackLaserScan(const sensor_msgs::LaserScan& msg)
{
  francor::perception::LaserScan scan(msg);
  std::vector<francor::perception::LaserScan> segments(scan.splitInSegments(0.15));
  std::vector<francor::perception::LineSegment> lines;

  for (const auto& segment : segments)
    detector_.process(segment, lines);

  publishLines(lines, msg.header);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_feature_detector");
  ros::NodeHandle nh;

  pubLines_ = nh.advertise<visualization_msgs::Marker>("/perception/features/lines", 1);
  ros::Subscriber subLaserScan = nh.subscribe("/scan", 2, callbackLaserScan);

  ros::spin();

  return 0;
}