#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <common/MathConstExpr.h>
#include <laser_data_fusion/laserInfo.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <mutex>

static int g_nodeNum = 0;
std::vector<float> ranges(POINTS_PER_LOOP);
std::vector<float> intensities(POINTS_PER_LOOP);
std::vector<ros::Time> updateTime(POINTS_PER_LOOP);

void PointAnalysis(double intensity, double range, double angle, const double offset[]) {
  // Change coordinate and angle from laser to base_link
  double x = range * std::cos(offset[2] + angle) + offset[0];
  double y = range * std::sin(offset[2] + angle) + offset[1];
  double thetaInDeg = atan2(y, x) * RAD_TO_DEG;
  if (thetaInDeg < 0) {
    thetaInDeg += ANGLES_OF_LOOP;
  }

  // Classification of points and get the ranges
  int index = floor(thetaInDeg * POINTS_PER_ANGLE);
  ranges[index] = sqrt(x * x + y * y);
  intensities[index] = intensity;
  updateTime[index] = ros::Time::now();
}

void ScanFilter(const sensor_msgs::LaserScan& scan, int laserIndex) {
  for (int i = 0; i < scan.ranges.size(); ++i) {
    double angle = scan.angle_min + i * scan.angle_increment;
    if (angle <= ANGLE_RANGE[laserIndex][0] || angle >= ANGLE_RANGE[laserIndex][1]) {
      PointAnalysis(scan.intensities[i], scan.ranges[i], angle, LASER_OFFSET[laserIndex]);
    }
  }
}

void Laser0ToBase(const sensor_msgs::LaserScan& scan) {
  ScanFilter(scan, 0);
}

void Laser1ToBase(const sensor_msgs::LaserScan& scan) {
  ScanFilter(scan, 1);
}

void MergeScanMessage(sensor_msgs::LaserScan& scan, ros::Time scan_time) {
  scan.header.stamp = scan_time;
  scan.header.frame_id = "laser";
  scan.angle_min = 0;
  scan.angle_max = 2 * PI;
  scan.angle_increment = DEG_TO_RAD / POINTS_PER_ANGLE;
  scan.time_increment = 1 / (LOOP_RATE * POINTS_PER_LOOP);
  scan.range_min = DIST_RANGE[0];
  scan.range_max = DIST_RANGE[1];
  scan.scan_time = scan.time_increment * g_nodeNum;

  scan.ranges.resize(POINTS_PER_LOOP);
  scan.intensities.resize(POINTS_PER_LOOP);
  for (int index = 0; index < POINTS_PER_LOOP; ++index) {
    if ((ros::Time::now() - updateTime[index]).toSec() < 0.2) {
      scan.ranges[index] = ranges[index];
      scan.intensities[index] = intensities[index];
    }
  }
  g_nodeNum++;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_fusion");
  ros::NodeHandle n;

  std::for_each(updateTime.begin(), updateTime.end(), [](ros::Time timeNow) { timeNow = ros::Time::now(); });
  ros::Subscriber sub1 = n.subscribe("/laser_scan0", 1, Laser0ToBase);
  ros::Subscriber sub2 = n.subscribe("/laser_scan1", 1, Laser1ToBase);
  ROS_INFO("subscribe the data of two laser successfully");

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("/scan", 1000);
  sensor_msgs::LaserScan scanPub;
  ros::Rate loop_rate(LOOP_RATE);

  while (ros::ok()) {
    MergeScanMessage(scanPub, ros::Time::now());
    pub.publish(scanPub);
    ros::spinOnce();
    std::for_each(scanPub.ranges.begin(), scanPub.ranges.end(), [](auto& length) { length = DIST_RANGE[1]; });
    scanPub.intensities.clear();
    loop_rate.sleep();
  }
  return 0;
}
