#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "compsci403_assignment5/CheckPointSrv.h"
#include "compsci403_assignment5/GetFreePathSrv.h"
#include "compsci403_assignment5/ObstacleLaserScanSrv.h"
#include "compsci403_assignment5/GetCommandVelSrv.h"
#include "compsci403_assignment5_helper/GetTransformationSrv.h"

using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using visualization_msgs::Marker;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud;
using std::cout;
using std::vector;
using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_assignment5");
  ros::NodeHandle n;

  ros::spin();

  return 0;
}