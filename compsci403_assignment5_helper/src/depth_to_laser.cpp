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
#include <sensor_msgs/Image.h>

#include "compsci403_assignment5_helper/GetTransformationSrv.h"

using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Point;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud;
using std::cout;
using std::vector;
using namespace std;

// Global Parameters
const float robotHeight_ = 0.36;
const float epsilon_ = 0.15;

// Kinect Intrinsics
const float a_ = 3.008;
const float b_ = -0.002745;
const float px_ = 320;
const float py_ = 240;
const float fx_ = 588.446;
const float fy_ = -564.227;

// Laser Scan Parameters
const float minAngle_ = -28.0 * M_PI/ 180;
const float maxAngle_ = 28.0 * M_PI/ 180;
const float stepSize_ = 1.0 * M_PI/ 180;
const float maxRange_ = 4.0;
const float minRange_ = 0.8;
const int laserScanLength_ = 57;

Matrix3f R_;
Vector3f T_;

// // Publisher for input point cloud.
// ros::Publisher point_cloud_publisher_;
//
// // Publisher for filtered point cloud.
// ros::Publisher filtered_point_cloud_publisher_;

// Publisher for obstacle laser scan.
ros::Publisher laser_scan_publisher_;

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPointToVector(const Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32
Point32 ConvertVectorToPoint(const Vector3f& vec) {
  Point32 point;
  point.x = vec(0);
  point.y = vec(1);
  point.z = vec(2);
  return point;
}

// Helper function to filter the point cloud in order to omit the ground plane
void ObstaclePointCloudHelper(const Matrix3f& R, const Vector3f& T,
                              const vector<Vector3f>& point_cloud,
                              vector<Vector3f>& filtered_point_cloud) {

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    Vector3f point_tmp = R * point_cloud[i] + T;
    if(epsilon_ < point_tmp(2) && point_tmp(2) < robotHeight_) {
      filtered_point_cloud.push_back(point_tmp);
    }
  }
}

void PointCloudToLaserScanHelper(const vector<Vector3f>& point_cloud,
                                 vector<float>& ranges){

  // Calculate the angle of all the points in the point cloud in the polar
  // coordinate system (projected to the ground plane)
  vector<float> angles(point_cloud.size());
  for(size_t i = 0; i < point_cloud.size(); ++i) {
    angles[i] = atan2(point_cloud[i](1), point_cloud[i](0));
  }

  float ang = minAngle_;
  for(int i = 0; i < laserScanLength_; ++i) {
    // Define the query slice for current laser beam
    float start_range = ang - stepSize_/2;
    float end_range = ang + stepSize_/2;

    // The minimum distance reading in the current slice
    float min_dist = maxRange_;

    // Fill the laser scan with exhastive search.
    // A hash table could be used for faster results.
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      if(start_range <= angles[j] && angles[j] < end_range) {
        float dist = sqrt(point_cloud[j](0) * point_cloud[j](0) +
                          point_cloud[j](1) * point_cloud[j](1));
        if(dist < min_dist && minRange_ < dist && dist < maxRange_){
          min_dist = dist;
        }
      }
    }
    ranges.push_back(min_dist);
    ang += stepSize_;
  }
}

// Helper function to convert depth to 3D point
void Get3DPointFromDisparity(int x, int y, float disparity,
                             float* X, float* Y, float* Z) {

  float depth = 1/ (a_ + b_ * disparity);
  *X = depth * ((float)x - px_)/fx_;
  *Y = depth * ((float)y - py_)/fy_;
  *Z = depth;
}

void DepthImageCallback(const sensor_msgs::Image& depth_image) {

  LaserScan obstacle_laser_scan;
  obstacle_laser_scan.header.frame_id = "/base_laser";
  obstacle_laser_scan.angle_min = minAngle_;
  obstacle_laser_scan.angle_max = maxAngle_;
  obstacle_laser_scan.angle_increment = stepSize_;
  obstacle_laser_scan.range_min = minRange_;
  obstacle_laser_scan.range_max = maxRange_ + 0.1;

  // PointCloud point_cloud_pub;
  // point_cloud_pub.header = depth_image.header;
  //
  // PointCloud filtered_point_cloud_pub;
  // filtered_point_cloud_pub.header = depth_image.header;

  vector<Vector3f> point_cloud;
  for (unsigned int y = 0; y < depth_image.height; ++y) {
    for (unsigned int x = 0; x < depth_image.width; x = x + 10) {
      // Add code here to only process only every nth pixel

      uint16_t byte0 = depth_image.data[2 * (x + y * depth_image.width) + 0];
      uint16_t byte1 = depth_image.data[2 * (x + y * depth_image.width) + 1];
      if (!depth_image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the most
      // significant 5 bits to extract the lowest 11 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;

      // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics
      // and add it to point cloud.
      float X,Y,Z;
      Get3DPointFromDisparity(x, y, raw_depth, &X, &Y, &Z);
      Vector3f point(Z, -X, Y);
      point_cloud.push_back(point);
    }
  }

  // Filter the point cloud to remove the ground
  // Matrix3f R = MatrixXf::Identity(3,3);
  // Vector3f T(0.13, 0, 0.305);
  vector<Vector3f> filtered_point_cloud;
  ObstaclePointCloudHelper(R_, T_, point_cloud, filtered_point_cloud);

  // // Publish the input point cloud for debugging purposes
  // for(size_t i = 0; i < point_cloud.size(); ++i) {
  //   point_cloud_pub.points.push_back(ConvertVectorToPoint(point_cloud[i]));
  // }
  // point_cloud_publisher_.publish(point_cloud_pub);
  //
  // // Publish the filtered point cloud for debugging purposes
  // for(size_t i = 0; i < filtered_point_cloud.size(); ++i) {
  //   filtered_point_cloud_pub.points.push_back(
  //     ConvertVectorToPoint(filtered_point_cloud[i]));
  // }
  // filtered_point_cloud_publisher_.publish(filtered_point_cloud_pub);

  // Convert the point cloud to a laser scan
  vector<float> ranges;
  PointCloudToLaserScanHelper(filtered_point_cloud, ranges);

  // Publish the obstacle laser scan
  obstacle_laser_scan.ranges = ranges;
  obstacle_laser_scan.header.stamp = ros::Time::now();
  laser_scan_publisher_.publish(obstacle_laser_scan);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "depthimg_to_laserscan_converter");
  ros::NodeHandle n;

  ros::service::waitForService("/COMPSCI403/GetTransformation");
  ros::ServiceClient get_transform_client =
    n.serviceClient<compsci403_assignment5_helper::GetTransformationSrv>(
      "/COMPSCI403/GetTransformation");
  compsci403_assignment5_helper::GetTransformationSrv get_transform_srv;
  if (get_transform_client.call(get_transform_srv)) {
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        R_(row, col) = get_transform_srv.response.R[row*3 + col];
      }
    }
    T_ = ConvertPointToVector(get_transform_srv.response.T);
  } else {
    ROS_WARN("Failed to call GetTransformationSrv; using default transforms.");
    R_ = MatrixXf::Identity(3,3);
    T_ = Vector3f(0.13, 0, 0.305);
  }

  // point_cloud_publisher_ =
  //     n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/PointCloudTest",1);
  // filtered_point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud>(
  //   "/COMPSCI403/FilteredPointCloudTest",1);

  laser_scan_publisher_ =
    n.advertise<sensor_msgs::LaserScan>("/COMPSCI403/LaserScan",1);

  ros::Subscriber depth_image_subscriber =
    n.subscribe("/COMPSCI403/DepthImage", 1, DepthImageCallback);

  ros::spin();

  return 0;
}
