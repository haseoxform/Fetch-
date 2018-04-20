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

// Global Parameters
const float gRobotRadius = 0.18;
const float gRobotHeight = 0.36;
const float gEpsilon = 0.15;
const float gAMaxLin = 1 * 0.5; // m/s^2
const float gAMaxRot = 1 * 2.0; // rad/s^2
const float gVMaxLin = 0.5; // m/s
const float gVMaxRot = 1.5; // rad/s
const float gDT = 0.02; // s

// Laser Scan Parameters
float gLaserMinAngle;
float gLaserMaxAngle;
float gLaserStepSize;
float gLaserMinRange;
float gLaserMaxRange;
int gLaserScanLength;

// Publisher for velocity command.
ros::Publisher velocity_command_publisher_;

// Publisher for velocity command marker.
ros::Publisher command_marker_publisher_;

// Publisher for robot's trajectory.
ros::Publisher trajectory_publisher_;

// Subscriber for obstacle laser scan.
ros::Subscriber laser_scan_subscriber_;

// Last received odometry message.
Odometry last_odometry_;
Vector2f last_odom_(0,0);

// Sensor to robot frame transformation
MatrixXf robot_R_sensor_;
Vector3f robot_T_sensor_;

LaserScan obstacle_laser_scan_;

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPointToVector(const Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32.
Point32 ConvertVectorToPoint(const Vector3f& vector) {
  Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

// Helper function to find the free path length
bool FreePathLength(float* free_path_length, const Vector2f& point,
                    const Vector2f& vel) {

  int is_obstacle = false;
  *free_path_length = gLaserMaxRange;

  if(vel(1) == 0) {
    *free_path_length = point(0) - gRobotRadius;
    is_obstacle = true;
    return is_obstacle;
  }
  // radius of rotation
  float R = fabs(vel(0)/vel(1));
  Vector2f center(0, 0);
  center(1) = (vel(1) > 0)? R : -R;
  Vector2f center_to_obstacle(point(0) - center(0), point(1) - center(1));
  float dist_to_center = sqrt(center_to_obstacle(0) * center_to_obstacle(0) +
                              center_to_obstacle(1) * center_to_obstacle(1));

  float min_dist = R - gRobotRadius;
  float max_dist = R + gRobotRadius;
  if(min_dist <= dist_to_center && dist_to_center <= max_dist) {
    is_obstacle = true;
    float obstacle_angle = atan2(center_to_obstacle(1), center_to_obstacle(0));
    *free_path_length = (obstacle_angle < 0) ? ( M_PI/2 + obstacle_angle) * R :
                                               ( M_PI/2 - obstacle_angle) * R;
    float delta = fabs(dist_to_center - R);
    float correction = sqrt(gRobotRadius * gRobotRadius - delta * delta);
    *free_path_length -= correction;
  }

  return is_obstacle;
}

void InitializeDynamicWindow(MatrixXf& win_v, MatrixXf& win_w,
                             const Vector2f& V0) {

  int size = win_v.rows();
  int center_ind = (size - 1)/2;
  float v_step = gAMaxLin * gDT / ((size - 1)/2);
  float w_step = gAMaxRot * gDT / ((size - 1)/2);
  float v_bias = 0.1;


  for(int i = 0; i < size; ++i) {
    for(int j = 0; j < size; ++j) {
      float v = (i - center_ind) * v_step + V0(0);
      float w = (j - center_ind) * w_step + V0(1);
      v = (v > gVMaxLin)? gVMaxLin : v;
      v = (v < v_bias) ? v_bias : v;
      w = (w > gVMaxRot)? gVMaxRot : w;
      win_v(i,j) = v;
      win_w(i,j) = w;
    }
  }
}

// Convert a laser scan from polar coordinate to cartesian coordinate system
void LaserScanToPoint(const vector<float>& ranges, vector<Vector2f>& points) {

  float min_angle = gLaserMinAngle;
  float max_angle = gLaserMaxAngle;
  float step_size = gLaserStepSize;
  size_t laser_length = gLaserScanLength;

  for(size_t i =0; i < ranges.size(); ++i) {
    float angle = gLaserMinAngle + i * step_size;
    Vector2f point(cos(angle) * ranges[i], sin(angle) * ranges[i]);
    points.push_back(point);
  }
}

void LaserScanToRobotFrame(const vector<float>& laser_frame_ranges,
                           vector<float>& robot_frame_ranges) {
  vector<Vector2f> P_2d;
  LaserScanToPoint(laser_frame_ranges, P_2d);

  robot_frame_ranges.clear();
  for (size_t i = 0; i < P_2d.size(); i++) {
    Vector3f P(P_2d[i](0), P_2d[i](1), 0.0);
    Vector3f P_prime = robot_R_sensor_ * P + robot_T_sensor_;

    // Set ranges for points detected "behind" robot to zero
    if (P_prime(0) <= 0.0) {
      robot_frame_ranges.push_back(0.0);
      continue;
    }

    float dist = sqrt(P_prime(0) * P_prime(0) + P_prime(1) * P_prime(1));
    robot_frame_ranges.push_back(dist);
  }
}

// Evaluate a cost function over the dynamic window and return the index of
// the best cell
void GetBestCommand(const MatrixXf& win_v, const MatrixXf& win_w,
                    const vector<Vector2f>& obstacles, MatrixXf& scores,
                    MatrixXd& is_admissible, Vector2f& index_best) {

  float v_coeff = 1;
  float angle_coeff = -0.003; // -5;
  float dist_coeff = 1; //10;

  size_t size = win_v.rows();
  Vector2f current_best_index(-1, -1);
  float current_best_score = -1000000;
  Vector2f vel;
  MatrixXf free_path_m(size, size);

  for(size_t i = 0; i < size; ++i) {
    for(size_t j = 0; j < size; ++j) {
      vel(0) = win_v(i,j);
      vel(1) = win_w(i,j);
      float free_path_length = gLaserMaxRange;
      float distance = gLaserMaxRange;

      // Go over all the obstacle points in the laser scan to find the free
      // path length
      for(size_t k = 0; k < obstacles.size(); ++k) {
        float free_path_length_current;
        bool current_obstacle =
          FreePathLength(&free_path_length_current, obstacles[k], vel);

        if(free_path_length_current < free_path_length) {
          free_path_length = free_path_length_current;
        }

        if (!current_obstacle && free_path_length_current < distance) {
          distance = free_path_length_current;
        }
      }

      // Check if the current velocity is admissible
      bool is_admissible_tmp = true;

      if(vel(0) > sqrt(2 * gAMaxLin * free_path_length)){
        is_admissible_tmp = false;
      }

      is_admissible(i,j) = (is_admissible_tmp == true) ? 1: 0;

      // Calculate the score for current velocity
      scores(i, j) = (v_coeff * vel(0)) +
                     (angle_coeff * fabs(vel(1)) * gDT) +
                     (dist_coeff * distance);

      // Keep the best score so far
      if(is_admissible_tmp == 1 && scores(i, j) > current_best_score) {
        current_best_score = scores(i, j);
        current_best_index << i, j;
      }
      free_path_m(i,j) = free_path_length;
    }
  }
  index_best = current_best_index;
}

void PredictTrajectory(vector<Vector3f>& points, float v, float w) {
  Vector2f vel(v, w);
  float step_size = 0.1; // meters
  int step_num = (int)(4.0 / step_size);

  if(vel(1) == 0) {
    // Straight Path
    float length = 0;
    for(int i = 0; i < step_num; ++i) {
      length += step_size;
      Vector3f new_point;
      new_point(0) = 0;
      new_point(1) = length;
      new_point(2) = 0;
      points.push_back(new_point);
    }
  } else {
    // radius of rotation
    float R = fabs(vel(0)/vel(1));
    Vector2f center(0, 0);
    center(1) = (vel(1) > 0)? R: -R;
    float angle_increment = step_size / fabs(R);

    float theta = 0;
    for(int i = 0; i < step_num; ++i) {
      theta += angle_increment;
      float del_x = R * sin(theta);
      float del_y = R * cos(theta);
      Vector3f new_point;
      new_point(0) = center(0) + del_x;
      new_point(1) = (center(1) > 0)? -del_y + center(1) : del_y + center(1);
      new_point(2) = 0;
      points.push_back(new_point);
    }
  }
}

bool CheckPointService(
    compsci403_assignment5::CheckPointSrv::Request& req,
    compsci403_assignment5::CheckPointSrv::Response& res) {

  // Observed point.
  const Vector2f P(req.P.x, req.P.y);
  // Desired velocity vector.
  const Vector2f V(req.v, req.w);
  bool is_obstacle = false;
  float free_path_length = 0.0;

  // Compute is_obstacle and free_path_length.
  is_obstacle = FreePathLength(&free_path_length, P, V);

  res.free_path_length = free_path_length;
  res.is_obstacle = is_obstacle;
  return true;
}

bool GetFreePathService(
    compsci403_assignment5::GetFreePathSrv::Request& req,
    compsci403_assignment5::GetFreePathSrv::Response& res) {

  const LaserScan& laser_scan = req.laser_scan;
  const Vector2f V(req.v, req.w);

  bool is_obstacle = false;
  float free_path_length = laser_scan.range_max + gRobotRadius;
  for (size_t i = 0; i < laser_scan.ranges.size(); i++) {
    if (laser_scan.ranges[i] < laser_scan.range_min ||
        laser_scan.ranges[i] > laser_scan.range_max) {
      continue;
    }

    const float angle = laser_scan.angle_min +
                        laser_scan.angle_increment * static_cast<float>(i);
    const Vector2f P = laser_scan.ranges[i] * Vector2f(cos(angle), sin(angle));

    // Ignore points "behind" the robot
    if (P(0) <= 0.0) {
      continue;
    }

    float current_path_length = laser_scan.range_max + gRobotRadius;
    bool current_obstacle = false;
    current_obstacle = FreePathLength(&current_path_length, P, V);

    free_path_length = min(current_path_length, free_path_length);
    is_obstacle = current_obstacle || is_obstacle;
  }

  res.is_obstacle = is_obstacle;
  res.free_path_length = free_path_length;

  return true;
}

bool ObstacleLaserScanService(
    compsci403_assignment5::ObstacleLaserScanSrv::Request& req,
    compsci403_assignment5::ObstacleLaserScanSrv::Response& res) {

  const LaserScan& S = req.S;

  Matrix3f R = MatrixXf::Identity(3,3);
  for (int row; row < 3; row++) {
    for (int col; col < 3; col++) {
      R(row, col) = req.R[row*3 + col];
    }
  }

  Vector3f T = ConvertPointToVector(req.T);

  LaserScan S_prime;
  S_prime.header.stamp = ros::Time::now();
  S_prime.header.frame_id = "/base_footprint";
  S_prime.angle_min = S.angle_min;
  S_prime.angle_max = S.angle_max;
  S_prime.angle_increment = S.angle_increment;
  S_prime.time_increment = S.time_increment;
  S_prime.scan_time = S.scan_time;
  S_prime.range_min = S.range_min;
  S_prime.range_max = S.range_max;

  vector<float> robot_frame_ranges;
  LaserScanToRobotFrame(S.ranges, robot_frame_ranges);
  S_prime.ranges = robot_frame_ranges;

  res.S_prime = S_prime;

  return true;
}

bool GetCommandVelService(
    compsci403_assignment5::GetCommandVelSrv::Request& req,
    compsci403_assignment5::GetCommandVelSrv::Response& res) {

  // The input v0 and w0 are each vectors. The x component of v0 is the linear
  // velocity towards forward direction and the z component of w0 is the
  // rotational velocity around the z axis, i.e. around the center of rotation
  // of the robot and in counter-clockwise direction
  const Vector2f V(req.v_0, req.w_0);

  // Convert the laser scan to 2D points in cartesian coordinate system
  vector<Vector2f> obstacles;
  LaserScanToPoint(obstacle_laser_scan_.ranges, obstacles);

  // Implement dynamic windowing approach to find the best velocity command for next time step
  int win_size = 41;
  MatrixXf win_v(win_size, win_size);
  MatrixXf win_w(win_size, win_size);
  MatrixXf scores(win_size, win_size);
  MatrixXd is_admissible(win_size, win_size);
  Vector2f index_best;
  InitializeDynamicWindow(win_v, win_w, V);
  GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best);

  // Return the best velocity command
  // Cv is of type Point32 and its x component is the linear velocity towards
  // forward direction; you do not need to fill its other components
  // Take the best velocity based on the cost function contingent upon there
  // has been any admissible velocities
  if(index_best(0) != -1)
    res.C_v = win_v(index_best(0), index_best(1));
  else
    res.C_v = 0;

  // Cw is of type Point32 and its z component is the rotational velocity
  // around z axis; you do not need to fill its other components
  if(index_best(0) != -1)
    res.C_w = win_w(index_best(0), index_best(1));
  else
    res.C_w = 0;

  return true;
}

void OdometryCallback(const nav_msgs::Odometry& odometry) {
  last_odometry_ = odometry;
}

void LaserScanCallback(const sensor_msgs::LaserScan& laser_scan) {

  obstacle_laser_scan_ = laser_scan;
  gLaserMinAngle = obstacle_laser_scan_.angle_min;
  gLaserMaxAngle = obstacle_laser_scan_.angle_max;
  gLaserStepSize = obstacle_laser_scan_.angle_increment;
  gLaserMinRange = obstacle_laser_scan_.range_min;
  gLaserMaxRange = obstacle_laser_scan_.range_max;
  gLaserScanLength = ((gLaserMaxAngle - gLaserMinAngle) / gLaserStepSize) + 1;

  Twist command_vel;
  Marker command_vel_marker;
  PointCloud trajectory_pub;

  command_vel_marker.header = laser_scan.header;
  command_vel_marker.scale.x = 0.1;
  command_vel_marker.scale.y = 0.1;
  command_vel_marker.color.a = 0.5;
  command_vel_marker.color.r = 255;
  command_vel_marker.type = command_vel_marker.ARROW;
  command_vel_marker.action = command_vel_marker.ADD;

  // TODO: For best results, filter and smooth the input odometry readings to
  // reduce jittery movements. Here the velocity commands from previous time
  // step is given as current velocity.
  const float v0 = last_odom_(0);
  const float w0 = last_odom_(1);

  // TODO: Comment the above two lines and uncomment the following two lines to
  // use the odometry readings as current velocity
  // const float v0 = last_odometry_.twist.twist.linear.x;
  // const float w0 = last_odometry_.twist.twist.angular.z;

  Vector2f V(v0, w0);

  // Convert the laser scan to 2D points in cartesian coordinate system
  vector<Vector2f> obstacles;
  LaserScanToPoint(obstacle_laser_scan_.ranges, obstacles);

  // Implement dynamic windowing approach to find the best velocity command for
  // next time step
  int win_size = 41;
  MatrixXf win_v(win_size, win_size);
  MatrixXf win_w(win_size, win_size);
  MatrixXf scores(win_size, win_size);
  MatrixXd is_admissible(win_size, win_size);
  Vector2f index_best;
  InitializeDynamicWindow(win_v, win_w, V);
  GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best);

  // Return the best velocity command
  // Cv is of type Point32 and its x component is the linear velocity towards
  // forward direction; you do not need to fill its other components
  // Take the best velocity based on the cost function contingent upon there
  // has been any admissible velocities
  if(index_best(0) != -1) {
    command_vel.linear.x = win_v(index_best(0), index_best(1));
  } else {
    command_vel.linear.x = 0;
  }

  // Cw is of type Point32 and its z component is the rotational velocity
  // around z axis; you do not need to fill its other components
  if(index_best(0) != -1) {
    command_vel.angular.z = win_w(index_best(0), index_best(1));
  } else {
    command_vel.angular.z = 0;
  }

  // Create the robots future trajectory
  vector<Vector3f> trajectory;
  PredictTrajectory(trajectory, command_vel.linear.x, command_vel.angular.z);

  // Publish the robot's trajectory for debugging purposes
  for(size_t i = 0; i < trajectory.size(); ++i) {
    trajectory_pub.points.push_back(ConvertVectorToPoint(trajectory[i]));
  }
  trajectory_publisher_.publish(trajectory_pub);

  // Publish the velocity command marker for debugging purposes
  vector<Point> marker_end_points(2);
  Point start;
  start.x = 0;
  start.y = 0;
  start.z = 0;

  Point end;
  end.x = command_vel.linear.x;
  end.y = command_vel.angular.z;
  end.z = 0;

  marker_end_points[0] = start;
  marker_end_points[1] = end;

  command_vel_marker.points = marker_end_points;

  last_odom_(0) = command_vel.linear.x;
  last_odom_(1) = command_vel.angular.z;

  command_marker_publisher_.publish(command_vel_marker);
  velocity_command_publisher_.publish(command_vel);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_assignment5");
  ros::NodeHandle n;

  ros::service::waitForService("/COMPSCI403/GetTransformation");
  ros::ServiceClient get_transform_client =
    n.serviceClient<compsci403_assignment5_helper::GetTransformationSrv>(
      "/COMPSCI403/GetTransformation");
  compsci403_assignment5_helper::GetTransformationSrv get_transform_srv;
  if (get_transform_client.call(get_transform_srv)) {
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        robot_R_sensor_(row,col) = get_transform_srv.response.R[row*3 + col];
      }
    }
    robot_T_sensor_ = ConvertPointToVector(get_transform_srv.response.T);
  } else {
    ROS_WARN("Failed to call GetTransformationSrv; using default transforms.");
    robot_R_sensor_ = MatrixXf::Identity(3,3);
    robot_T_sensor_ = Vector3f(0.13, 0, 0.305);
  }

  ros::ServiceServer service1 = n.advertiseService(
    "/COMPSCI403/CheckPoint", CheckPointService);
  ros::ServiceServer service2 = n.advertiseService(
    "/COMPSCI403/GetFreePath", GetFreePathService);
  ros::ServiceServer service3 = n.advertiseService(
    "/COMPSCI403/ObstacleLaserScan", ObstacleLaserScanService);
  ros::ServiceServer service4 = n.advertiseService(
    "/COMPSCI403/GetCommandVel", GetCommandVelService);

  velocity_command_publisher_ =
    n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  command_marker_publisher_ =
    n.advertise<Marker>("/COMPSCI403/VelocityCommandMarkerTest", 1);

  trajectory_publisher_ =
    n.advertise<PointCloud>("/COMPSCI403/TrajectoryTest", 1);

  ros::Subscriber laser_scan_subscriber_ =
    n.subscribe("/COMPSCI403/LaserScan", 1, LaserScanCallback);

  ros::Subscriber odometry_subscriber =
    n.subscribe("/odom", 1, OdometryCallback);

  ros::spin();

  return 0;
}
