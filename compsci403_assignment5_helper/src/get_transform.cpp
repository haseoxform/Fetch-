#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>

#include "compsci403_assignment5_helper/GetTransformationSrv.h"

using std::vector;

bool GetTransformationService(
    compsci403_assignment5_helper::GetTransformationSrv::Request& req,
    compsci403_assignment5_helper::GetTransformationSrv::Response& res) {

  geometry_msgs::Point32 T;

  res.R[0] = res.R[4] = res.R[8] = 1.0;
  res.R[1] = res.R[2] = res.R[3] = res.R[5] = res.R[6] = res.R[7] = 0.0;

  T.x = 0.13350;
  T.y = 0.0;
  T.z = 0.3050;
  res.T = T;

  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "get_transform");
  ros::NodeHandle n;

  ros::ServiceServer service1 = n.advertiseService(
    "/COMPSCI403/GetTransformation", GetTransformationService);

  ros::spin();

  return(0);
}
