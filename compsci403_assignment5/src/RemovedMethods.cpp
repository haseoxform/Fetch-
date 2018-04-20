// Helper function to find the free path length
bool FreePathLength(float* free_path_length, const Vector2f& point, const Vector2f& vel) {

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

// Evaluate a cost function over the dynamic window and return the index of
// the best cell
void GetBestCommand(const MatrixXf& win_v, const MatrixXf& win_w, const vector<Vector2f>& obstacles, MatrixXf& scores, MatrixXd& is_admissible, Vector2f& index_best) {

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

bool CheckPointService(compsci403_assignment5::CheckPointSrv::Request& req, compsci403_assignment5::CheckPointSrv::Response& res) {

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

bool GetFreePathService(compsci403_assignment5::GetFreePathSrv::Request& req, compsci403_assignment5::GetFreePathSrv::Response& res) {

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

//CUT BUT WILL USE SIMILAR IDEAS
bool GetCommandVelService(compsci403_assignment5::GetCommandVelSrv::Request& req, compsci403_assignment5::GetCommandVelSrv::Response& res) {

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
  MatrixXf scores(win_size, win_size); //REMOVED
  MatrixXd is_admissible(win_size, win_size);
  Vector2f index_best;
  InitializeDynamicWindow(win_v, win_w, V);
  GetBestCommand(win_v, win_w, obstacles, scores, is_admissible, index_best); //REMOVED

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