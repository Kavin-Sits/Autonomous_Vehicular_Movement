//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::max;
using std::min;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(cp1_distance, 25, "Distance to travel for 1D TOC (cp1)");
DEFINE_double(cp1_curvature, 0, "Curvature for arc path (cp1)");

DEFINE_double(cp2_curvature, 0, "Curvature for arc path (cp2)");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
// seconds
const float TIME_STEP = 0.05; // Interval between time steps
const float SYSTEM_LATENCY = 0.15;
// meters/second
const float MAX_VELOCITY = 1;
// meters/second^2
const float MAX_ACCELERATION = 3;
const float MAX_DECELERATION = -3;
// meters
const float OBSTACLE_MARGIN = 0.1;
const float H = 0.4 + OBSTACLE_MARGIN; // car length
const float W = 0.15 + OBSTACLE_MARGIN; // car width
const float TEMPORARY_GOAL_DIST = 10; // carrot on a stick target
// meters^-1
const float CURVATURE_STEP = 0.2;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    // Global variables
    prev_velocity(0),
    dist_remaining(FLAGS_cp1_distance),
    next_curvature(FLAGS_cp2_curvature),
    sensor_range(0.0),

    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  
  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.

  // The latest observed point cloud is accessible via "point_cloud_"
  // printf("starting\n\n");
  /* Instantiating values related to obstacles and curvature
  ___________________________________________*/
  next_curvature = GetOptimalCurvature(CURVATURE_STEP);
  float freePathLength = GetFreePathLength(next_curvature);
  VisualizeFreePathLengths();
  // VisualizeFreePathLength(0.8);
  printf("\nFree path length: %f\n", freePathLength);
  dist_remaining = freePathLength;

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = next_curvature;
  drive_msg_.velocity = Navigation::GetVelocity(dist_remaining);
  printf("Speed: %f\n", drive_msg_.velocity);
  printf("Curvature: %f\n", drive_msg_.curvature);
  prev_velocity = drive_msg_.velocity;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  // Code to track time
  /*
  float t_start = GetMonotonicTime();
  float t_end = GetMonotonicTime();
  printf("Total time for funtion run: %f\n", t_end - t_start);
  */
}

// Given the distance remaining, handle time optimal control with latency adjustments and return the required velocity
float Navigation::GetVelocity(float dist_remaining) {
  float dist_remaining_latency_accomodated = dist_remaining - prev_velocity * SYSTEM_LATENCY;
  float velocity_after_accelerating = min(MAX_VELOCITY, prev_velocity + MAX_ACCELERATION * TIME_STEP); // Capped at max velocity
  float dist_traversed_while_decelerating = abs(pow(velocity_after_accelerating, 2) / (2 * MAX_DECELERATION));
  // Accelerate if not at max speed, and there is distance left
  if(prev_velocity < MAX_VELOCITY && dist_remaining_latency_accomodated > dist_traversed_while_decelerating) {
    return velocity_after_accelerating;
  }
  // Cruise if at max speed, and there is distance left
  else if(prev_velocity == MAX_VELOCITY && dist_remaining_latency_accomodated > dist_traversed_while_decelerating) {
    return MAX_VELOCITY;
  }
  // Decelerate if not enough distance left
  else {
    float required_deceleration = dist_remaining_latency_accomodated <= 0 ?
      MAX_DECELERATION :
      -1 * (pow(prev_velocity, 2) / (2 * dist_remaining_latency_accomodated));
    float deceleration = max(MAX_DECELERATION, required_deceleration);
    return max(0.0f, prev_velocity + deceleration * TIME_STEP);
  }
}

// Get the curvature for the path that returns the best score
float Navigation::GetOptimalCurvature(float curvature_increment) {
  float highest_score = -1 * __FLT_MAX__;
  float best_curvature = -1.0;
  for(float i = -1.0; i<=1.0; i+=curvature_increment){
    float score = GetPathScore(i);
    if(score > highest_score){
      highest_score = score;
      best_curvature = i;
    }
  }
  return best_curvature;
}

// Print the free paths that the car can take
void Navigation::VisualizeFreePathLengths() {
  vector<int> colors = {0xffbf00, 0xff7f50, 0xde3163, 0x9fe2bf, 0x40e0d0, 0x6495ed, 0xccccff};
  int colorIndex = 0;
  for(float i = -1.0; i<=1.0; i+=CURVATURE_STEP) {
    visualization::DrawPathOption(i, GetFreePathLength(i), 0, colors[colorIndex % colors.size()], true, local_viz_msg_);
    colorIndex++;
  }
}

// void Navigation::VisualizeFreePathLength(float curvature) {
//   int color = 0xffbf00;
//   printf("Free Path length for curvature %f: %f\n", curvature, GetFreePathLength(curvature));
//   visualization::DrawPathOption(curvature, GetFreePathLength(curvature), 0, color, true, local_viz_msg_);
// }

// Score the path produced by a given curvature based off of its free path length, clearance, and distance to goal
float Navigation::GetPathScore(float curvature) {
  const float fpl_weight = 1.0;
  const float clearance_weight = 0.5;
  const float dist_to_goal_weight = 0.25;

  return GetFreePathLength(curvature) * fpl_weight + GetClearance(curvature) * clearance_weight + GetDistanceToGoal(curvature) * dist_to_goal_weight;
}

// Get the closest obstacle along a path
Vector2f Navigation::GetClosestObstacle(float curvature) {
  float closest_fpl = sensor_range;
  // Initialize closest_obstacle to the end of our sensor range path
  Vector2f closest_obstacle;
  if(abs(curvature) < kEpsilon)
    closest_obstacle = Vector2f(closest_fpl, 0);
  else {
    float r = 1.0 / curvature;
    float theta = max((float) M_2_PI, closest_fpl / r);
    float x = r * cos(theta);
    float y = r - r * sin(theta) * (curvature > 1 ? 1 : -1);
    closest_obstacle = Vector2f(x, y);
  }

  for(int i = 0; i<(int)point_cloud_.size(); i++) {
    Vector2f p = point_cloud_[i];
    if(IsObstacle(p, curvature)) {
      float p_fpl = GetFreePathLengthForPoint(p, curvature);
      if(p_fpl < closest_fpl) {
        closest_fpl = p_fpl;
        closest_obstacle = p;
      }
    }
  }

  return closest_obstacle;
}

// Get free path length to any obstacle
float Navigation::GetFreePathLength(float curvature) {
  Vector2f closest_obstacle = GetClosestObstacle(curvature);
  // std::cout << closest_obstacle << std::endl;
  return GetFreePathLengthForPoint(closest_obstacle, curvature);
}

// Get free path length to a particular point
float Navigation::GetFreePathLengthForPoint(Vector2f p, float curvature) {
  // Straight line case
  if(abs(curvature) <= kEpsilon)
    return p[0] - H;
  else {
    float r = abs(1.0 / curvature);
    float x = p[0];
    // flip y-coordinate if curvature is negative
    float y = curvature < 0 ? -p[1] : p[1];
    float theta = std::atan2(x, r - y);
    float omega = std::atan2(H, r - W);
    float phi = (theta < 0 ? M_2PI + theta : theta) - omega;
    return r * phi;
  }
}

// Get the clearance for a particular path
float Navigation::GetClearance(float curvature) {
  float clearance = __FLT_MAX__;
  Vector2f closest_obstacle = GetClosestObstacle(curvature);
  float closest_obstacle_fpl = GetFreePathLengthForPoint(closest_obstacle, curvature);

  for(int i = 0; i<(int)point_cloud_.size(); i++) {
    Vector2f p = point_cloud_[i];
    float p_fpl = GetFreePathLengthForPoint(p, curvature);

    // Ignore obstacles and points near/past the end of our path
    if(!IsObstacle(p, curvature) && p_fpl > closest_obstacle_fpl - kEpsilon) {
      float p_clearance = GetClearanceForPoint(p, curvature);
      clearance = min(clearance, p_clearance);
    }
  }

  return clearance;
}

// Get the clearance for a particular obstacle
float Navigation::GetClearanceForPoint(Vector2f p, float curvature) {
  // Straight line case
  if(abs(curvature) <= kEpsilon)
    return abs(p[1]) - W;
  else {
    float r = abs(1 / curvature);
    Vector2f c = Vector2f(0, r);
    // Flip y-coordinate if curvature is negative
    Vector2f p_adjusted = Vector2f(p[0], curvature > 0 ? p[1] : -p[1]);
    float r1 = c[1] - W;
    float r2 = sqrt(pow(c[1] + W, 2) + pow(H, 2));
    float point_radius = (p_adjusted - c).norm();

    // Calculate clearance based on whether or not the point is inside or outside of the car's turn
    return point_radius >= r2 ? point_radius - r2 : r1 - point_radius;
  }
}

// Get the closest distance the car will be to our temporary goal
float Navigation::GetDistanceToGoal(float curvature) {
  Vector2f closest_obstacle = GetClosestObstacle(curvature);
  Vector2f closest_point_of_approach = GetClosestPointOfApproach(curvature);
  Vector2f goal = Vector2f(TEMPORARY_GOAL_DIST, 0);
  // An obstacle is between the car and its closest point of approach
  if(GetFreePathLengthForPoint(closest_obstacle, curvature) < GetFreePathLengthForPoint(closest_point_of_approach, curvature))
    return (goal - closest_obstacle).norm();
  // No obstacle, return distance from closest point of approach
  return (goal - closest_point_of_approach).norm();
}

// Get the closest point of approach
Vector2f Navigation::GetClosestPointOfApproach(float curvature) {
  // Straight line case
  if(abs(curvature) <= kEpsilon)
    return Vector2f(0, TEMPORARY_GOAL_DIST);

  float r = 1.0 / curvature;
  Vector2f c = Vector2f(0, r);
  Vector2f goal = Vector2f(TEMPORARY_GOAL_DIST, 0);
  float point_goal_dist = (c - goal).norm() - r;
  float theta = std::atan2(TEMPORARY_GOAL_DIST, r);
  return Vector2f(TEMPORARY_GOAL_DIST - point_goal_dist * sin(theta), point_goal_dist * cos(theta));
}

bool Navigation::IsObstacle(Vector2f p, float curvature){
  // Straight path case
  if(abs(curvature) <= kEpsilon)
    return abs(p[1])<=W && p[0]>0;
  else {
    float r = 1 / curvature;
    Vector2f c = Vector2f(0, abs(r));
    // Flip p y-coordinate if curvature is negative
    Vector2f p_adjusted = Vector2f(p[0], curvature > 0 ? p[1] : -p[1]);
    float r1 = c[1] - W;
    float r2 = sqrt(pow(c[1] + W, 2) + pow(H, 2));
    return (p_adjusted - c).norm() >= r1 && (p_adjusted - c).norm() <= r2;
  }
}

// void Navigation::colorize(){
//   vector<int> colors = {0xeb4034, 0xfcba03, 0x34eb5f, 0x34eb5f, 0xeb34e5, 0x73fc03, 0x03fcf0, 0x6bfc03, 0x03fc8c, 0xc603fc};
//   int colorPaths = 0xb734eb;
//   for(float j=-1.0; j<1.0; j+=CURVATURE_STEP){
//     vector<Vector2f> obstacles = curvature_Obstacles[getIndexFromCurvature(j)];
//     for (int i=0; i<(int)obstacles.size(); i++){
//       visualization::DrawPoint(obstacles[i], colorPaths/*colors[getIndexFromCurvature(j)%5]*/, local_viz_msg_);
//       visualization::DrawPathOption(j, GetFreePathLength(j), ClearanceComputation(j), colorPaths, true, local_viz_msg_);
//     }
//   }
// }

}  // namespace navigation
