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
const float MAX_VEL = 1;
const float MAX_ACC = 3;
const float MAX_DEC = -3;
const float CAR_LEN = 0.4;
const float H = 0.5;
const float W = 0.25;
const float ANGLE_INC = 0.05;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    prev_velocity(0),
    remaining_dist(FLAGS_cp1_distance),
    obstacle_margin(0.1),
    produced_curvature(FLAGS_cp2_curvature),
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
  // 1D TOC travel
  // remaining_dist = remaining_dist - (loc - odom_loc_).norm();
  // printf("Remaining Distance %f\n", remaining_dist);
  // printf("total traversal %f\n", (loc-odom_start_loc_).norm());
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
  /* Uncomment section below for visualizations
  ___________________________________________*/
  curvature_Obstacles = populateCurvatureObstacles();
  produced_curvature = GetOptimalCurvature(ANGLE_INC);
  float freePathLength = GetFreePathLength(produced_curvature);
  // colorize();
  printf("\nFree path length: %f\n", freePathLength);
  remaining_dist = freePathLength;
  // printf("ending\n\n");
  /*
  __________________________________________
  */

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = produced_curvature;
  drive_msg_.velocity = Navigation::InstantaneousTimeDecision();
  printf("Speed: %f\n", Navigation::InstantaneousTimeDecision());
  prev_velocity = drive_msg_.velocity;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

float Navigation::InstantaneousTimeDecision(){
  float remaining_dist_latency_accomodated = remaining_dist - prev_velocity * 0.15;
  float velocity_req = prev_velocity+MAX_ACC*0.05;
  if(prev_velocity<MAX_VEL && remaining_dist_latency_accomodated > -pow(std::min(MAX_VEL, velocity_req), 2)/(2*MAX_DEC)){
    float calc_vel = MAX_ACC * 0.05 + prev_velocity;
    return std::min(calc_vel, MAX_VEL);
  }
  else if(prev_velocity==MAX_VEL && remaining_dist_latency_accomodated > -pow(MAX_VEL, 2)/(2*MAX_DEC)){
    return prev_velocity;
  }
  else{
    float decel;
    if(remaining_dist_latency_accomodated<0){
      decel = MAX_DEC;
    } else {
      decel = (-1 * pow(prev_velocity, 2) / (2*remaining_dist_latency_accomodated));
    }
    float decel_adj = std::max(decel,MAX_DEC);
    float vel = decel_adj * 0.05 + prev_velocity;
    return std::max(vel, 0.0f);
  }
}

float Navigation::GetOptimalCurvature(float angleIncrement){
  float highestScore = -1 * __FLT_MAX__;
  float bestCurvature = 0.0;
  for(float i=0.0; i<=1.0; i+=angleIncrement){
    float currentScore = GetPathScore(i);
    float currentOppositeScore = GetPathScore(-1.0 * i);
    if(currentScore>highestScore){
      highestScore = currentScore;
      bestCurvature = i;
    }
    if(currentOppositeScore>highestScore){
      highestScore = currentOppositeScore;
      bestCurvature = -1.0 * i;
    }
    // printf("Free path length: %f and curvature: %f\n", currentScore, i);
    // if(i!=0) printf("Free path length: %f and curvature: %f\n", currentOppositeScore, -i);
  }
  // printf("Highest score: %f and best curvature %f\n", highestScore, bestCurvature);
  return bestCurvature;
}

float Navigation::GetPathScore(float curvature){
  const float weight_1 = 1;
  const float weight_2 = .2;//.52;
  const float weight_3 = 0;//0.05;//.15;

  return GetFreePathLength(curvature) * weight_1 + ClearanceComputation(curvature) * weight_2 + GetClosestPointOfApproach(curvature) * weight_3;
}

float Navigation::GetFreePathLength(float curvature) {
  float freePathLength = sensor_range;
  vector<Vector2f> obstacles = curvature_Obstacles[getIndexFromCurvature(curvature)];
  for (int i=0; i<(int)obstacles.size(); i++){
      float calculatedLength = GetFreePathLengthForPoint(obstacles[i], curvature);
      freePathLength = std::min(calculatedLength, freePathLength); 
  }
  return freePathLength;
}

float Navigation::GetFreePathLengthForPoint(Vector2f p, float curvature) {
  if(abs(curvature) <= kEpsilon){
    return p[0] - H;
  }
  else{
    float r = abs(1.0/curvature);
    float x = p[0];
    float y = curvature < 0 ? -p[1] : p[1];
    float theta = std::atan2(x, r - y);
    float omega = std::atan2(H, r - W);
    float phi = (theta < 0 ? M_2PI + theta : theta) - omega;
    return r * phi;
  }
}

float Navigation::ClearanceComputation(float curvature){
  float minComp = __FLT_MAX__;
  for (int i=0; i<(int)point_cloud_.size(); i++){
    if (!detectObstacles(point_cloud_[i], curvature)){
      minComp = std::min(minComp, ClearanceComputationForPoint(point_cloud_[i], curvature));
    }
  }
  return minComp;
}

float Navigation::ClearanceComputationForPoint(Vector2f p, float curvature){
  if(abs(curvature) <= kEpsilon){
    return abs(p[1]) - W;
  }
  else {
    float radius = abs(1/curvature);
    Vector2f c = Vector2f(0, radius);
    Vector2f pAdj = Vector2f(p[0], curvature > 0 ? p[1] : -p[1]);
    float r1 = c[1] - W;
    float r2 = sqrt(pow((c[1] + W),2) + pow(H,2));
    float pointRadius = (pAdj - c).norm();
    if(pointRadius >= r2) {
      return pointRadius - r2;
    } else {
      return r1 - pointRadius;
    }
  }
}

float Navigation::GetClosestPointOfApproach(float curvature){
  if(abs(curvature)<kEpsilon) return 0;

  float radius = 1/curvature;
  return sqrt(pow(radius,2) + pow(10,2)) - radius;
}

bool Navigation::detectObstacles(Vector2f p, float curvature){
  if(abs(curvature) <= kEpsilon){
    return abs(p[1])<=W && p[0]>0;
  }
  else{
    Vector2f c = Vector2f(0, abs(1/curvature));
    Vector2f pAdj = Vector2f(p[0], curvature > 0 ? p[1] : -p[1]);
    float r1 = c[1] - W;
    float r2 = sqrt(pow((c[1] + W),2) + pow(H,2));
    return ((pAdj - c).norm() >= r1 && (pAdj - c).norm() <= r2);
  }
}

vector<Vector2f> Navigation::getObstacleForCurvature(float curvature){
  vector<Vector2f> obstacle_store;
  for (int i=0; i<(int)point_cloud_.size(); i++){
    if (detectObstacles(point_cloud_[i], curvature)){
      obstacle_store.push_back(point_cloud_[i]);
    }
  }
  return obstacle_store;
}

vector<vector<Vector2f>> Navigation::populateCurvatureObstacles(){
  vector<vector<Vector2f>> obstacle_curvature_store;
  for(float i=-1.0; i<1.0; i+=ANGLE_INC){
    obstacle_curvature_store.push_back(getObstacleForCurvature(i));
  }
  return obstacle_curvature_store;
}

int Navigation::getIndexFromCurvature(float curvature){
  return round((curvature + 1) * (1/ANGLE_INC)); 
}

void Navigation::colorize(){
  vector<int> colors = {0xeb4034, 0xfcba03, 0x34eb5f, 0x34eb5f, 0xeb34e5, 0x73fc03, 0x03fcf0, 0x6bfc03, 0x03fc8c, 0xc603fc};
  int colorPaths = 0xb734eb;
  for(float j=-1.0; j<1.0; j+=ANGLE_INC){
    vector<Vector2f> obstacles = curvature_Obstacles[getIndexFromCurvature(j)];
    for (int i=0; i<(int)obstacles.size(); i++){
      visualization::DrawPoint(obstacles[i], colorPaths/*colors[getIndexFromCurvature(j)%5]*/, local_viz_msg_);
      visualization::DrawPathOption(j, GetFreePathLength(j), ClearanceComputation(j), colorPaths, true, local_viz_msg_);
    }
  }
}

}  // namespace navigation
