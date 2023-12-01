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
const float MAX_VEL = 1;
const float MAX_ACC = 3;
const float MAX_DEC = -3;

const float OBSTACLE_MARGIN = 0.1;

const float CAR_LEN = 0.4;
const float LOCAL_GOAL_RADIUS = 1;
const float H = 0.5;
const float W = 0.25;

const float TEMPORARY_GOAL_DIST = 10; // carrot on a stick target
// meters^-1
const float CURVATURE_STEP = 0.2;

const float ANGLE_INC = 0.2;
const float MAX_BRANCH_LENGTH = 0.5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    prev_velocity(0),
    temp_goal_dist(0),
    remaining_dist(FLAGS_cp1_distance),
    obstacle_margin(0.1),
    produced_curvature(FLAGS_cp2_curvature),
    sensor_range(0.0),
    maxX(0.0),
    minX(__FLT_MAX__),
    maxY(0.0),
    minY(__FLT_MAX__),
    pathReady(false),
    target(0,0),
    localTarget(0,0),
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0){

  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  createCSpace();
  root = new TreeNode();
}

void Navigation::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.

    allNodes.clear();
    root = new TreeNode(loc);
    allNodes.push_back(root);
    printf("root has been set\n");
    printf("root has been set to position (%f, %f)\n", loc.x(), loc.y());

}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  target = loc;

  allNodes.clear();
  root = new TreeNode(robot_loc_);
  allNodes.push_back(root);

  bool reachedGoal = false;
  int barrier = 1000000;
  int i = 0;
  while(!reachedGoal && i<barrier){
    Vector2f xRand = sampleState();
    TreeNode* xNear = nearestNeighbor(xRand);
    Vector2f xNew = steer(xRand, xNear->point);
    if (isCollisionFree(xNear->point, xNew)){
      if((xNew-loc).norm()<=kEpsilon) reachedGoal = true;
      TreeNode* nextNode = new TreeNode(xNew);
      allNodes.push_back(nextNode);
      xNear->addChild(nextNode);
    }
    i++;
  }

  path.clear();
  createPath(root);
  pathReady = true;
  // printf("size of tree %d\n", (int)allNodes.size());

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

  // if(!pathReady || (robot_loc_-target).norm()<=LOCAL_GOAL_RADIUS){
  //   drive_msg_.curvature =0;
  //   drive_msg_.velocity = 0;

  //   // Add timestamps to all messages.
  //   local_viz_msg_.header.stamp = ros::Time::now();
  //   global_viz_msg_.header.stamp = ros::Time::now();
  //   drive_msg_.header.stamp = ros::Time::now();
  //   // Publish messages.
  //   viz_pub_.publish(local_viz_msg_);
  //   viz_pub_.publish(global_viz_msg_);
  //   drive_pub_.publish(drive_msg_);

  //   return;
  // }

  // for(Circle c: cSpaceCircles){
  //   visualization::DrawCircle(c, 0x00539C, global_viz_msg_);
  // }

  // for(Rectangle r: cSpaceRectangles){
  //   visualization::DrawRectangle(r, 0x00539C, global_viz_msg_);
  // }

  // Circle car = Circle(robot_loc_, LOCAL_GOAL_RADIUS);
  // visualization::DrawCircle(car, 0xFF0000, global_viz_msg_);

  // visualizeTree(root);
  // visualizePath(root);
  // // for(TreeNode* t: allNodes){
  // //   // printf("point value: (%f,%f)\n", t.point.x(), t.point.y());
  // //   visualization::DrawParticle(t->point, 0, local_viz_msg_);
  // // }

  // // If odometry has not been initialized, we can't do anything.
  // if (!odom_initialized_) return;
  // // float t_start = GetMonotonicTime();
  // // The control iteration goes here. 
  // // Feel free to make helper functions to structure the control appropriately.

  // // The latest observed point cloud is accessible via "point_cloud_"
  // // printf("starting\n\n");
  // /* Instantiating values related to obstacles and curvature
  // ___________________________________________*/
  // // curvature_Obstacles = populateCurvatureObstacles(); this
  // // produced_curvature = GetOptimalCurvature(ANGLE_INC); this
  // // float freePathLength = GetFreePathLength(produced_curvature);this


  // produced_curvature = GetOptimalCurvature(CURVATURE_STEP);
  // float freePathLength = GetFreePathLength(produced_curvature);
  // VisualizeFreePathLengths();
  // // VisualizeFreePathLength(0.8);
  // printf("\nFree path length: %f\n", freePathLength);
  // remaining_dist = freePathLength;

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = 0;//produced_curvature;
  drive_msg_.velocity = 4.5;//GetVelocity(remaining_dist);
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
  // float t_end = GetMonotonicTime();
  // printf("Total time for funtion run: %f\n", t_end - t_start);
}

// Given the distance remaining, handle time optimal control with latency adjustments and return the required velocity
float Navigation::GetVelocity(float dist_remaining) {
  float dist_remaining_latency_accomodated = dist_remaining - prev_velocity * SYSTEM_LATENCY;
  float velocity_after_accelerating = std::min(MAX_VEL, prev_velocity + MAX_ACC * TIME_STEP); // Capped at max velocity
  float dist_traversed_while_decelerating = abs(pow(velocity_after_accelerating, 2) / (2 * MAX_DEC));
  // Accelerate if not at max speed, and there is distance left
  if(prev_velocity < MAX_VEL && dist_remaining_latency_accomodated > dist_traversed_while_decelerating) {
    return velocity_after_accelerating;
  }
  // Cruise if at max speed, and there is distance left
  else if(prev_velocity == MAX_VEL && dist_remaining_latency_accomodated > dist_traversed_while_decelerating) {
    return MAX_VEL;
  }
  // Decelerate if not enough distance left
  else {
    float required_deceleration = dist_remaining_latency_accomodated <= 0 ?
      MAX_DEC :
      -1 * (pow(prev_velocity, 2) / (2 * dist_remaining_latency_accomodated));
    float deceleration = std::max(MAX_DEC, required_deceleration);
    return std::max(0.0f, prev_velocity + deceleration * TIME_STEP);
  }
}

//Determining what curvature to use based on other helper methods
float Navigation::GetOptimalCurvature(float curvature_increment) {
  float highest_score = -1 * __FLT_MAX__;
  float best_curvature = -1.0;

  setLocalTarget();

  for(float i = -1.0; i<=1.0; i+=curvature_increment){
    float score = GetPathScore(i);
    if(score > highest_score){
      highest_score = score;
      best_curvature = i;
    }
  }
  return best_curvature;
}

//Calculating scores using combination of approaches
float Navigation::GetPathScore(float curvature){
  const float fpl_weight = 0.1;
  const float clearance_weight = 0.001;
  const float dist_to_goal_weight = 0.989;

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
    float theta = max((float) M_2PI, closest_fpl / r);
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

//Helper methods for path score calulation
//____________________________________________________________________________________________________________________________________________________
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
  // Vector2f closest_obstacle = GetClosestObstacle(curvature);
  Vector2f closest_point_of_approach = GetClosestPointOfApproach(curvature);
  // visualization::DrawParticleWithColor(closest_point_of_approach, 0, global_viz_msg_, 0x0000FF);
  // // An obstacle is between the car and its closest point of approach
  // if(GetFreePathLengthForPoint(closest_obstacle, curvature) < GetFreePathLengthForPoint(closest_point_of_approach, curvature))
  //   return (localTarget - closest_obstacle).norm();
  // // No obstacle, return distance from closest point of approach
  return -(localTarget - closest_point_of_approach).norm();
}

Vector2f Navigation::GetClosestPointOfApproach(float curvature) {
  // Straight line case
  if(abs(curvature) <= kEpsilon) {
    Vector2f goal_base_link = MapFrameToBaseLinkForPoint(localTarget, robot_loc_, robot_angle_);
    Vector2f closest_point_base = Vector2f(goal_base_link.x(), 0.0);
    return BaseLinkToMapFrameForPoint(closest_point_base, robot_loc_, robot_angle_);
  }

  float r = 1.0 / curvature;
  Vector2f c = BaseLinkToMapFrameForPoint(Vector2f(0, r), robot_loc_, robot_angle_);
  // visualization::DrawParticleWithColor(c, 0, global_viz_msg_, 0x0FF00F);
  float center_goal_dist = (c - localTarget).norm();

  Vector2f direction_to_target = localTarget - c;
  Vector2f normalized_direction = direction_to_target / center_goal_dist;
  Vector2f center_to_closest_point = normalized_direction * abs(r);

  return c + center_to_closest_point;
}

void Navigation::setLocalTarget(){
  Circle car = Circle(robot_loc_, LOCAL_GOAL_RADIUS);
  bool found = false;

  while(!found && pathReady) {
    for(int i=0; i<(int)path.size(); i++){
      vector<Vector2f> intersections = car.intersectionPt(path.at(i));
      if(intersections.size()>0){
        found = true;
        localTarget = intersections[0];
        visualization::DrawParticleWithColor(localTarget, 0, global_viz_msg_, 0x00FF00);
        break;
      }
    }

    if(!found){
      pathReady = false;
      SetNavGoal(target, 0);
    }
  }
}

Vector2f Navigation::BaseLinkToMapFrameForPoint(Vector2f linePt, Vector2f robotLoc, float theta){
  Eigen::Rotation2Df thetaRotation(theta);
  Eigen::Matrix2f rot = thetaRotation.toRotationMatrix();
  Vector2f pWorld = rot * linePt + robotLoc;
  return pWorld;
}

Vector2f Navigation::MapFrameToBaseLinkForPoint(Vector2f linePt, Vector2f robotLoc, float theta){
  linePt -= robotLoc;
  Eigen::Rotation2Df thetaRotation(theta);
  Eigen::Matrix2f rot = thetaRotation.toRotationMatrix();
  return rot * linePt;
  // Eigen::Rotation2Df thetaRotation(theta);
  // Eigen::Matrix2f rot = thetaRotation.toRotationMatrix();
  // Vector2f pWorld = rot * linePt + robotLoc;
  // return pWorld;
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

//____________________________________________________________________________________________________________________________________________________
bool Navigation::createPath(TreeNode* startNode) {
  if ((startNode->point - target).norm() <= kEpsilon){
    return true;
  }
  for(int i=0; i<(int)startNode->children.size(); i++){
    if (createPath(startNode->children[i])){
      path.push_back(Line2f(startNode->point, startNode->children[i]->point));
      return true;
    }
  }
  return false;
}

void Navigation::createCSpace(){
  float r = sqrt(W * W + H * H)/2;
  for (int i = 0; i < (int) map_.lines.size(); i++) {
    Line2f line = map_.lines[i];
    maxX = max(max(line.p1.x(), line.p0.x()), maxX);
    minX = min(min(line.p1.x(), line.p0.x()), minX);
    maxY = max(max(line.p1.y(), line.p0.y()), maxY);
    minY = min(min(line.p1.y(), line.p0.y()), minY);
    Circle boundingCircle0 = Circle(line.p0, r);
    Circle boundingCircle1 = Circle(line.p1, r);
    float dx = (line.p1.x() - line.p0.x());
    float dy = (line.p1.y() - line.p0.y());
    float dxNorm = dx / sqrt(dx * dx + dy * dy);
    float dyNorm = dy / sqrt(dx * dx + dy * dy);
    vector<Vector2f> rectPoints = {
      Vector2f(line.p0.x() - r * dyNorm, line.p0.y() + r * dxNorm),
      Vector2f(line.p1.x() - r * dyNorm, line.p1.y() + r * dxNorm),
      Vector2f(line.p1.x() + r * dyNorm, line.p1.y() - r * dxNorm),
      Vector2f(line.p0.x() + r * dyNorm, line.p0.y() - r * dxNorm)
    };
    Rectangle boundingRectangle = Rectangle(rectPoints);
    cSpaceCircles.push_back(boundingCircle0);
    cSpaceCircles.push_back(boundingCircle1);
    cSpaceRectangles.push_back(boundingRectangle);
  }
}

Vector2f Navigation::sampleState(){
  float goalBias = rng_.UniformRandom(0, 1);
  if(goalBias<=0.05){
    return target;
  }
  Vector2f xRand;
  bool validStateFound = false;
  do {
    xRand = Vector2f(rng_.UniformRandom(minX, maxX), rng_.UniformRandom(minY, maxY));
    validStateFound = true;
    for(int i = 0; i < (int) cSpaceCircles.size(); i++) {
      if(cSpaceCircles[i].containsPoint(xRand)) {
        validStateFound = false;
        break;
      }
    }
    if(validStateFound) {
      for(int i = 0; i < (int) cSpaceRectangles.size(); i++) {
        if(cSpaceRectangles[i].containsPoint(xRand)) {
          validStateFound = false;
          break;
        }
      }
    }
  } while(!validStateFound);
  return xRand;
}

TreeNode* Navigation::nearestNeighbor(Vector2f target){
  TreeNode* desiredNode = root;
  float minSquaredDist = __FLT_MAX__;
  for(int i=0; i<(int) allNodes.size(); i++){
    float currSquaredDist = (target - allNodes[i]->point).squaredNorm();
    if (currSquaredDist < minSquaredDist){
      desiredNode = allNodes[i];
      minSquaredDist = currSquaredDist;
    }
  }

  return desiredNode;
}

Vector2f Navigation::steer(Vector2f xRand, Vector2f xNear){
  float dx = xRand.x() - xNear.x();
  float dy = xRand.y() - xNear.y();

  float dxNorm = dx / sqrt(dx * dx + dy * dy);
  float dyNorm = dy / sqrt(dx * dx + dy * dy);
  float branchLength = min(MAX_BRANCH_LENGTH, (xRand - xNear).norm());

  return Vector2f(xNear.x() + branchLength * dxNorm, xNear.y() + branchLength * dyNorm);
}

bool Navigation::isCollisionFree(Vector2f xNear, Vector2f xNew) {
  Line2f branch = Line2f(xNear.x(), xNear.y(), xNew.x(), xNew.y());
  for (int i = 0; i < (int) cSpaceCircles.size(); i++) {
    if(cSpaceCircles[i].intersectsLine(branch)) return false;
  }
  for (int i = 0; i < (int) cSpaceRectangles.size(); i++) {
    if(cSpaceRectangles[i].intersectsLine(branch)) return false;
  }
  return true;
}

void Navigation::visualizeTree(TreeNode* startNode){

  for(int i=0; i<(int)startNode->children.size(); i++){
    visualization::DrawLine(startNode->point, startNode->children[i]->point, 0xEE4E34, global_viz_msg_);
    visualizeTree(startNode->children[i]);
  }
  return;
}

bool Navigation::visualizePath(TreeNode* startNode){
  if ((startNode->point - target).norm() <= kEpsilon){
    return true;
  }
  for(int i=0; i<(int)startNode->children.size(); i++){
    if (visualizePath(startNode->children[i])){
      visualization::DrawLine(startNode->point, startNode->children[i]->point, 0x79FF23, global_viz_msg_);
      return true;
    }
  }
  return false;
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

// void Navigation::colorize(){
//   vector<int> colors = {0xeb4034, 0xfcba03, 0x34eb5f, 0x34eb5f, 0xeb34e5, 0x73fc03, 0x03fcf0, 0x6bfc03, 0x03fc8c, 0xc603fc};
//   int colorPaths = 0xb734eb;
//   for(float j=-1.0; j<1.0; j+=ANGLE_INC){
//     vector<Vector2f> obstacles = curvature_Obstacles[getIndexFromCurvature(j)];
//     for (int i=0; i<(int)obstacles.size(); i++){
//       visualization::DrawPoint(obstacles[i], colorPaths/*colors[getIndexFromCurvature(j)%5]*/, local_viz_msg_);
//       visualization::DrawPathOption(j, GetFreePathLength(j), ClearanceComputation(j), colorPaths, true, local_viz_msg_);
//     }
//   }
// }

// Print the free paths that the car can take
void Navigation::VisualizeFreePathLengths() {
  vector<int> colors = {0xffbf00, 0xff7f50, 0xde3163, 0x9fe2bf, 0x40e0d0, 0x6495ed, 0xccccff};
  int colorIndex = 0;
  for(float i = -1.0; i<=1.0; i+=CURVATURE_STEP) {
    visualization::DrawPathOption(i, GetFreePathLength(i), 0, colors[colorIndex % colors.size()], true, local_viz_msg_);
    colorIndex++;
  }
}

void Navigation::VisualizeFreePathLength(float curvature) {
  int color = 0xffbf00;
  // printf("Free Path length for curvature %f: %f\n", curvature, GetFreePathLength(curvature));
  visualization::DrawPathOption(curvature, GetFreePathLength(curvature), 0, color, true, local_viz_msg_);
}

// void Navigation::printObstacleList(){
//   printf("\n\n");
//   for(float j=-1.0; j<1.0; j+=ANGLE_INC){
//     vector<Vector2f> obstacles = curvature_Obstacles[getIndexFromCurvature(j)];
//     printf("Obstacle list for curvature %f\n", j);
//     for (int i=0; i<(int)obstacles.size(); i++){
//       printf("(%f,%f) ", obstacles[i][0], obstacles[i][1]);
//     }
//     printf("End of list for curvature %f\n", j);
//   }
//   printf("\n");
// }

}  // namespace navigation
