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
const float MAX_VEL = 1;
const float MAX_ACC = 3;
const float MAX_DEC = -3;
const float CAR_LEN = 0.4;
const float H = 0.5;
const float W = 0.25;
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
    target(0,0),
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

  printf("size of tree %d\n", (int)allNodes.size());

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

  for(Circle c: cSpaceCircles){
    visualization::DrawCircle(c, 0x0000FF, local_viz_msg_);
  }

  for(Rectangle r: cSpaceRectangles){
    visualization::DrawRectangle(r, 0x0000FF, local_viz_msg_);
  }

  printf("root children size: %d\n", (int) root->children.size());
  if(root->children.size() > 0){
    printf("root has point (%f,%f) and child point (%f, %f)\n", root->point.x(), root->point.y(), root->children[0]->point.x(), root->children[0]->point.y());
    visualization::DrawLine(root->point, root->children[0]->point, 0x0000FF, local_viz_msg_);
  }
  visualizeTree(root);
  visualizePath(root);
  // for(TreeNode* t: allNodes){
  //   // printf("point value: (%f,%f)\n", t.point.x(), t.point.y());
  //   visualization::DrawParticle(t->point, 0, local_viz_msg_);
  // }

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  // float t_start = GetMonotonicTime();
  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.

  // The latest observed point cloud is accessible via "point_cloud_"
  // printf("starting\n\n");
  /* Instantiating values related to obstacles and curvature
  ___________________________________________*/
  // curvature_Obstacles = populateCurvatureObstacles(); this
  // produced_curvature = GetOptimalCurvature(ANGLE_INC); this
  // float freePathLength = GetFreePathLength(produced_curvature);this


  // colorize();
  // visualization::DrawPathOption(produced_curvature, GetFreePathLength(produced_curvature), ClearanceComputation(produced_curvature), 0x34eb5b, true, local_viz_msg_);
  // printf("\nFree path length: %f\n", freePathLength);

  // remaining_dist = freePathLength; this

  // printf("ending\n\n");
  /*
  __________________________________________
  */
  // printObstacleList();

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = 1;//produced_curvature; this
  drive_msg_.velocity = 1;//Navigation::InstantaneousTimeDecision(); this
  // printf("Speed: %f\n", drive_msg_.velocity);
  // printf("Curvature: %f\n", drive_msg_.curvature);
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

//Handling Time Optimal Control with latency adjustments
float Navigation::InstantaneousTimeDecision(){
  float remaining_dist_latency_accomodated = remaining_dist - prev_velocity * 0.3;
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

//Determining what curvature to use based on other helper methods
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

//Calculating scores using combination of approaches
float Navigation::GetPathScore(float curvature){
  const float weight_1 = 0.85;
  const float weight_2 = 0.05;//.52;
  const float weight_3 = 0.10;//0.05;//.15;

  return GetFreePathLength(curvature) * weight_1 + ClearanceComputation(curvature) * weight_2 + GetClosestPointOfApproach(curvature) * weight_3;
}

//Helper methods for path score calulation
//____________________________________________________________________________________________________________________________________________________
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
  // need to hande 0 curvature case
  vector<Vector2f> obstacles = curvature_Obstacles[getIndexFromCurvature(curvature)];
  if(abs(curvature)<kEpsilon) {
    // get the closest obstacle
    Vector2f closestObstacle = obstacles[0];
    for(int i = 0; i<(int)obstacles.size(); i++) {
      float x = obstacles[i][0];
      if(x < closestObstacle[0]) {
        closestObstacle = obstacles[i];
      }
    }

    // ignore clearance candidates past our closest obstacle
    for (int i=0; i<(int)point_cloud_.size(); i++){
      if (!detectObstacles(point_cloud_[i], curvature)){
        float x = point_cloud_[i][0];
        if(x > closestObstacle[0]) continue;
        minComp = std::min(closestObstacle[0] - x, minComp);
      }
    }
    return minComp;
  }

  float r = abs(1/curvature);

  // get theta for the closest obstacle
  float smallestTheta = __FLT_MAX__;
  for (int i=0; i<(int)point_cloud_.size(); i++){
    if (!detectObstacles(point_cloud_[i], curvature)){
      float x = point_cloud_[i][0];
      float y = point_cloud_[i][1];
      // need to handle negative theta
      float theta = std::atan2(x, r - y);
      smallestTheta = std::min(theta, smallestTheta);
    }
  }

  for (int i=0; i<(int)point_cloud_.size(); i++){
    if (!detectObstacles(point_cloud_[i], curvature)){
      float x = point_cloud_[i][0];
      float y = point_cloud_[i][1];
      // need to handle negative theta
      float theta = std::atan2(x, r - y);
      // ignore points that are past our closest obstacle
      if(theta > smallestTheta) {
        continue;
      }
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
  vector<Vector2f> obstacles = curvature_Obstacles[getIndexFromCurvature(curvature)];

  if(abs(curvature)<kEpsilon) {
    Vector2f closestObstacle = obstacles[0];
    for(int i = 0; i<(int)obstacles.size(); i++) {
      float x = obstacles[i][0];
      if(x < closestObstacle[0]) {
        closestObstacle = obstacles[i];
      }
    }
    float x = closestObstacle[0];
    return x > temp_goal_dist ? 0 : (temp_goal_dist - x);
  }

  float radius = abs(1/curvature);
  Vector2f closestObstacle = obstacles[0];
  // handle for if no obstacle exists
  float smallestTheta = __FLT_MAX__;
  for(int i = 0; i<(int)obstacles.size(); i++) {
    float x = obstacles[i][0];
    float y = obstacles[i][1];
    // add error handling for negative angles
    float theta = std::atan2(x, radius - y);
    if(theta < smallestTheta) {
      closestObstacle = obstacles[i];
      smallestTheta = theta;
    }
  }
  // add error handling for negative angles
  float omega = std::atan2(temp_goal_dist, radius);

  // if we have no obstacles before we reach the nearest point
  if(smallestTheta > omega) {
    return sqrt(pow(radius,2) + pow(temp_goal_dist,2)) - radius;
  }

  Vector2f closestPoint = Vector2f(radius * sin(smallestTheta), radius - radius * cos(smallestTheta));
  Vector2f tempGoal = Vector2f(temp_goal_dist, 0);
  float closestDistance = (closestPoint - tempGoal).norm();
  return closestDistance;
}

//____________________________________________________________________________________________________________________________________________________


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
    visualization::DrawLine(startNode->point, startNode->children[i]->point, 0xFF0000, local_viz_msg_);
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
      visualization::DrawLine(startNode->point, startNode->children[i]->point, 0x00FF00, local_viz_msg_);
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
