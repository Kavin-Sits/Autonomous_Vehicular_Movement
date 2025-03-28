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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"
#include "visualization/visualization.h"
#include "shared/util/treeNode.h"
#include "shared/util/random.h"
#include "shared/math/line2d.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

using std::vector;
using Eigen::Vector2f;
using geometry::Line2f;

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void Initialize(const std::string& map_file,
                                const Eigen::Vector2f& loc,
                                const float angle);

  float GetLaunchDistError(float v, float a, float b, float c);

  float GetLaunchDist(float v);

  float GetLaunchVelocity(float targetDist);
  
  float GetVelocity(float dist_remaining);

  float GetOptimalCurvature(float angleIncrement);

  float GetPathScore(float curvature);

  Eigen::Vector2f GetClosestObstacle(float curvature);

  float GetFreePathLength(float curvature);

  float GetFreePathLengthForPoint(Eigen::Vector2f p, float curvature);

  float GetClearance(float curvature);

  float GetClearanceForPoint(Eigen::Vector2f p, float curvature);

  float GetDistanceToGoal(float curvature);
  
  Eigen::Vector2f GetClosestPointOfApproach(float curvature);

  void setLocalTarget();

  Vector2f BaseLinkToMapFrameForPoint(Vector2f linePt, Vector2f robotLoc, float theta);

  Vector2f MapFrameToBaseLinkForPoint(Vector2f linePt, Vector2f robotLoc, float theta);

  bool IsObstacle(Eigen::Vector2f p, float curvature);

  bool createPath(TreeNode* startNode);

  void createCSpace();

  Eigen::Vector2f sampleState();

  TreeNode* nearestNeighbor(Vector2f target);

  Vector2f steer(Vector2f xRand, Vector2f xNear);

  bool isCollisionFree(Vector2f xNear, Vector2f xNew);

  void visualizeTree(TreeNode* startNode);

  bool visualizePath(TreeNode* startNode);

  bool detectObstacles(Eigen::Vector2f p, float curvature);

  std::vector<std::vector<Eigen::Vector2f>> populateCurvatureObstacles();

  std::vector<Eigen::Vector2f> getObstacleForCurvature(float curvature);

  int getIndexFromCurvature(float curvature);

  void VisualizeFreePathLengths();

  void VisualizeFreePathLength(float curvature);

  float prev_velocity;
  float temp_goal_dist;
  float remaining_dist;
  float obstacle_margin;
  float produced_curvature;
  float sensor_range;
  float maxX;
  float minX;
  float maxY;
  float minY;
  bool pathReady;
  Eigen::Vector2f target;
  Eigen::Vector2f localTarget;
  TreeNode* root;
  std::vector<TreeNode*> allNodes;
  std::vector<std::vector<Eigen::Vector2f>> curvature_Obstacles;
  std::vector<Circle> cSpaceCircles;
  std::vector<Rectangle> cSpaceRectangles;
  vector<Line2f> path;

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Random number generator.
  util_random::Random rng_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
