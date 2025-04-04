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
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"

#include "particle_filter.h"
#include "visualization/visualization.h"

using amrl_msgs::VisualizationMsg;
using geometry::Line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;
using visualization::DrawParticleWithColor;

// Create command line arguements
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");
DEFINE_string(init_topic,
              "/set_pose",
              "Name of ROS topic for initialization");

DECLARE_int32(v);

// Create config reader entries
CONFIG_STRING(map_name_, "map");
CONFIG_FLOAT(init_x_, "init_x");
CONFIG_FLOAT(init_y_, "init_y");
CONFIG_FLOAT(init_r_, "init_r");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

bool run_ = true;
particle_filter::ParticleFilter particle_filter_;
ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
ros::Publisher laser_publisher_;
VisualizationMsg vis_msg_;
amrl_msgs::Localization2DMsg localization_msg_;
sensor_msgs::LaserScan last_laser_msg_;

vector<Vector2f> trajectory_points_;
string current_map_;

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;
  localization_msg_.header = header;
  vis_msg_ = visualization::NewVisualizationMessage("map", "particle_filter");
}

void PublishParticles() {
  vector<particle_filter::Particle> particles;
  particle_filter_.GetParticles(&particles);
  // particle_filter::Particle maxParticle;
  // float maxWeight = 0;

// Visualizing point clouds of specific predicted particles
// particle_filter::Particle p = {Vector2f(0,0), M_PI, 0.01};
// for (int i=0; i<(int)particles.size(); i++) {
//   if (particles[i].weight>p.weight){
//     p = particles[i];
//   }
// }
  // DrawParticleWithColor(p.loc, p.angle, vis_msg_, 0x00FF00);
  // vector<Vector2f> predictedPtCloud;
  // particle_filter_.GetPredictedPointCloud(p.loc, p.angle, 1000, 0.02, 30.0, -2, 2, &predictedPtCloud);
  // for (int i=0; i<(int)predictedPtCloud.size(); i++){
  //   DrawPoint(predictedPtCloud[i], 0x00FF00, vis_msg_);
  // }

  // particle_filter::Particle p2 = {Vector2f(-21.85,10.25), 0, 1};
  // DrawParticleWithColor(p2.loc, p2.angle, vis_msg_, 0x0000FF);
  // particle_filter_.GetPredictedPointCloud(p2.loc, p2.angle, 1033, 0.02, 30.0, -2.2514, 2.2514, &predictedPtCloud);
  // for (int i=0; i<(int)predictedPtCloud.size(); i++){
  //   DrawPoint(predictedPtCloud[i], 0x0000FF, vis_msg_);
  // }
  for (int i=0; i<(int)particle_filter_.observed_point_cloud_store.size(); i++){
    DrawPoint(particle_filter_.observed_point_cloud_store[i], 0x00AAA7, vis_msg_);
  }

  for (int i=0; i<(int)particles.size(); i++) {
    particle_filter::Particle p = particles[i];
    if (i==0){
      DrawParticleWithColor(p.loc, p.angle, vis_msg_, 0x0000FF);
    }
    else if ( i==(int)particles.size()-1)
    {
      DrawParticleWithColor(p.loc, p.angle, vis_msg_, 0x00FF00);
    }
    else{
      DrawParticleWithColor(p.loc, p.angle, vis_msg_, 0xFF0000);
    }
    // if (p.weight>maxWeight){
    //   maxWeight = p.weight;
    //   maxParticle = p;
    // }
  }
  // DrawParticleWithColor(maxParticle.loc, maxParticle.angle, vis_msg_, 0x00FF00);
}

void PublishPredictedScan() {
  // const uint32_t kColor = 0xd67d00;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  vector<Vector2f> predicted_scan;
  particle_filter_.GetPredictedPointCloud(
      robot_loc,
      robot_angle,
      last_laser_msg_.ranges.size(),
      last_laser_msg_.range_min,
      last_laser_msg_.range_max,
      last_laser_msg_.angle_min,
      last_laser_msg_.angle_max,
      &predicted_scan);
  for (const Vector2f& p : predicted_scan) {
    DrawPoint(p, 0x0000FF, vis_msg_);
  }
}

void PublishTrajectory() {
  const uint32_t kColor = 0xadadad;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  static Vector2f last_loc_(0, 0);
  if (!trajectory_points_.empty() &&
      (last_loc_ - robot_loc).squaredNorm() > Sq(1.5)) {
    trajectory_points_.clear();
  }
  if (trajectory_points_.empty() ||
      (robot_loc - last_loc_).squaredNorm() > 0.25) {
    trajectory_points_.push_back(robot_loc);
    last_loc_ = robot_loc;
  }
  for (size_t i = 0; i + 1 < trajectory_points_.size(); ++i) {
    DrawLine(trajectory_points_[i],
             trajectory_points_[i + 1],
             kColor,
             vis_msg_);
  }
}

void PublishVisualization() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.05) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);

  PublishParticles();
  PublishPredictedScan();
  PublishTrajectory();
  visualization_publisher_.publish(vis_msg_);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  last_laser_msg_ = msg;
  particle_filter_.ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max);
  PublishVisualization();
}

void PublishLocation() {
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  localization_msg_.header.stamp = ros::Time::now();
  localization_msg_.map = current_map_;
  localization_msg_.pose.x = robot_loc.x();
  localization_msg_.pose.y = robot_loc.y();
  localization_msg_.pose.theta = robot_angle;
  localization_publisher_.publish(localization_msg_);
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  particle_filter_.ObserveOdometry(odom_loc, odom_angle);
  PublishLocation();
  PublishVisualization();
}

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

void InitCallback(const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f init_loc(msg.pose.x, msg.pose.y);
  const float init_angle = msg.pose.theta;
  current_map_ = msg.map;
  const string map_file = GetMapFileFromName(current_map_);
  printf("Initialize: %s (%f,%f) %f\u00b0\n",
         current_map_.c_str(),
         init_loc.x(),
         init_loc.y(),
         RadToDeg(init_angle));
  particle_filter_.Initialize(map_file, init_loc, init_angle);
  trajectory_points_.clear();
}

void ProcessLive(ros::NodeHandle* n) {
  ros::Subscriber initial_pose_sub = n->subscribe(
      FLAGS_init_topic.c_str(),
      1,
      InitCallback);
  ros::Subscriber laser_sub = n->subscribe(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  ros::Subscriber odom_sub = n->subscribe(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  particle_filter_.Initialize(
      GetMapFileFromName(current_map_),
      Vector2f(CONFIG_init_x_, CONFIG_init_y_),
      DegToRad(CONFIG_init_r_));
  while (ros::ok() && run_) {
    ros::spinOnce();
    PublishVisualization();
    Sleep(0.01);
  }
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "particle_filter", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  InitializeMsgs();
  current_map_ = CONFIG_map_name_;

  visualization_publisher_ =
      n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);
  laser_publisher_ =
      n.advertise<sensor_msgs::LaserScan>("scan", 1);

  ProcessLive(&n);

  return 0;
}
