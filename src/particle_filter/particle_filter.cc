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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::Line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace {
  const float K_1 = 0.05;
  const float K_2 = 0.05;
  const float K_3 = 0.05;
  const float K_4 = 0.05;
}

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    /*
    const Line2f map_line = map_.lines[i];
    // The Line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    Line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
    */
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float deltaX = odom_loc[0] - prev_odom_loc_[0];
  // float deltaY = odom_loc[1] - prev_odom_loc_[1];
  // float deltaTheta = odom_angle - prev_odom_angle_;

  // float epsilonXorY = rng_.Gaussian(0, K_1 * sqrt(deltaX * deltaX + deltaY * deltaY) + K_2 * abs(deltaTheta));
  // float epsilonTheta = rng_.Gaussian(0, K_3 * sqrt(deltaX * deltaX + deltaY * deltaY) + K_4 * abs(deltaTheta));

  // Update particle pose by movement change
  // Sample errors ex, ey, ethetha from normal distributions
  // Add errors to the particle pose
  if(!odom_initialized_){
    printf("Odom Angle %f and Odom Loc (%f, %f)\n", odom_angle, odom_loc[0], odom_loc[1]);
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
  }

  printf("Odom Angle %f and Odom Loc (%f, %f)\n", odom_angle, odom_loc[0], odom_loc[1]);

  for(int i=0; i<FLAGS_num_particles; i++){
    Particle particleInit = particles_.at(i);

    Eigen::Rotation2Df r1(prev_odom_angle_);
    Eigen::Matrix2f m1 = r1.toRotationMatrix();
    // printf("Prev Odom Angle %f\n", prev_odom_angle_);
    Eigen::Matrix3d aRobotT1Matrix;
    aRobotT1Matrix << m1(0,0), m1(0,1), prev_odom_loc_[0],
      m1(1,0), m1(1,1), prev_odom_loc_[1],
      0, 0, 1;

    Eigen::Rotation2Df r2(odom_angle);
    Eigen::Matrix2f m2 = r2.toRotationMatrix();
    // printf("Odom Angle %f\n", odom_angle);
    Eigen::Matrix3d aRobotT2Matrix;
    aRobotT2Matrix << m2(0,0), m2(0,1), odom_loc[0],
      m2(0,0), m2(0,1), odom_loc[1],
      0, 0, 1;
    // printf("T1 Matrix\n");
    // cout << "\nt1 matrix:\n" <<  aRobotT1Matrix << endl;

    // printf("T2 Matrix\n");
    // cout << "\nt2 matrix:\n" << aRobotT2Matrix << endl;

    Eigen::Matrix3d resultantMatrix = aRobotT1Matrix.inverse() * aRobotT2Matrix;
    // printf("Resultant Matrix\n");
    // cout << "\nfinal showing :\n" << resultantMatrix << endl;
    // printf("what is this value: %f", resultantMatrix(0,2));
    // printf("%f, %f\n", resultantMatrix(0,2), resultantMatrix(1,2));
    
    float deltaX = resultantMatrix(0, 2);
    float deltaY = resultantMatrix(1, 2);
    float deltaTheta = odom_angle - prev_odom_angle_;

    float epsilonX = 0;//rng_.Gaussian(0, K_1 * sqrt(pow(deltaX, 2) + pow(deltaY, 2)) + K_2 * abs(deltaTheta));
    float epsilonY = 0;//rng_.Gaussian(0, K_1 * sqrt(pow(deltaX, 2) + pow(deltaY, 2)) + K_2 * abs(deltaTheta));
    float epsilonTheta = 0;//rng_.Gaussian(0, K_3 * sqrt(pow(deltaX, 2) + pow(deltaY, 2)) + K_4 * abs(deltaTheta));

    particleInit.loc[0] += epsilonX + deltaX;
    particleInit.loc[1] += epsilonY + deltaY;
    particleInit.angle += epsilonTheta + deltaTheta;
    particles_[i] = particleInit; // Maybe don't need this
  }

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;

}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  printf("Initial Loc is (%f, %f) and initial angle is %f", loc[0], loc[1], angle);
  prev_odom_loc_= Vector2f(0, 0);
  prev_odom_angle_ = 0;
  // odom_initialized_ = true;
  odom_initialized_ = false;

  particles_.clear();
  
  for(int i = 0; i<FLAGS_num_particles; i++){
    float x = rng_.Gaussian(loc[0], 0.01);
    float y = rng_.Gaussian(loc[1], 0.01);
    float theta = rng_.Gaussian(angle, 0.01);
    Particle p = {Vector2f(x,y), theta, 1};
    particles_.push_back(p);
  }

  map_.Load(map_file);

  //don't need the map for cp3
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  float xsum = 0;
  float ysum = 0;
  float angleSum = 0;
  for(int i=0; i<FLAGS_num_particles; i++){
    xsum+=particles_[i].loc[0];
    ysum+=particles_[i].loc[1];
    angleSum+=particles_[i].angle;
  }
  loc = Vector2f(xsum / FLAGS_num_particles, ysum / FLAGS_num_particles);
  angle = angleSum / FLAGS_num_particles;
}


}  // namespace particle_filter
