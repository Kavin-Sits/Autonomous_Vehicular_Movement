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
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"
#include "visualization/visualization.h"

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
  const float K_1 = 0.3;
  const float K_2 = 0.3;
  const float K_3 = 0.3;
  const float K_4 = 0.3;
  const float dShort = 0.1;
  const float dLong = 0.1;
  const float rangeSTD = 0.5; //placeholder
  const float gammaP = 0.2; // placeholder
  const float NUM_RAYS_SKIPPED = 10;
  const Vector2f kLaserLoc(0.2, 0);
}

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    rays_initialized_(false) {
      vis_msg_ = visualization::NewVisualizationMessage("map", "particle_filter");
    }


void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

//CP4
void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {

                               
  if(!rays_initialized_ && num_ranges!=0){
    // printf("num ranges: %d\n", num_ranges);
    for(int i=0; i<num_ranges; i+=NUM_RAYS_SKIPPED){
      float theta = (angle_max-angle_min)/((float) num_ranges) * i + angle_min;
      // Location of the laser on the robot. Assumes the laser is forward-facing.
      Line2f line = Line2f(range_min * cos(theta) + kLaserLoc[0],range_min * sin(theta) + kLaserLoc[1], range_max * cos(theta) + kLaserLoc[0], range_max * sin(theta) + kLaserLoc[1]);
      // printf("ray values: (%f, %f) to (%f, %f)\n", line.p0[0], line.p0[1], line.p1[0], line.p1[1]);
      rays.push_back(line);
    }
    rays_initialized_ = true;
  }
  
  // printf("printing message %d\n", vis_msg_.ns.at(0));
  vector<Line2f> raysInMapFrame;

  for(Line2f ray: rays){
    Line2f line = BaseLinkToMapFrameForLine(ray, loc, angle);
    // printf("line values: (%f, %f) to (%f, %f)\n", line.p0[0], line.p0[1], line.p1[0], line.p1[1]);
    visualization::DrawLine(line.p0, line.p1, 0x0F0F0F, vis_msg_);
    raysInMapFrame.push_back(line);
  }
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(raysInMapFrame.size());
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); i++) {
    scan[i] = Vector2f(0, 0);
  }
  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for(int j=0; j<(int)(raysInMapFrame.size()); j++) {
    // printf("looking for intersections between point rays at ray %d\n", j);
    Line2f currLine = raysInMapFrame.at(j);
    float minDistance = range_max;
    Vector2f closestPt = currLine.p1;
    for (int i = 0; i < (int) map_.lines.size(); i++) {
      // printf("looking for intersections between point rays at ray %d and line %d\n", j, i);
      const Line2f map_line = map_.lines[i];
    // The Line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
     // Line segment from (1,2) to (3.4).

      // Check for intersections:
      bool intersects = map_line.Intersects(currLine);
      // You can also simultaneously check for intersection, and return the point
      // of intersection:
      Vector2f intersection_point; // Return variable
      intersects = map_line.Intersection(currLine, &intersection_point);
      if (intersects) {
        float intersectDistance = (intersection_point-loc).norm();
        if (intersectDistance < minDistance){
          minDistance = intersectDistance;
          closestPt = intersection_point;
        }
        // printf("Intersects at %f,%f\n", intersection_point.x(), intersection_point.y());
      } else {
        // printf("No intersection\n");
      }
    }
    scan[j] = closestPt;
  }
}

// Observation likelihood function: p(s|x) <-x = predict scan, s= observation
// s hat = predicted scan
// gamma of 1/5 to 5?


Vector2f ParticleFilter::BaseLinkToMapFrameForPoint(Vector2f linePt, Vector2f pLoc, float theta){
  Eigen::Rotation2Df thetaRotation(theta);
  Eigen::Matrix2f rot = thetaRotation.toRotationMatrix();
  Vector2f pWorld = rot * linePt + pLoc;
  return pWorld;
}

Line2f ParticleFilter::BaseLinkToMapFrameForLine(Line2f line, Vector2f pLoc, float theta){
  Vector2f p0 = Vector2f(line.p0.x(), line.p0.y());
  Vector2f p1 = Vector2f(line.p1.x(), line.p1.y());

  // printf("old p0: (%f, %f) and p1: (%f, %f)\n", line.p0.x(), line.p0.y(), line.p1.x(), line.p1.y());
  p0 = BaseLinkToMapFrameForPoint(p0, pLoc, theta);
  p1 = BaseLinkToMapFrameForPoint(p1, pLoc, theta);
  // printf("new p0: (%f, %f) and p1: (%f, %f)\n", p0.x(), p0.y(), p1.x(), p1.y());

  return Line2f(p0[0], p0[1], p1[0], p1[1]);
}

//CP4: called for every particle
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

  vector<Vector2f> observed_point_cloud_;
  // The LaserScan parameters are accessible as follows:
  // msg.angle_increment // Angular increment between subsequent rays
  // msg.angle_max // Angle of the last ray
  // msg.angle_min // Angle of the first ray
  // msg.range_max // Maximum observable range
  // msg.range_min // Minimum observable range
  // msg.ranges[i] // The range of the i'th ray
  // printf("Max: %f, Min: %f \n", msg.angle_max, msg.angle_min);
  float angle_increment = (angle_max-angle_min)/ranges.size();
  // printf("observed ranges size: %d", (int)ranges.size());
  for(int i=0; i<(int)ranges.size(); i+=NUM_RAYS_SKIPPED){
    float range = ranges[i];
    if (range>range_max){
      range = range_max;
    }
    else if (range<range_min){
      range = range_min;
    }
    Vector2f pointInBaseLink = Vector2f(range*cos(angle_increment*i + angle_min)+kLaserLoc[0], range*sin(angle_increment*i + angle_min)+kLaserLoc[1]);
    Vector2f pointInMapFrame = BaseLinkToMapFrameForPoint(pointInBaseLink, p_ptr->loc, p_ptr->angle);
    observed_point_cloud_.push_back(pointInMapFrame);
  }
  
  vector<Vector2f> predictedPtCloud;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), range_min, range_max, angle_min, angle_max, &predictedPtCloud);
  float weight = 0;

  Vector2f laserCoordParticle = Vector2f(p_ptr->loc.x() + cos(p_ptr->angle), p_ptr->loc.y() + sin(p_ptr->angle));

  // printf("Size of observed point cloud: %d, size of predicted point cloud: %d\n", (int)observed_point_cloud_.size(), (int)predictedPtCloud.size());
  for(int i=0; i<(int)observed_point_cloud_.size(); i++){
    float predictedRange = (predictedPtCloud.at(i)-laserCoordParticle).norm();
    if (ranges[i] < range_min || ranges[i] > range_max){
      weight += 0;
    }
    else if (ranges[i] < predictedRange - dShort)
    {
      // printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
      weight += -((dShort * dShort)/(rangeSTD * rangeSTD));
      /* code */
    }
    else if (ranges[i] > predictedRange + dLong)
    {
      /* code */
      // printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
      weight += -((dLong * dLong)/(rangeSTD * rangeSTD));
    }
    else{
      // printf("cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
      // printf("observed point: (%f,%f), predicted point: (%f, %f)\n", observed_point_cloud_.at(i).x(), observed_point_cloud_.at(i).y(), predictedPtCloud.at(i).x(), predictedPtCloud.at(i).y());
    weight += -(observed_point_cloud_.at(i)-predictedPtCloud.at(i)).squaredNorm()/(rangeSTD*rangeSTD);
    }
  }

  p_ptr->weight = gammaP * weight;
  observed_point_cloud_store = observed_point_cloud_;
}

//MS2
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
  float weightSum = 0;
  std::vector<Particle> newParticles;

  for(int i=0; i<(int)FLAGS_num_particles; i++){
    weightSum += particles_[i].weight;
  }

  float x = rng_.UniformRandom(0, weightSum);
  for(int i=0; i<(int) particles_.size(); i++){
    x += weightSum / (float) particles_.size();
    if (x>weightSum){
      x -= weightSum;
    }
    float newW = 0;
    for(int j=0; j<(int)FLAGS_num_particles; j++){
      newW += particles_[j].weight;
      if (newW > x){
        newParticles.push_back(particles_[j]);
        break;
      }
    }
    if ((int)newParticles.size() < i + 1){
      newParticles.push_back(particles_[0]);
    }
  }
  particles_ = newParticles;

}

//Loop through particles and call update
void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  for(int i=0; i<(int)particles_.size(); i++){
    Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[i]);
  }

  // Particle p = {Vector2f(21.85,10.25), M_PI, 1};
  // Particle p2 = {Vector2f(21.85,10.25), 0, 1};
  // Update(ranges, range_min, range_max, angle_min, angle_max, &p);
  // Update(ranges, range_min, range_max, angle_min, angle_max, &p2);
  // printf("p weight is %f and p2 weight is %f\n", p.weight, p2.weight);

  NormalizeLogLikelihood();
  Resample();
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
    // printf("Odom Angle %f and Odom Loc (%f, %f)\n", odom_angle, odom_loc[0], odom_loc[1]);
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
  }

  for(int i=0; i<FLAGS_num_particles; i++){
    Particle currentParticle = particles_.at(i);

    Eigen::Matrix3f t1Matrix;
    Eigen::Rotation2Df t1Angle(prev_odom_angle_);
    Eigen::Matrix2f t1RotationMatrix = t1Angle.toRotationMatrix();
    t1Matrix << t1RotationMatrix(0,0), t1RotationMatrix(0,1), prev_odom_loc_[0],
    t1RotationMatrix(1,0), t1RotationMatrix(1,1), prev_odom_loc_[1],
    0, 0, 1;
    Eigen::Matrix3f t1InverseMatrix = t1Matrix.inverse();

    Eigen::Matrix3f t2Matrix;
    Eigen::Rotation2Df t2Angle(odom_angle);
    Eigen::Matrix2f t2RotationMatrix = t2Angle.toRotationMatrix();
    t2Matrix << t2RotationMatrix(0,0), t2RotationMatrix(0,1), odom_loc[0],
    t2RotationMatrix(1,0), t1RotationMatrix(1,1), odom_loc[1],
    0, 0, 1;

    Eigen::Matrix3f t1MapMatrix;
    Eigen::Rotation2Df t1MapAngle(currentParticle.angle);
    Eigen::Matrix2f t1MapRotationMatrix = t1MapAngle.toRotationMatrix();
    t1MapMatrix << t1MapRotationMatrix(0,0), t1MapRotationMatrix(0,1), currentParticle.loc[0],
    t1MapRotationMatrix(1,0), t1MapRotationMatrix(1,1), currentParticle.loc[1],
    0, 0, 1;
    

    Eigen::Matrix3f solutionMatrix = t1MapMatrix * t1InverseMatrix * t2Matrix;

    float mapX = solutionMatrix(0,2);
    float mapY = solutionMatrix(1,2);
    float deltaX = mapX - currentParticle.loc[0];
    float deltaY = mapY - currentParticle.loc[1];
    float deltaTheta = math_util::AngleMod(odom_angle - prev_odom_angle_);

    // if(abs(deltaX) > 1) deltaX = 0;
    // if(abs(deltaY) > 1) deltaY = 0;

    // printf("The deltas: x: %f, y:%f, theta:%f\n", deltaX, deltaY, deltaTheta);
    
    float epsilonX = rng_.Gaussian(0, K_1 * sqrt(deltaX * deltaX + deltaY * deltaY) + K_2 * abs(deltaTheta));
    float epsilonY = rng_.Gaussian(0, K_1 * sqrt(deltaX * deltaX + deltaY * deltaY) + K_2 * abs(deltaTheta));
    float epsilonTheta = rng_.Gaussian(0, K_3 * sqrt(deltaX * deltaX + deltaY * deltaY) + K_4 * abs(deltaTheta));

    // printf("The epsilons: x: %f, y:%f, theta:%f\n", epsilonX, epsilonY, epsilonTheta);

    currentParticle.loc[0] += deltaX + epsilonX;
    currentParticle.loc[1] += deltaY + epsilonY;
    currentParticle.angle += deltaTheta + epsilonTheta;

    particles_[i] = currentParticle;

  }


  // printf("Odom Angle %f and Odom Loc (%f, %f)\n", odom_angle, odom_loc[0], odom_loc[1]);

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
  printf("Particles Cleared\n\n");
  
  for(int i = 0; i<FLAGS_num_particles; i++){
    float x = rng_.Gaussian(loc[0], 0.01);
    float y = rng_.Gaussian(loc[1], 0.01);
    float theta = rng_.Gaussian(angle, 0.01);
    Particle p = {Vector2f(x,y), theta, 1};
    particles_.push_back(p);
  }

  map_.Load(map_file);
}

//change to weighted mean
void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  // float xsum = 0;
  // float ysum = 0;
  // float angleSum = 0;
  // for(int i=0; i<FLAGS_num_particles; i++){
  //   xsum+=particles_[i].loc[0];
  //   ysum+=particles_[i].loc[1];
  //   angleSum+=particles_[i].angle;
  // }
  // loc = Vector2f(xsum / FLAGS_num_particles, ysum / FLAGS_num_particles);
  // angle = angleSum / FLAGS_num_particles;

  particle_filter::Particle maxParticle = particles_[0];
  float maxWeight = maxParticle.weight;
  for (const particle_filter::Particle& p : particles_) {
     printf("Particle weight later: %f\n", p.weight);
    if (p.weight>maxWeight){
      maxWeight = p.weight;
      maxParticle = p;
    }
  }

  printf("__________________________\n");

  loc = maxParticle.loc;
  angle = maxParticle.angle;
}

void ParticleFilter::NormalizeLogLikelihood(){
  float maxWeight = particles_[0].weight;
  for(int i=0; i<(int)particles_.size(); i++){
    if (particles_[i].weight > maxWeight){
      maxWeight = particles_[i].weight;
    }
  }
  for (int i=0; i<(int)particles_.size(); i++){
    particles_[i].weight = exp(particles_[i].weight - maxWeight); 
  }
}

}  // namespace particle_filter
