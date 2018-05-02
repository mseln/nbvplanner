/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RRTGP_H_
#define RRTGP_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <kdtree/kdtree.h>
#include <nbvplanner/tree.h>
#include <nbvplanner/Node.h>

#include <octomap/octomap.h>
// #include <octomap/OcTree.h>
// #include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// #include <octomap_msgs/BoundingBoxQuery.h>


// #include<octomap/OcTreeBase.h>


#include <pigain/Node.h>
#include <pigain/Query.h>


#define SQ(x) ((x)*(x))
#define SQRT2 0.70711


float CylTest_CapsFirst( const octomap::point3d & pt1,
                         const octomap::point3d & pt2, float lsq, float rsq, const octomap::point3d & pt );

namespace nbvInspection {

class RrtGP : public TreeBase<Eigen::Vector4d>
{
 public:
  typedef Eigen::Vector4d StateVec;

  RrtGP(const ros::NodeHandle& nh);
  ~RrtGP();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void initialize(int actions_taken = 1);
  virtual void iterate(int iterations);
  virtual std::vector<nbvplanner::Node> getBestBranch(std::string targetFrame);
  virtual void clear();
  virtual void memorizeBestBranch();
  void publishNode(Node<StateVec> * node);
  void publishGain(double gain, Eigen::Vector3d position);
  void octomapCallback(const octomap_msgs::Octomap& msg);
  bool collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r);
  std::pair<double, double> gainCubature(StateVec state);
  geometry_msgs::Pose stateVecToPose(StateVec stateVec, std::string targetFrame);
 protected:
  ros::NodeHandle nh_;
  kdtree * kdTree_;
  octomap::OcTree * ot_;
  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int r_ID_; // Unique id for rays
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::vector<double> inspectionThrottleTime_;
  ros::Subscriber octomap_sub_;
  ros::Publisher gain_pub_; 
  ros::ServiceClient gp_query_client_;
};
}

#endif
