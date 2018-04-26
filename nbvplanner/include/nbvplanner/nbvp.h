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

#ifndef NBVP_H_
#define NBVP_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>
#include <multiagent_collision_check/Segment.h>
#include <nbvplanner/nbvp_srv.h>
#include <nbvplanner/mesh_structure.h>
#include <nbvplanner/tree.hpp>
#include <nbvplanner/rrtgp.h>

#include <nbvplanner/nbvpAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

template<typename stateVec>
class nbvPlanner
{

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber posClient_;
  ros::Publisher  position_pub_;
  ros::Subscriber pointcloud_sub_;

  Params params_;
  volumetric_mapping::OctomapManager * manager_;

  actionlib::SimpleActionServer<nbvplanner::nbvpAction> as_;
  nbvplanner::nbvpFeedback feedback_;
  nbvplanner::nbvpResult result_;

  bool ready_;

 public:
  typedef std::vector<stateVec> vector_t;
  TreeBase<stateVec> * tree_;

  nbvPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~nbvPlanner();
  bool setParams();
  void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void execute(const nbvplanner::nbvpGoalConstPtr& goal);
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
};
}

#endif // NBVP_H_
