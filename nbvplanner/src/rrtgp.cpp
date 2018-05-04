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

#ifndef RRTGP_HPP_
#define RRTGP_HPP_

#include <cstdlib>
#include <nbvplanner/rrtgp.h>
#include <nbvplanner/tree.hpp>

nbvInspection::RrtGP::RrtGP(const ros::NodeHandle& nh) :
  nh_(nh),
  gain_pub_(nh_.advertise<pigain::Node>("/gain_node", 1000)),
  gp_query_client_(nh_.serviceClient<pigain::Query>("/gp_query_server")),
  octomap_sub_(nh_.subscribe("octomap", 1, &nbvInspection::RrtGP::octomapCallback, this))
{
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  ot_ = new octomap::OcTree(1);  // Create dummy OcTree to prevent crash due to ot_ tree not initialized
}

nbvInspection::RrtGP::~RrtGP()
{
  delete rootNode_;
  kd_free(kdTree_);
}

void nbvInspection::RrtGP::setStateFromPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);
}

void nbvInspection::RrtGP::iterate(int iterations)
{
  ros::Time start_time = ros::Time::now();
  // ROS_INFO_STREAM("Starting iterate: " << ros::Time::now() - start_time);
// In this function a new configuration is sampled and added to the tree.
  StateVec newState;

// Sample over a sphere with the radius of the maximum diagonal of the exploration
// space. Throw away samples outside the sampling region it exiting is not allowed
// by the corresponding parameter. This method is to not bias the tree towards the
// center of the exploration space.
  double radius = sqrt(
      SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
      + SQ(params_.minZ_ - params_.maxZ_));

  // ROS_INFO_STREAM("Sampling point... " << ros::Time::now() - start_time);
  bool solutionFound = false;
  while (!solutionFound) {
    for (int i = 0; i < 3; i++) {
      newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
      continue;
    // Offset new state by root
    newState += rootNode_->state_;
    if (!params_.softBounds_) {
      if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
        continue;
      } else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
        continue;
      }
    }
    solutionFound = true;
  }
  // ROS_INFO_STREAM("Found point... " << ros::Time::now() - start_time);
  // ROS_INFO_STREAM("Finding neighbour... " << ros::Time::now() - start_time);

// Find nearest neighbour
  kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }
  nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
      nearest);
  kd_res_free(nearest);

  // ROS_INFO_STREAM("Found neighbour... " << ros::Time::now() - start_time);
  // ROS_INFO_STREAM("Collision checking... " << ros::Time::now() - start_time);

  // Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                            newState[2] - origin[2]);
  if (direction.norm() > params_.extensionRange_) {
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];

  octomap::point3d query(newState[0], newState[1], newState[2]);
  octomap::OcTreeNode* result = ot_->search (query);

  if(result){
    if(!collisionLine(origin, origin + direction, 0.5)) {
      // ROS_INFO_STREAM("Collision check ok... " << ros::Time::now() - start_time);
      // ROS_INFO_STREAM("Calculating gain... " << ros::Time::now() - start_time);
      // Sample the new orientation
      // newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
      std::pair<double, double> ret = gainCubature(newState);
      newState[3] = ret.second; // Set angle to angle with highest information gain

      // Create new node and insert into tree
      nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ = newParent->gain_
          + ret.first * exp(-params_.degressiveCoeff_ * newNode->distance_);

      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      // ROS_INFO_STREAM("Calculated gain... " << ros::Time::now() - start_time);

      // Display new node
      publishNode(newNode);

      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
    else {
      // ROS_INFO_STREAM("Collision check fail... " << ros::Time::now() - start_time);
    }
  }
  else{
    // ROS_INFO_STREAM("New pose in unknown area... " << ros::Time::now() - start_time);
  }
  // ROS_INFO_STREAM("Done! " << ros::Time::now() - start_time);
}

void nbvInspection::RrtGP::initialize(int actions_taken)
{
// This function is to initialize the tree, including insertion of remainder of previous best branch.
  g_ID_ = 0;
// Initialize kd-tree with root node
  kdTree_ = kd_create(3);

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = NULL;

  if (bestBranchMemory_.size()) {
    exact_root_ = bestBranchMemory_[bestBranchMemory_.size()-actions_taken];
    rootNode_->state_ = exact_root_;
  } else {
    exact_root_ = root_;
    rootNode_->state_ = root_;
  }
  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(),
             rootNode_);
  iterationCount_++;

  // Insert all nodes of the remainder of the previous best branch, checking for collisions and
  // recomputing the gain.
  for (int i = bestBranchMemory_.size()-actions_taken-1; i >= 0; --i) {
    StateVec newState = bestBranchMemory_[i];
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
        nearest);
    kd_res_free(nearest);

    // Check for collision
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                              newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];

    if(!collisionLine(origin, origin + direction, 0.5)) {
      std::pair<double, double> ret = gainCubature(newState);
      newState[3] = ret.second; // Set angle to angle with highest information gain

      // Create new node and insert into tree
      nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ = newParent->gain_
          + ret.first * exp(-params_.degressiveCoeff_ * newNode->distance_);

      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      // Display new node
      publishNode(newNode);

      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }

  // Publish visualization of total exploration area
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
  p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
  p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = params_.maxX_ - params_.minX_;
  p.scale.y = params_.maxY_ - params_.minY_;
  p.scale.z = params_.maxZ_ - params_.minZ_;
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

std::vector<nbvplanner::Node> nbvInspection::RrtGP::getBestBranch(std::string targetFrame)
{
// This function returns the best branch
  std::vector<nbvplanner::Node> ret;
  nbvInspection::Node<StateVec> * current = bestNode_;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      nbvplanner::Node node;
      node.pose = stateVecToPose(current->state_, targetFrame);
      node.gain = current->gain_;
      ret.push_back(node);
      current = current->parent_;
    }
    nbvplanner::Node node;
    node.pose = stateVecToPose(current->state_, targetFrame);
    node.gain = current->gain_;
    ret.push_back(node);
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

std::pair<double, double> nbvInspection::RrtGP::gainCubature(StateVec state)
{
  double gain = 0.0;

  pigain::Query srv;
  srv.request.point.x = state[0];
  srv.request.point.y = state[1];
  srv.request.point.z = state[2];

  // ROS_INFO_STREAM("Calling gp_query_service");
  if (gp_query_client_.call(srv)) {
    // ROS_INFO_STREAM("mu = " << srv.response.mu << " sigma = " << srv.response.sigma);
    if(srv.response.sigma < 0.2){
      gain = srv.response.mu;
      double yaw = 0;
      return std::make_pair(gain, yaw);
    }
    else{
      // ROS_WARN_STREAM("Sigma too high, calculating gain explicitly");
    }
  }
  else {
    ROS_WARN("Failed to call gp_query_service");
  }

  // This function computes the gain
  // TODO Parameterize
  int n_rays = 1000;
  double fov_y = 56, fov_p = 42;

  double dr = 0.05, dphi = 10, dtheta = 10;
  double dphi_rad = M_PI*dphi/180.0f, dtheta_rad = M_PI*dtheta/180.0f;
  double r; int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec, dir;

  int id=0;
  for(theta = -180; theta < 180; theta += dtheta){
    theta_rad = M_PI*theta/180.0f;
    for(phi = 90 - fov_p/2; phi < 90 + fov_p/2; phi += dphi){
      phi_rad = M_PI*phi/180.0f;

      double g = 0;
      for(r = 0; r < params_.gainRange_; r+=dr){
        vec[0] = state[0] + r*cos(theta_rad)*sin(phi_rad);
        vec[1] = state[1] + r*sin(theta_rad)*sin(phi_rad);
        vec[2] = state[2] + r*cos(phi_rad);
        dir = vec - origin;

        octomap::point3d query(vec[0], vec[1], vec[2]);
        octomap::OcTreeNode* result = ot_->search (query);
        if (result) {
          // Break if occupied so we don't count any information gain behind a wall.
          if(result->getLogOdds() > 0)
            break;
        }
        else {
          g += (2*r*r*dr + 1/6*dr*dr*dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad/2);
        }
      }

      gain += g; 
      gain_per_yaw[theta] += g;

      Eigen::Vector3d o(1,0,0);
      visualization_msgs::Marker a;
      a.header.stamp = ros::Time::now();
      a.header.seq = r_ID_;
      a.header.frame_id = params_.navigationFrame_;
      a.id = r_ID_; r_ID_++;
      a.ns = "rays";
      a.type = visualization_msgs::Marker::ARROW;
      a.action = visualization_msgs::Marker::ADD;
      a.pose.position.x = state[0];
      a.pose.position.y = state[1];
      a.pose.position.z = state[2];

      Eigen::Quaternion<double> q;
      q.setFromTwoVectors(o, dir);
      q.normalize();
      a.pose.orientation.x = q.x();
      a.pose.orientation.y = q.y();
      a.pose.orientation.z = q.z();
      a.pose.orientation.w = q.w();

      a.scale.x = r;
      a.scale.y = 0.01;
      a.scale.z = 0.01;
      a.color.r = g * 8;
      a.color.g = 127.0 / 255.0;
      a.color.b = 0.0;
      a.color.a = 0.3;
      a.lifetime = ros::Duration(10.0);
      a.frame_locked = false;

      // visualization_msgs::Marker a = createRayMarker(origin, dir, r, g, r_ID_++, params_.navigationFrame_);
      params_.inspectionPath_.publish(a);


    }
  }

  int best_yaw = 0;
  double best_yaw_score = 0;
  for(int yaw = -180; yaw < 180; yaw++){
    double yaw_score = 0;
    for(int fov = -fov_y/2; fov < fov_y/2; fov++){
      int theta = yaw+fov;
      if(theta < -180) theta+=360;
      if(theta >  180) theta-=360;
      yaw_score += gain_per_yaw[theta];
    }

    if(best_yaw_score < yaw_score){
      best_yaw_score = yaw_score;
      best_yaw = yaw;
    }
  }

  gain = best_yaw_score; 
  // ROS_INFO_STREAM("Gain is " << gain);
  publishGain(gain, origin);

  double yaw = M_PI*best_yaw/180.f;
  return std::make_pair(gain, yaw);
}

void nbvInspection::RrtGP::memorizeBestBranch()
{
  bestBranchMemory_.clear();
  Node<StateVec> * current = bestNode_;
  while (current->parent_ /* && current->parent_->parent_ */) {
    bestBranchMemory_.push_back(current->state_);
    current = current->parent_;
  }
}

void nbvInspection::RrtGP::clear()
{
  delete rootNode_;
  rootNode_ = NULL;

  counter_ = 0;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;

  r_ID_ = 0;

  kd_free(kdTree_);
}

void nbvInspection::RrtGP::publishNode(Node<StateVec> * node)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(node->gain_ / 20.0, 0.05);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0 / 255.0;
  p.color.g = 167.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);

  if (!node->parent_)
    return;

  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_branches";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03;
  p.scale.z = 0.03;
  p.color.r = 100.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.7;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void nbvInspection::RrtGP::publishGain(double gain, Eigen::Vector3d position){
  pigain::Node node;
  node.gain = gain;
  node.position.x = position[0];
  node.position.y = position[1];
  node.position.z = position[2];
  gain_pub_.publish(node);
}

void nbvInspection::RrtGP::octomapCallback(const octomap_msgs::Octomap& msg){
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  ot_ = (octomap::OcTree*)aot;
}

bool nbvInspection::RrtGP::collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r){
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(  p2[0], p2[1], p2[2]);
  octomap::point3d min(std::min(p1[0], p2[0])-r, std::min(p1[1], p2[1])-r, std::min(p1[2], p2[2])-r);
  octomap::point3d max(std::max(p1[0], p2[0])+r, std::max(p1[1], p2[1])+r, std::max(p1[2], p2[2])+r);
  double lsq = (end-start).norm_sq();
  double rsq = r*r;
  for(octomap::OcTree::leaf_bbx_iterator it  = ot_->begin_leafs_bbx(min, max);
                                         it != ot_->end_leafs_bbx(); ++it){
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    if(it->getLogOdds() > 0 ){ // Node is occupied
      if(CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0) return true;
      if((end-pt).norm() < r) return true;
    }
  }

  return false;
}

geometry_msgs::Pose nbvInspection::RrtGP::stateVecToPose(StateVec stateVec, std::string targetFrame){

  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(targetFrame, params_.navigationFrame_, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return geometry_msgs::Pose();
  }
  tf::Vector3 origin(stateVec[0], stateVec[1], stateVec[2]);
  double yaw = stateVec[3];

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  origin = transform * origin;
  quat = transform * quat;
  tf::Pose poseTF(quat, origin);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(poseTF, pose);

  return pose;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc: 
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only 
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
float CylTest_CapsFirst( const octomap::point3d & pt1, 
                         const octomap::point3d & pt2, float lsq, float rsq, const octomap::point3d & pt ) {
	float dx, dy, dz;	    // vector d  from line segment point 1 to point 2
	float pdx, pdy, pdz;	// vector pd from point 1 to test point
	float dot, dsq;

	dx = pt2.x() - pt1.x();	  // translate so pt1 is origin.  Make vector from
	dy = pt2.y() - pt1.y();   // pt1 to pt2.  Need for this is easily eliminated
	dz = pt2.z() - pt1.z();

	pdx = pt.x() - pt1.x();		// vector from pt1 to test point.
	pdy = pt.y() - pt1.y();
	pdz = pt.z() - pt1.z();

	// Dot the d and pd vectors to see if point lies behind the 
	// cylinder cap at pt1.x, pt1.y, pt1.z

	dot = pdx * dx + pdy * dy + pdz * dz;

	// If dot is less than zero the point is behind the pt1 cap.
	// If greater than the cylinder axis line segment length squared
	// then the point is outside the other end cap at pt2.

	if( dot < 0.0f || dot > lsq ) {
		return( -1.0f );
	}
	else {
		// Point lies within the parallel caps, so find
		// distance squared from point to line, using the fact that sin^2 + cos^2 = 1
		// the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
		// Carefull: '*' means mult for scalars and dotproduct for vectors
		// In short, where dist is pt distance to cyl axis: 
		// dist = sin( pd to d ) * |pd|
		// distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
		// dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
		// dsq = pd * pd - dot * dot / lengthsq
		//  where lengthsq is d*d or |d|^2 that is passed into this function 

		// distance squared to the cylinder axis:

		dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lsq;

		if( dsq > rsq ) {
			return( -1.0f );
		}
		else {
			return( dsq );		// return distance squared to axis
		}
	}
}
#endif
