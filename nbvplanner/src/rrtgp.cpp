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
#include <multiagent_collision_check/multiagent_collision_checker.h>
#include <nbvplanner/rrtgp.h>
#include <nbvplanner/tree.hpp>

nbvInspection::RrtGP::RrtGP(volumetric_mapping::OctomapManager * manager, const ros::NodeHandle& nh) :
  nh_(nh),
  gain_pub_(nh_.advertise<pigain::Node>("/gain_node", 1000))
{
  manager_ = manager;
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
  }

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/nbvp/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ = ros::package::getPath("nbvplanner") + "/data/"
        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
        + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
        + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

nbvInspection::RrtGP::~RrtGP()
{
  delete rootNode_;
  kd_free(kdTree_);
  if (fileResponse_.is_open()) {
    fileResponse_.close();
  }
  if (fileTree_.is_open()) {
    fileTree_.close();
  }
  if (filePath_.is_open()) {
    filePath_.close();
  }
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

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
}

void nbvInspection::RrtGP::iterate(int iterations)
{
// In this function a new configuration is sampled and added to the tree.
  StateVec newState;

// Sample over a sphere with the radius of the maximum diagonal of the exploration
// space. Throw away samples outside the sampling region it exiting is not allowed
// by the corresponding parameter. This method is to not bias the tree towards the
// center of the exploration space.
  double radius = sqrt(
      SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
      + SQ(params_.minZ_ - params_.maxZ_));
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

// Find nearest neighbour
  kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }
  nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
      nearest);
  kd_res_free(nearest);

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
  if (volumetric_mapping::OctomapManager::CellStatus::kFree
      == manager_->getLineStatusBoundingBox(
          origin, direction + origin + direction.normalized() * params_.dOvershoot_,
          params_.boundingBox_)
      && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_)) {
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
    if (volumetric_mapping::OctomapManager::CellStatus::kFree
        == manager_->getLineStatusBoundingBox(
            origin, direction + origin + direction.normalized() * params_.dOvershoot_,
            params_.boundingBox_)
        && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_,
                                      segments_)) {

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
  // This function computes the gain
  // TODO Parameterize
  int n_rays = 1000;
  double fov_y = 56, fov_p = 42;

  double dr = 0.2, dphi = 10, dtheta = 10;
  double dphi_rad = M_PI*dphi/180.0f, dtheta_rad = M_PI*dtheta/180.0f;
  double r; int phi, theta;
  double phi_rad, theta_rad;

  double gain = 0.0;
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

        // Check cell status and add to the gain.
        double probability;
        volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
            vec, &probability);
        if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
          g += (2*r*r*dr + 1/6*dr*dr*dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad/2);
        }
        else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
          // Break if occupied so we don't count any information gain behind a wall.
          break;
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
  ROS_INFO_STREAM("Gain is " << gain);
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

  if (params_.log_) {
    for (int i = 0; i < node->state_.size(); i++) {
      fileTree_ << node->state_[i] << ",";
    }
    fileTree_ << node->gain_ << ",";
    for (int i = 0; i < node->parent_->state_.size(); i++) {
      fileTree_ << node->parent_->state_[i] << ",";
    }
    fileTree_ << node->parent_->gain_ << "\n";
  }
}

void nbvInspection::RrtGP::publishGain(double gain, Eigen::Vector3d position){
  pigain::Node node;
  node.gain = gain;
  node.position.x = position[0];
  node.position.y = position[1];
  node.position.z = position[2];
  gain_pub_.publish(node);
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
#endif
