#ifndef RRTVIS_H
#define RRTVIS_H

#include <string>

#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>

visualization_msgs::Marker createRayMarker(Eigen::Vector3d pos, Eigen::Vector3d dir, double r, double gain, int id, std::string frame_id);

#endif
