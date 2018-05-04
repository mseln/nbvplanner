#include <nbvplanner/rrtviz.h>

visualization_msgs::Marker createRayMarker(Eigen::Vector3d pos, Eigen::Vector3d dir, double r, double gain, int id, std::string frame_id){
      visualization_msgs::Marker a;
      a.header.stamp = ros::Time::now();
      a.header.seq = id;
      a.header.frame_id = frame_id;
      a.id = id;
      a.ns = "rays";
      a.type = visualization_msgs::Marker::ARROW;
      a.action = visualization_msgs::Marker::ADD;
      a.pose.position.x = pos[0];
      a.pose.position.y = pos[1];
      a.pose.position.z = pos[2];

      Eigen::Vector3d o(1,0,0);
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
      a.color.r = gain * 8;
      a.color.g = 127.0 / 255.0;
      a.color.b = 0.0;
      a.color.a = 0.3;
      a.lifetime = ros::Duration(10.0);
      a.frame_locked = false;
}
