#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <fstream>
#include <ostream>
#include "groundtruthprovider.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  ROS_INFO("Let's publish the groundtuth");
  // read txt pose file
  std::string pkg_path = ros::package::getPath("groundtruth_odom");
  groundtruthProvider gp(pkg_path + "/data",nh);
  gp.publishPath();

  // let's publish odometry messages
  std::vector<std::vector<double>> odom = gp.getPoseList();
  std::vector<std::vector<double>> vel = gp.getVelocityList();
  ros::Publisher pubOdom;
  pubOdom = nh.advertise<nav_msgs::Odometry> ("groundtruth_odom/odom", 1);

  ROS_INFO("Now let's publish the odometry");
  for (int i=0; i<static_cast<int>(odom.size()); i++){
    double x, y, z, roll, pitch, yaw;
    x = odom[i][0];
    y = odom[i][1];
    z = odom[i][2];
    roll = odom[i][3];
    pitch = odom[i][4];
    yaw = odom[i][5];

    // tf to update the robot frame
    static tf::TransformBroadcaster tfMap2Base;
    tf::Transform map_to_base_link = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
    tfMap2Base.sendTransform(tf::StampedTransform(map_to_base_link, ros::Time::now(), "map", "robot"));

    // publish the odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    odom.twist.twist.linear.x = vel[i][0];
    odom.twist.twist.linear.y = vel[i][1];
    odom.twist.twist.linear.z = vel[i][2];
    pubOdom.publish(odom);
    usleep(1e5);
  }


  ros::spin();
}
