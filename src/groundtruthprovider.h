#ifndef GROUNDTRUTHPROVIDER_H
#define GROUNDTRUTHPROVIDER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <fstream>
#include <ostream>


class groundtruthProvider
{
public:
  groundtruthProvider(std::string folderPath, ros::NodeHandle nh);
  Eigen::Matrix3d eul2rotm(double roll, double pitch, double yaw);
  void publishPath();

  std::vector<std::vector<double>> getPoseList(){return _poseList;}
  std::vector<std::vector<double>> getVelocityList(){return _odomList;}

private:
  ros::NodeHandle _nh;
  ros::Publisher _pubPath;

  std::vector<std::vector<double>> _poseList;
  std::vector<std::vector<double>> _odomList;
};

#endif // GROUNDTRUTHPROVIDER_H
