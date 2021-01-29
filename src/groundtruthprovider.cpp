#include "groundtruthprovider.h"

groundtruthProvider::groundtruthProvider(std::string folderPath, ros::NodeHandle nh): _nh(nh)
{
  // parsing the data folder
  bool in = true;
  int i = 0;
  while (in){
    std::string txtPath;
    i++;
    int no = log10(i);
    int numberOfZeros = 10-no;
    for (int j = 0; j<numberOfZeros-1; j++){
      txtPath = txtPath + "0";
    }
    txtPath = folderPath + "/" + txtPath + std::to_string(i) + ".txt";
    std::ifstream flow(txtPath);
    if (flow){
      std::vector<double> odom;
      double value;
      for (int i=0; i<8; i++){
        flow >> value;
      }
      for (int i=0; i<3;i++){
        flow >> value;
        odom.push_back(value);
      }
      for (int i=0;i<9;i++){
        flow >> value;
      }
      for (int i=3;i<6;i++){
        flow >> value;
        odom.push_back(value);
      }
      _odomList.push_back(odom);
    } else {
      in = false;
    }
  }

  // building the pose list
  double dt = 0.1;
  Eigen::Vector3d xyz;
  xyz << 0,0,0;
  Eigen::Vector3d rpy;
  rpy << 0,0,0;
  for (int i=0; i < static_cast<int>(_odomList.size()); i++){

    Eigen::Matrix3d R_robot2world;
    rpy(0) += _odomList[i][3]*dt;
    rpy(1) += _odomList[i][4]*dt;
    rpy(2) += _odomList[i][5]*dt;
    R_robot2world = eul2rotm(rpy(0), rpy(1), rpy(2));

    Eigen::Vector3d deplacement;
    deplacement << _odomList[i][0]*dt, _odomList[i][1]*dt, _odomList[i][2]*dt;
    xyz += R_robot2world*deplacement;

    std::vector<double> pose;
    pose.push_back(xyz(0));
    pose.push_back(xyz(1));
    pose.push_back(xyz(2));
    pose.push_back(rpy(0));
    pose.push_back(rpy(1));
    pose.push_back(rpy(2));
    _poseList.push_back(pose);
  }
}

Eigen::Matrix3d groundtruthProvider::eul2rotm(double roll, double pitch, double yaw){
  Eigen::Matrix3d rotx;
  rotx << 1,0,0,
      0,cos(roll),-sin(roll),
      0,sin(roll),cos(roll);
  Eigen::Matrix3d roty;
  roty << cos(pitch),0,sin(pitch),
      0,1,0,
      -sin(pitch),0,cos(pitch);
  Eigen::Matrix3d rotz;
  rotz << cos(yaw), -sin(yaw),0,
      sin(yaw),cos(yaw),0,
      0,0,1;
  return rotx*roty*rotz;
}

void groundtruthProvider::publishPath(){

  // initializing the publisher
  _pubPath = _nh.advertise<nav_msgs::Path>     ("groundtruth_odom/path", 1);

  usleep(2e6);
  for (int i=0; i<static_cast<int>(_poseList.size()); i++){
    double x,y,z,roll,pitch,yaw;
    x = _poseList[i][0];
    y = _poseList[i][1];
    z = _poseList[i][2];
    roll = _poseList[i][3];
    pitch = _poseList[i][4];
    yaw = _poseList[i][5];

    static nav_msgs::Path pathMsg;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = z;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    pathMsg.poses.push_back(pose_stamped);
    pathMsg.header.stamp = ros::Time::now();
    pathMsg.header.frame_id = "map";
    _pubPath.publish(pathMsg);
    usleep(1e2);
  }

}
