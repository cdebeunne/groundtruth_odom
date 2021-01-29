# groundtruth_odom

This is a simple implementation of a ROS visualizer for navigation purpose. To try it, open a terminal and type this:

```
cd ~/catkin_ws/src
git clone https://github.com/cdebeunne/groundtruth_odom.git
cd ..
catkin_make
roslaunch groundtruth_odom run.launch
```

It will plot the provided KITTI groundtruth, you can try another one by simply editing the folder path in the main.

![](https://github.com/cdebeunne/groundtruth_odom/blob/master/launch/pic.png)
