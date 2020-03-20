# ICP_Matching
3D ICP matching using PCL and ROS.
Takes in pointcloud and imu data, matches two consecutive clouds using ICP and outputs cumulative pose w.r.t the frame at the initial position of the robot.

Requirements:
PCL,
ROS

Input:
sensor_msgs/PointCloud2,
sensor_msgs/Imu

Output:
geometry_msgs/PoseWithCovarianceStamped

How to use:

1. Clone the repository in to your workspace
2. catkin_make #compile
3. Edit the parameters in config/icp_localizer.yaml to your suiting.
4. roslaunch icp3d icp_localizer.launch #to run the code
