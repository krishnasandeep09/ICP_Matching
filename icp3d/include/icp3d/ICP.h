/**********************************

    Created on : 2nd March 2020 
    Author     : Krishna Sandeep

**********************************/

#ifndef ICP3D_ICP_H
#define ICP3D_ICP_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <string>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

class ICP3D
{
    public:
        ICP3D(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~ICP3D() {};
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); //point cloud callback
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); //imu data callback

    private:
        void cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //crops cloud using box filter
        void removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //removes noise using Statistical outlier removal
        void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //downsampling the point cloud using Voxelgrid
        void removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr); //ground removal using RANSAC
        void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //filtering the point cloud

        ros::Subscriber pc_sub; //point cloud subscriber
        ros::Subscriber imu_sub; //IMU data subscriber
        ros::Publisher pose_pub; //publishes geometry_msgs::PoseWithCovariance msgs
        pcl::PointCloud<pcl::PointXYZ> _prev_cloud; //point cloud at the previous time instance
        Eigen::Matrix4f prev_transformation; //cumulative transformation until the previous time instance
        bool is_initial, is_imu_start; //boolean to tell if this is 1st iteration of the algo and start of imu reading
        double _prev_acc, _curr_acc; //accleration in consecutive time stamps
        double _prev_imu_time, _curr_imu_time; //consecutive time stamps of imu data 
        double _prev_time_stamp; //time stamp of the previous point cloud

        /*---------sub-pub parameters----------*/
        std::string _point_cloud_topic; //point cloud ros topic to subscribe to
        std::string _imu_topic; //imu ros topic to subscribe to
        std::string _pose_topic; //pose ros topic to which to publish
        
        /*----------ICP parameters------------*/
        double _leaf_size; //leaf size for voxel grid
        double _minX, _maxX, _minY, _maxY, _minZ, _maxZ; //min and max pts for box filter
        int _mean_k; //number of neighbors to analyze for each point for noise removal
        double _std_mul; //standard deviation multiplication threshold for noise removal

        double _dist_threshold; //distance threshold for RASNSAC to consider a point as inlier (for ground removal)
        int _eps_angle; //allowed difference of angles in degrees for perpendicular plane model

        double _transformation_epsilon; //minimum transformation difference for termination condition
        int _max_iters; //max number of registration iterations
        double _euclidean_fitness_epsilon; //maximum allowed Euclidean error between two consecutive steps in the ICP loop
        double _max_correspondence_distance; //correspondences with higher distances will be ignored
        double _speed; //speed for initial guess
        double _yaw_rate; //change in yaw for initial guess
};

#endif