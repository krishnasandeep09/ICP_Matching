/**********************************

    Created on : 2nd March 2020 
    Author     : Krishna Sandeep

**********************************/

#include "icp3d/ICP.h"

#include <iostream>
#include <chrono>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/registration/icp.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

/* @brief Constructor */
ICP3D::ICP3D(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    /* Loading parameters */
    private_nh.param("leaf_size", _leaf_size, 0.1);
	ROS_INFO("leaf_size: %f", _leaf_size);
    private_nh.param("dist_threshold", _dist_threshold, 0.1);
    ROS_INFO("dist_threshold: %f", _dist_threshold);
    private_nh.param("eps_angle", _eps_angle, 15);
    ROS_INFO("eps_angle: %d deg", _eps_angle);
    private_nh.param("minX", _minX, 0.0);
	ROS_INFO("minX: %f", _minX);
    private_nh.param("minY", _minY, -25.0);
	ROS_INFO("minY: %f", _minY);
    private_nh.param("minZ", _minZ, -3.0);
	ROS_INFO("minZ: %f", _minZ);
    private_nh.param("maxX", _maxX, 40.0);
	ROS_INFO("maxX: %f", _maxX);
    private_nh.param("maxY", _maxY, 25.0);
	ROS_INFO("maxY: %f", _maxY);
    private_nh.param("maxZ", _maxZ, 3.0);
	ROS_INFO("maxZ: %f", _maxZ);
    private_nh.param("mean_k", _mean_k, 50);
	ROS_INFO("mean_k: %d", _mean_k);
    private_nh.param("std_mul", _std_mul, 1.0);
	ROS_INFO("std_mul: %f", _std_mul);

    private_nh.param("transformation_epsilon", _transformation_epsilon, 0.01);
    ROS_INFO("transformation_epsilon: %f", _transformation_epsilon);
    private_nh.param("max_iterations", _max_iters, 75);
    ROS_INFO("max_iterations: %d", _max_iters);
    private_nh.param("euclidean_fitness_epsilon", _euclidean_fitness_epsilon, 0.1);
    ROS_INFO("euclidean_fitness_epsilon: %f", _euclidean_fitness_epsilon);
    private_nh.param("max_correspondence_distance", _max_correspondence_distance, 1.0);
    ROS_INFO("max_correspondence_distance: %f", _max_correspondence_distance);

    private_nh.param<std::string>("point_cloud_topic", _point_cloud_topic, "velodyne_points");
    ROS_INFO("point_cloud_topic: %s", _point_cloud_topic.c_str());
    private_nh.param<std::string>("imu_topic", _imu_topic, "imu/data");
    ROS_INFO("imu_topic: %s", _imu_topic.c_str());
    private_nh.param<std::string>("pose_topic", _pose_topic, "icp_pose");
    ROS_INFO("pose_topic: %s", _pose_topic.c_str());

    pc_sub = node.subscribe(_point_cloud_topic, 1, &ICP3D::cloudCallback, this);
    imu_sub = node.subscribe(_imu_topic, 1, &ICP3D::imuCallback, this);
    pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>(_pose_topic, 1);

    //initialising values
    _prev_acc = 0.0;
    _curr_acc = 0.0;
    _yaw_rate = 0.0;
    _speed = 0.0;

    is_initial = true;
    is_imu_start = true;
}

/* @brief Cropping the cloud using Box filter */
void ICP3D::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(_minX, _minY, _minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(_maxX, _maxY, _maxZ, 1.0));
    boxFilter.setInputCloud(in_cloud_ptr);
    boxFilter.filter(*out_cloud_ptr);

    //cout<<"Crop Input: "<<in_cloud_ptr->size()<<" pts, Crop output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

/* @brief Removing Noise using Statistical outlier method */
void ICP3D::removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setMeanK(_mean_k);
    sor.setStddevMulThresh(_std_mul);
    sor.filter(*out_cloud_ptr);

    //cout<<"Noise Input: "<<in_cloud_ptr->size()<<" pts, Noise output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

/* @brief Downsampling using Aprroximate Voxel grid filter */
void ICP3D::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    //cout<<"-------Downsampling cloud---------"<<endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    approx_vg.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    approx_vg.setInputCloud(in_cloud_ptr);
    approx_vg.filter(*out_cloud_ptr);

    //cout<<"DS Input: "<<in_cloud_ptr->size()<<" pts, DS output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

/* @brief Removes ground plane using perpendicular plane model with RANSAC */
void ICP3D::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr)
{
    //cout<<"-------Removing ground---------"<<endl;
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //Creating the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true); //optional
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_dist_threshold);
    seg.setAxis(Eigen::Vector3f(0,0,1)); //z-axis
    seg.setEpsAngle(_eps_angle);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout<<"Could not estimate the plane"<<endl;
    }

    //Remove ground from the cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true); //true removes the indices
    extract.filter(*out_cloud_ptr);

    //Extract ground from the cloud
    extract.setNegative(false); //false leaves only the indices
    extract.filter(*ground_plane_ptr);

    //cout<<"GR Input: "<<in_cloud_ptr->size()<<" pts, GR output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

void ICP3D::filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr only_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_noise_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cropCloud(in_cloud_ptr, cropped_cloud_ptr);
    removeGround(cropped_cloud_ptr, no_ground_cloud_ptr, only_ground_cloud_ptr);
    removeNoise(no_ground_cloud_ptr, no_noise_cloud_ptr);
    //removeNoise(no_ground_cloud_ptr, out_cloud_ptr);
    downsampleCloud(no_noise_cloud_ptr, out_cloud_ptr);

    return;
}

/* @brief Callback function to fetch IMU data, calculates speed and change in yaw */
void ICP3D::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(is_imu_start)
    {
        _prev_acc = msg->linear_acceleration.x;
        _prev_imu_time = msg->header.stamp.toSec();

        is_imu_start = false;
    }
    else
    {
        _curr_acc = msg->linear_acceleration.x;
        _curr_imu_time = msg->header.stamp.toSec();

        double del_time = _curr_imu_time - _prev_imu_time;
        double avg_acc = 0.5*(_prev_acc + _curr_acc);

        _speed = avg_acc*del_time;
        _yaw_rate = msg->angular_velocity.z;

        _prev_acc = _curr_acc;
        _prev_imu_time = _curr_imu_time;
    }

    return;
}

/* @brief Callback function for pointcloud, implements the ICP algo */
void ICP3D::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //cout<<"-------Entered callback---------"<<endl;
    
    if(is_initial)
    {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *prev_cloud_ptr);

        filterCloud(prev_cloud_ptr, filtered_cloud_ptr);
        
        _prev_cloud = *filtered_cloud_ptr;
        _prev_time_stamp = msg->header.stamp.toSec();

        //initialising the previous transformation
        prev_transformation = Eigen::Matrix4f::Identity();

        is_initial = false;
    }
    else
    {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(_prev_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        chrono::time_point<chrono::system_clock> start, end;

        start = chrono::system_clock::now(); 
        pcl::fromROSMsg(*msg, *current_cloud_ptr);
        
        filterCloud(current_cloud_ptr, filtered_cloud_ptr);
        
        //cout<<"Filtered cloud has "<<filtered_cloud_ptr->size()<<"points"<<endl;
        //cout<<"Current cloud has "<<current_cloud_ptr->size()<<"points"<<endl;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setTransformationEpsilon(_transformation_epsilon);
        icp.setMaximumIterations(_max_iters);
        icp.setMaxCorrespondenceDistance(_max_correspondence_distance);
        icp.setEuclideanFitnessEpsilon(_euclidean_fitness_epsilon);
        //icp.setInputSource(_prev_cloud_ptr);
        //icp.setInputTarget(_downsampled_cloud_ptr);
        icp.setInputSource(filtered_cloud_ptr);
        icp.setInputTarget(prev_cloud_ptr);

        double diff_time = msg->header.stamp.toSec() - _prev_time_stamp; //calculating time btw the matching pointclouds

        double diff_yaw = diff_time*_yaw_rate;
        Eigen::AngleAxisf init_rotation (diff_yaw, Eigen::Vector3f::UnitZ ());
        double del_x = diff_time*_speed;
        Eigen::Translation3f init_translation (del_x, 0.0, 0.0);
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

        //cout<<"-------Matching clouds---------"<<endl;
        //start = chrono::system_clock::now(); 
        icp.align(*transformed_cloud_ptr, init_guess);
        //icp.align(*transformed_cloud_ptr);
        end = chrono::system_clock::now();

        //cout<<"-------Matching done---------"<<endl;

        cout << "ICP has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << endl;

        chrono::duration<double> elapsed_seconds = end - start;
        //cout<<"elapsed time: " << elapsed_seconds.count() << "s\n"; 

        Eigen::Matrix4f t = icp.getFinalTransformation();
        Eigen::Matrix4f curr_transformation = prev_transformation*t; //final transformation matrix

        Eigen::Matrix3f mat; //rotation matrix
        Eigen::Vector3f trans; //translation vector
        
        trans << curr_transformation(0,3), curr_transformation(1,3), curr_transformation(2,3);
        mat << curr_transformation(0,0), curr_transformation(0,1), curr_transformation(0,2),
               curr_transformation(1,0), curr_transformation(1,1), curr_transformation(1,2),
               curr_transformation(2,0), curr_transformation(2,1), curr_transformation(2,2);

        Eigen::Quaternionf quat(mat); //rotation matrix stored as a quaternion

        geometry_msgs::PoseWithCovarianceStamped curr_pose;
        curr_pose.header.stamp = ros::Time::now();
        curr_pose.header.frame_id = "icp";
        curr_pose.pose.pose.position.x = trans[0];
        curr_pose.pose.pose.position.y = trans[1];        
        curr_pose.pose.pose.position.z = trans[2];        
        curr_pose.pose.pose.orientation.x = quat.x();
        curr_pose.pose.pose.orientation.y = quat.y();        
        curr_pose.pose.pose.orientation.z = quat.z();        
        curr_pose.pose.pose.orientation.w = quat.w();

        //----------Note: for now the covariance is left at default values-----------
        //---------later covariance values will also be used,------------------------
        //---------so this can used as input to probabilistic filter like EKF/UKF----

        pose_pub.publish(curr_pose); //publishing the current pose

        _prev_cloud = *filtered_cloud_ptr;
        prev_transformation = curr_transformation;
        _prev_time_stamp = msg->header.stamp.toSec();
    }

    return;
}