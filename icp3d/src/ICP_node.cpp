/**********************************

    Created on : 2nd March 2020 
    Author     : Krishna Sandeep

**********************************/

#include <ros/ros.h>

#include "icp3d/ICP.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ICP3D icp(node, private_nh); //instance of ICP3D class

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce(); //invokes callback
        loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}