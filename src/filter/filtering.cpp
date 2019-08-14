//#include "filtering.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
using namespace std;

std::vector<double> vec;
//init linacc x&y
//init wlpos x&y
geometry_msgs::Point lin_acc;
geometry_msgs::Point wlpos;
void CallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f]", msg->Point.position.x,msg-Point.position.y);
    lin_acc.position.x = msg->Point.position.x;
    lin_acc.position.y = msg->Point.position.y;
}
void CallBack1(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f]", msg->Point.position.x,msg-Point.position.y);
    wlpose.position.x = msg->Point.position.x;
    wlpose.position.y = msg->Point.position.y;
}
int main(int argc, char **argv)
{
        ros::init(argc, argv, "PosePublisher");

        ros::NodeHandle n;

        ros::Publisher pub = n.advertise<geometry_msgs::Pose>("meanandstddev", 100);
        ros::Subscriber sub = n.subscribe("/waterlinked/acoustic_position/raw", 1000, CallBack);//pointstamped
        //has two msg /waterlinked/acoustic_position/filtered and /waterlinked/global_position and /waterlinked/acoustic_position/raw
        //ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, CallBack1);
        //inside data is linear_acceleration x y z
        //reading the data from IMU linear acceleration
        int a = 0, b = 0;
        float sumx = 0.0, sumy = 0.0, meanx = 0.0, meany = 0.0, a = 1.0, sqrx = 0.0, sqry = 0.0, stdx = 0.0, stdy = 0.0;

        if(a<100){
                    sumx+=wlpose.x;
                    sumy+=wlpose.y;
                    a++;
           }

            if(a>=100){
                meanx = sumx/a;
                meany = sumy/a;
            }
            if(b<100){

                    sqrx +=(meanx - wlpose.x)*(meanx - wlpose.x);
                    sqry += (meany - wlpose.y)*(meanx - wlpose.y);
                    b++;
            }
            if(b>=100){
                stdx = (1/(b-2))*sqrt(sqrx);
                stdy = (1/(b-2))*sqrt(sqry);
            }
            ROS_INFO("Data published mean x = [%f] mean y = [%f]", meanx, meany);
            ROS_INFO("Data published stdev x = [%f] stdev y = [%f]", stdx, stdy);


            ros::spin();





}
