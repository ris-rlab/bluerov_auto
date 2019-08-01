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

std::vector<double> vec;
nav_msgs::Odometry real;

void copydata(geometry_msgs::Pose desiredposition,int array[]){
    desiredposition.position.x =array[0];
    desiredposition.position.y =array[1];
    desiredposition.position.z =array[2];
    desiredposition.orientation.x = array[3];
    desiredposition.orientation.y = array[4];
    desiredposition.orientation.z = array[5];
    desiredposition.orientation.w = array[6];
}
void readDATA(int arr[6]){
string value;
int i = 0;
std::ifstream myFile;

myFile.open("filename.txt", std::ios::app);
if (myFile.is_open() && !(myFile.eof())){
    std::cout << "File is open."<<std::endl;
    getline(myFile, value);
    std::sstream s(value);
    while(s>>arr[i]){
            arr[i] = atoi(value);
            i++;
        std::cout << "value is " <<value<< std::endl;
        }

}
else std::cout << "Unable to open the file";
}
//convert orientation to roll pitch and yaw - error range in this angles
int checkData(geometry_msgs::Pose desiredposition, Odometry real)
{
    int x = 0, y = 0, z = 0, ox = 0, oy = 0, oz = 0, ow = 0, check = 0, error = 0.5;
if ((desiredposition.position.x < real.position.x + error) && (desiredposition.position.x > real.position.x - error)){
    x = 1;
}
if ((desiredposition.position.y < real.position.y + error) && (desiredposition.position.y > real.position.y - error)){
    y = 1;
}
if ((desiredposition.position.z < real.position.z + error) && (desiredposition.position.z > real.position.z - error)){
    z = 1;
}
if ((desiredposition.orientation.x < real.orientation.x + error) && (desiredposition.orientation.x > real.orientation.x - error)){
    ox = 1;
}
if ((desiredposition.orientation.y < real.orientation.y + error) && (desiredposition.orientation.y >  real.orientation.y - error)){
    oy = 1;
}
if ((desiredposition.orientation.z < real.orientation.z + error) && (desiredposition.orientation.z > real.orientation.z - error)){
    oz = 1;
}
if ((desiredposition.orientation.w < real.orientation.w + error) && (desiredposition.orientation.w > real.orientation.w - error)){
    ow = 1;
}
if (x+y+z+ox+oy+oz+ow == 7){
    check = 1;
}
return(check);
}

void CallBack(const nav_msg::Odometry::ConstPtr& msg)
{
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    real = msg->data;

}
int main(int argc, char **argv)
{


        ros::init(argc, argv, "PosePublisher");

        ros::NodeHandle n;

        ros::Publisher pub = n.advertise<geometry_msgs::Pose>("position", 100);
        ros::Subscriber sub = n.subscriber("odom", 1000, CallBack);
        int array[6];
        //ros::Publisher pub = n.advertise<geometry_msgs::Pose::Quaternion>("orientation", 100);
        while (ros::ok())
        {
                geometry_msgs::Pose desiredposition;
                int array[6];
                int r = 0;
                readDATA(array);
                copydata(desiredposition,array[6]);
                while (r == 0){
                r = checkData(desiredposition, real);
                if (r == 1){
                    readDATA(array);
                    copydata(desiredposition, array[6]);
                    pub.publish(desiredposition);
                    ROS_INFO("Data published");
                }
                else{
                    ros::Duration(5).sleep();
                }
                }
                ros::spinOnce();
                //Added a delay so not to spam
                sleep(2);

        }

}
