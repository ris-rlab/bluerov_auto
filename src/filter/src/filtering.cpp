//#include "filtering.h"
#include <ctime>
//#include <stdio.h>
//#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <sstream>
//#include <fstream>
#include <vector>
//#include <iostream>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <queue>
#include <cmath>
using namespace std;

std::vector<double> vec;
//init linacc x&y
//init wlpos x&y
geometry_msgs::Point lin_acc;
geometry_msgs::Point wlpose;
geometry_msgs::Point pos;
int a = 0, b = 0;
std::queue<float>vx, vy;

time_t t = 0;
float sumx = 0.0, sumy = 0.0, meanx = 0.0, meany = 0.0, sqrx = 0.0, sqry = 0.0, stdx = 0.0, stdy = 0.0;
//ros::Time time1 = 0, duration;
void CallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f]", msg->point.x,msg->point.y);
    lin_acc.x = msg->point.x;
    lin_acc.y = msg->point.y;
    cout<<"AA"<<time(NULL)-t<<endl;
    t = time(NULL);

   
}
void CallBack1(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    //ROS_INFO("Seq: [%d]", msg->header.seq);
    //ROS_INFO("Position-> x: [%f], y: [%f]", msg->point.x,msg->point.y);
    wlpose.x = msg->point.x;
    wlpose.y = msg->point.y;
    cout<<"ASSA"<<wlpose.x<<endl;
}
int main(int argc, char **argv)
{
        ros::init(argc, argv, "PosePublisher");

        ros::NodeHandle n;
	ros::Rate r(10);
        ros::Publisher pub = n.advertise<geometry_msgs::Point>("/meana", 100);
        ros::Subscriber sub = n.subscribe("/waterlinked/acoustic_position/raw", 1000, CallBack1);//pointstamped
        //has two msg /waterlinked/acoustic_position/filtered and /waterlinked/global_position and /waterlinked/acoustic_position/raw
        //ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, CallBack1);
        //inside data is linear_acceleration x y z
        //reading the data from IMU linear acceleration
	while(ros::ok()){ 
		ros::spinOnce();       
			if(a<100){
			    vx.push(wlpose.x);
			    vy.push(wlpose.y);
		            sumx+=wlpose.x;
		            sumy+=wlpose.y;
		            a++;
			}
			    
		   	    	
		  cout<<"We have "<<wlpose.x<<" "<<wlpose.y<<endl;
		  
		    if(a>=100){
			sumx = sumx - vx.front() + wlpose.x;
			sumy = sumy -vy.front()+wlpose.y;
			vx.pop();
			vy.pop();
			vx.push(wlpose.x);
			vy.push(wlpose.y);
		        meanx = sumx/vx.size();
		        meany = sumy/vy.size();
			cout<<"AAA"<<a<<endl;
			for(auto i = vx.begin(); i!=vx.end(); i++)
				 sqrx +=(meanx - *i)*(meanx - *i);
		     	for(auto i = vy.begin(); i!= vy.end(); i++)
				sqry+= (meany - *i)*(meany - *i);
			stdx = sqrt(sqrx/vx.size());
		        stdy = sqrt(sqry/vy.size());    
			pos.x = meanx;
			pos.y = meany;
			pub.publish(pos);      		   
		    }
		    ROS_INFO("Data published mean x = [%f] mean y = [%f]", meanx, meany);
		    ROS_INFO("Data published stdev x = [%f] stdev y = [%f]", stdx, stdy);


		    
		    r.sleep();

	}



}
