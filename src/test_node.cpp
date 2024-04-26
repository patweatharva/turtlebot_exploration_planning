#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc,char** argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    ros::Publisher voice;
    voice = nh.advertise<std_msgs::String>("voice", 10);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /* code for loop body */
        std_msgs::String msg;
        msg.data = "hello from test node !" ;
        voice.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }    
     
}