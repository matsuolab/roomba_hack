#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_straight");

    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop_rate(10);
    double start_time = ros::Time::now().toSec();

    while (ros::ok()){
        geometry_msgs::Twist vel;
        if(ros::Time::now().toSec() - start_time < 3.0){
            vel.linear.x = 0.5;
            vel.angular.z = 0.0;
            ROS_INFO("go straight(%f[m/s])", vel.linear.x);
        }else{
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            ROS_INFO("stop(%f[m/s])", vel.linear.x);
        }
        vel_pub.publish(vel);

        loop_rate.sleep();
    }

    return 0;
}
