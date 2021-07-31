#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

sensor_msgs::LaserScan scan;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_straight");

    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scan_callback);

    ros::Rate loop_rate(10);

    while (ros::ok()){
        geometry_msgs::Twist vel;
        if(!scan.ranges.empty()){
            std::vector<float>::iterator min_range = std::min_element(
                    scan.ranges.begin()+int(scan.ranges.size()*0.25),
                    scan.ranges.end()-int(scan.ranges.size()*0.25));
            size_t min_idx = std::distance(scan.ranges.begin(), min_range);
            ROS_INFO_STREAM("min_range: "<< scan.ranges[min_idx] << ", min_idx: " << min_idx);
            if(scan.ranges[min_idx] >= 0.4){
                vel.linear.x = 0.3;
                vel.angular.z = 0.0;
            }else{
                vel.linear.x = 0.0;
                if(min_idx < scan.ranges.size()/2.0){
                    vel.angular.z = 0.5;
                }else{
                    vel.angular.z = -0.5;
                }

            }
        }
        vel_pub.publish(vel);
        ROS_INFO_STREAM("vel: "<< vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
