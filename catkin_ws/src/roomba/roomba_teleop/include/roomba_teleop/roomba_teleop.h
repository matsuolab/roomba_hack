#ifndef __ROOMBA_TELEOP
#define __ROOMBA_TELEOP

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class RoombaTeleop{
    public:
        RoombaTeleop();
        void CommandCallback(const geometry_msgs::TwistConstPtr& msg);
        void JoyCallback(const sensor_msgs::JoyConstPtr& msg);

        void process();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle local_nh;

        //subscriber
        ros::Subscriber cmd_sub;
        ros::Subscriber joy_sub;

        //publisher
        ros::Publisher vel_pub;
        ros::Publisher dock_pub;
        ros::Publisher undock_pub;

        // flag
        bool auto_flag;
        bool move_flag;
        bool dock_flag;

        // ros message
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Twist joy_vel;

        int HZ;
        double MAX_SPEED;
        double MAX_YAWRATE;
        double VEL_RATIO;
};

#endif// __ROOMBA_TELEOP