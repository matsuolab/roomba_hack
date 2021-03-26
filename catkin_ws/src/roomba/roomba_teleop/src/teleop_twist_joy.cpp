#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopTwistJoy{
    public:
        TeleopTwistJoy();
        void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
        void CommandCallback(const geometry_msgs::TwistConstPtr& msg);

        void process();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        //subscriber
        ros::Subscriber joy_sub;
        ros::Subscriber cmd_sub;

        //publisher
        ros::Publisher vel_pub;
        ros::Publisher dock_pub;
        ros::Publisher undock_pub;

        geometry_msgs::Twist joy_vel;
        geometry_msgs::Twist cmd_vel;
        bool dock_mode;
        bool teleop_mode;
        double HZ;
        double MAX_SPEED;
        double MAX_YAWRATE;
        double VEL_RATIO;
};

TeleopTwistJoy::TeleopTwistJoy()
    :private_nh("~")
{
    //subscriber
    joy_sub = nh.subscribe("/joy",1, &TeleopTwistJoy::JoyCallback, this);
    cmd_sub = nh.subscribe("/planner/cmd_vel",1, &TeleopTwistJoy::CommandCallback, this);

    //publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
    dock_pub = nh.advertise<std_msgs::Empty>("/dock", 1, true);
    undock_pub = nh.advertise<std_msgs::Empty>("/undock", 1, true);

    //param
    private_nh.param("HZ", HZ, {20});
    private_nh.param("MAX_SPEED", MAX_SPEED, {0.5});
    private_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.5});
    private_nh.param("VEL_RATIO", VEL_RATIO, {0.5});

    dock_mode = false;
    teleop_mode = true;

}

void TeleopTwistJoy::CommandCallback(const geometry_msgs::TwistConstPtr& msg)
{
    cmd_vel = *msg;
}

void TeleopTwistJoy::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    sensor_msgs::Joy joy = *msg;

    joy_vel.linear.x = joy.axes[1]*MAX_SPEED;
    joy_vel.angular.z = joy.axes[0]*MAX_YAWRATE;

    if(joy.axes[7]==1.0){
        joy_vel.linear.x = VEL_RATIO*MAX_SPEED;
        joy_vel.angular.z = 0.0;
    }else if(joy.axes[7]==-1.0){
        joy_vel.linear.x = -VEL_RATIO*MAX_SPEED;
        joy_vel.angular.z = 0.0;
    }else if(joy.axes[6]==1.0){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = VEL_RATIO*MAX_YAWRATE;
    }else if(joy.axes[6]==-1.0){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = -VEL_RATIO*MAX_YAWRATE;
    }

    if(joy.buttons[4]==0){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = 0.0;
    }
    if(joy.buttons[0]==1){
        dock_mode = true;
    }
    if(joy.buttons[1]==1){
        dock_mode = false;
    }
    if(joy.buttons[2]==1){
        teleop_mode = true;
    }
    if(joy.buttons[3]==1){
        teleop_mode = false;
    }
}

void TeleopTwistJoy::process(){
    ros::Rate loop_rate(HZ);
    std_msgs::Empty empty_msgs;
    bool pre_dock_mode = dock_mode;
    while(ros::ok()){
        if(teleop_mode){
            if(dock_mode){
                if(pre_dock_mode!=dock_mode) dock_pub.publish(empty_msgs);
            }else{
                if(pre_dock_mode!=false) undock_pub.publish(empty_msgs);
                std::cout << "=== joy_vel ===" << std::endl;
                std::cout << joy_vel << std::endl;
                vel_pub.publish(joy_vel);
            }
        }else{
            if(pre_dock_mode!=false) undock_pub.publish(empty_msgs);
            std::cout << "=== cmd_vel ===" << std::endl;
            std::cout << cmd_vel << std::endl;
            vel_pub.publish(cmd_vel);
        }
        pre_dock_mode = dock_mode;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_twist_joy");

    TeleopTwistJoy teleop_twist_joy;
    teleop_twist_joy.process();

    return 0;
}
