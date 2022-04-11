#include "roomba_teleop/roomba_teleop.h"

RoombaTeleop::RoombaTeleop()
    :local_nh("~"),
     auto_flag(false), move_flag(false), dock_flag(false)
{
    //subscriber
    cmd_sub = nh.subscribe("/planner/cmd_vel",1, &RoombaTeleop::CommandCallback, this);
    joy_sub = nh.subscribe("/joy",1, &RoombaTeleop::JoyCallback, this);

    //publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
    dock_pub = nh.advertise<std_msgs::Empty>("/dock", 1, true);
    undock_pub = nh.advertise<std_msgs::Empty>("/undock", 1, true);

    // param
    local_nh.param("HZ", HZ, {20});
    local_nh.param("MAX_SPEED", MAX_SPEED, {0.5});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {1.0});
    local_nh.param("VEL_RATIO", VEL_RATIO, {0.5});
}

void RoombaTeleop::CommandCallback(const geometry_msgs::TwistConstPtr& msg)
{
    cmd_vel = *msg;
}

void RoombaTeleop::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    sensor_msgs::Joy joy = *msg;
    if(joy.buttons[2]){
        auto_flag = false;
    }else if(joy.buttons[3]){
        auto_flag = true;
    }
    if(joy.buttons[0]){
        move_flag = false;
    }else if(joy.buttons[1]){
        move_flag = true;
    }
    if(joy.buttons[6]){
        dock_flag = false;
    }else if(joy.buttons[7]){
        dock_flag = true;
    }

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
    if(!joy.buttons[4]){
        joy_vel.linear.x = 0.0;
        joy_vel.angular.z = 0.0;
    }
}

void RoombaTeleop::process()
{
    ros::Rate loop_rate(HZ);
    std_msgs::Empty empty_msgs;
    bool pre_dock_flag = dock_flag;
    while(ros::ok()){
        geometry_msgs::Twist vel;
        ROS_INFO("==== roomba teleop ====");
        if(dock_flag){
            if(!pre_dock_flag) dock_pub.publish(empty_msgs);
            ROS_INFO("docking");
        }else{
            if(pre_dock_flag) undock_pub.publish(empty_msgs);
            ROS_INFO_STREAM((move_flag ? "move" : "stop") << " : (" << (auto_flag ? "auto" : "manual") << ")");
            if(move_flag){
                if(auto_flag) vel = cmd_vel;
                else vel = joy_vel;
                vel.linear.x = std::min(std::max(vel.linear.x, -MAX_SPEED), MAX_SPEED);
                vel.angular.z = std::min(std::max(vel.angular.z, -MAX_YAWRATE), MAX_YAWRATE);
                ROS_INFO_STREAM(vel);
            }else{
                vel.linear.x = 0.0;
                vel.angular.z = 0.0;
            }
            vel_pub.publish(vel);
        }
        pre_dock_flag = dock_flag;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roomba_teleop");

    RoombaTeleop roomba_teleop;
    roomba_teleop.process();

    return 0;
}