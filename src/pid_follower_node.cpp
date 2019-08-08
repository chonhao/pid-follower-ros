extern "C"{
    #include "pid.h"
}

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <pid_follower/follower_err.h>
#include "pid_follower.h"
#include <stdlib.h> 

// using namespace std;

static void abs_limit(double *a, double ABS_MAX)
{
  if (*a > ABS_MAX)
      *a = ABS_MAX;
  if (*a < -ABS_MAX)
      *a = -ABS_MAX;
}

double pid_calc(pid_m *pid, double get, double set)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get; 

    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

    ROS_INFO("get: %lf\tset: %lf\terr: %lf", get, set, pid->err[NOW]);
    ROS_INFO("pout: %lf\tiout: %lf\tdout: %lf", pid->pout, pid->iout, pid->dout);


    abs_limit(&(pid->iout), pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->max_output);

    pid->err[LAST]  = pid->err[NOW];
    return pid->out;
}

void encoderVelocityCallback(const nav_msgs::OdometryConstPtr &msg){
    nav_msgs::Odometry odom = *msg;
    robot_chassis.position_x = odom.pose.pose.position.x;
    robot_chassis.position_y = odom.pose.pose.position.y;
    robot_chassis.orientation = odom.pose.pose.orientation.z;
    robot_chassis.speed_x = odom.twist.twist.linear.x;
    robot_chassis.speed_y = odom.twist.twist.linear.y;
    robot_chassis.speed_angular = odom.twist.twist.angular.z;
    // ROS_INFO("\n\tposition.x %f\
    // \n\tposition.y %f\
    // \n\torientation %f\
    // \n\tspeed.x %f\
    // \n\tspeed.y %f\
    // \n\tspeed.angular %f\n\
    // ", robot_chassis.position_x, robot_chassis.position_y,\
    // robot_chassis.orientation, robot_chassis.speed_x,\
    // robot_chassis.speed_y, robot_chassis.speed_angular);
}

void followerErrCallback(const pid_follower::follower_errConstPtr &msg){
    pid_follower::follower_err err = *msg;
    follower_err.is_obj_being_tracked = err.is_obj_being_tracked;
    follower_err.direction_err = err.direction_err;
    follower_err.distance_err = err.distance_err;
    // ROS_INFO("%d %lf %lf", follower_err.is_obj_being_tracked, follower_err.direction_err,\
    follower_err.distance_err);
}

void setSpeed(int which, double speed){
    switch (which){
        case SPEED_LINEAR: my_cmd_vel.linear.x = speed;
            break;
        case SPEED_ANGULAR: my_cmd_vel.angular.z = speed;
            break;
        default:
            ROS_INFO("Something went wrong");
            break;
    }
}

int main(int argc, char **argv){
    my_cmd_vel = {};

    ros::init(argc, argv, "pid_follower_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_raw_odom = nh.subscribe("raw_odom", 1000, &encoderVelocityCallback);
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Subscriber sub_follower_err = nh.subscribe("follower_err", 1000, &followerErrCallback);

    ros::NodeHandle nh_private("~");
    nh_private.param<double>("distance_max_out", pid_distance.max_output, 0.0);
    nh_private.param<double>("distance_integral_limit", pid_distance.integral_limit, 0.0);
    nh_private.param<double>("distance_kp", pid_distance.p, 0.0);
    nh_private.param<double>("distance_ki", pid_distance.i, 0.0);
    nh_private.param<double>("distance_kd", pid_distance.d, 0.0);

    nh_private.param<double>("direction_max_out", pid_direction.max_output, 0.0);
    nh_private.param<double>("direction_integral_limit", pid_direction.integral_limit, 0.0);
    nh_private.param<double>("direction_ki", pid_direction.i, 0.0);
    nh_private.param<double>("direction_kp", pid_direction.p, 0.0);
    nh_private.param<double>("direction_kd", pid_direction.d, 0.0);

    double distance_target, direction_target, distance_target_temp, direction_target_temp;
    nh_private.param<double>("distance_target", distance_target, 0.0);
    nh_private.param<double>("direction_target", direction_target, 0.0);
    distance_target_temp = distance_target;
    direction_target_temp = direction_target;
    
    ros::Rate loop_rate(5);
    while (ros::ok()){
        double distance_output = pid_calc(&pid_distance, follower_err.distance_err, distance_target_temp);
        double direction_output = pid_calc(&pid_direction, follower_err.direction_err, direction_target_temp);

        if(abs(follower_err.direction_err - direction_target)<40){
            direction_output = 0;
        }

        ROS_INFO("output %lf", direction_output);

        if(follower_err.is_obj_being_tracked && follower_err.distance_err > 0){
            distance_target_temp = distance_target;
            direction_target_temp = direction_target;
        }else{
            distance_target_temp = follower_err.distance_err;
            direction_target_temp = follower_err.direction_err;
        }

        setSpeed(SPEED_LINEAR, -1 * distance_output);
        setSpeed(SPEED_ANGULAR, direction_output);

        pub_cmd_vel.publish(my_cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}