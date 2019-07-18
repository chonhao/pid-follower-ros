#include "pid.h"

#define SPEED_LINEAR 1
#define SPEED_ANGULAR 2

#define LINEAR_SPEED_MAX 1.20
#define ANGULAR_SPEED_MAX 1.0

struct chassis{
    double position_x, position_y, orientation, \
    speed_x, speed_y, speed_angular;
};

struct follower_err{
    uint8_t is_obj_being_tracked;
    double direction_err, distance_err;
};

chassis robot_chassis;
geometry_msgs::Twist my_cmd_vel;
follower_err follower_err;

pid_m pid_direction, pid_distance;