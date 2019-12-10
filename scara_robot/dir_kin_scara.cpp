#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <math.h>

float a1 = 0.3, a2 = 0.3, d4 = 0.0;

double joint_1_angle_old = 0.0;
double joint_2_angle_old = 0.0;
double joint_3_shift_old = 0.0;
double joint_4_angle_old = 0.0;

void f_kinematic(const sensor_msgs::JointState &msg)
{
    auto joint_1_angle = msg.position[0];
    auto joint_2_angle = msg.position[1];
    auto joint_3_shift = msg.position[2];
    auto joint_4_angle = msg.position[3];

    if(joint_1_angle_old != joint_1_angle || 
        joint_2_angle_old != joint_2_angle ||
        joint_3_shift_old != joint_3_shift ||
        joint_4_angle_old != joint_4_angle)
    {

        float px = a1 * cos(joint_1_angle) + a2 * cos(joint_1_angle + joint_2_angle);
        float py = a1 * sin(joint_1_angle) + a2 * sin(joint_1_angle + joint_2_angle);
        float pz = -joint_3_shift;
        float teta_z = joint_1_angle + joint_2_angle - joint_4_angle;

        joint_1_angle_old = joint_1_angle;
        joint_2_angle_old = joint_2_angle;
        joint_3_shift_old = joint_3_shift;
        joint_4_angle_old = joint_4_angle;

        ROS_INFO("Position: [%.3f, %.3f, %.3f]\n", px, py, pz);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "f_kin");

    ros::NodeHandle fkin;

    ros::Subscriber readAngles = fkin.subscribe("/joint_states", 1000, f_kinematic);

    ros::spin();
}