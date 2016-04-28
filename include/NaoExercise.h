#ifndef NAOEXERCISE_H
#define NAOEXERCISE_H

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <nao_msgs/JointAnglesWithSpeed.h>

#include <tf/transform_datatypes.h>

#include <pthread.h>
#include <math.h>
#include "JPCT_Math.h"

enum ProgramStatus {PLAYBACK, EXERCISING, PROMPT, COMPLETE, LOCKED};

class NaoExercise {

private:
    ProgramStatus status;

    ros::NodeHandle n;
    nao_msgs::JointAnglesWithSpeed nao_joint_msg;
    ros::Subscriber myo_l_sub, myo_u_sub, playback_l_sub, playback_u_sub, progress_sub, mode_sub;
    ros::Publisher joint_pub, speech_pub;
    ros::ServiceClient stiffness_enable_client, stiffness_disable_client, rest_position_client;

    std_msgs::String greeting_msg, yourturn_msg, retry_msg, prompt_msg, complete_msg;
    geometry_msgs::Quaternion upper_myo_msg;
    const static int END_FLAG;

public:
    NaoExercise();
    void pubQuat2NaoArm(geometry_msgs::Quaternion);
    void myo_l_callback(geometry_msgs::Quaternion);
    void myo_u_callback(geometry_msgs::Quaternion);
    void mode_callback(std_msgs::Int32);
    void progress_callback(std_msgs::Float64);
    void playback_u_callback(geometry_msgs::Quaternion);
    void playback_l_callback(geometry_msgs::Quaternion);
    
    void begin();
    void end();
    void posture_setup();

    ProgramStatus getStatus();
};

#endif
