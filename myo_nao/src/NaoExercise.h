#ifndef NAOEXERCISE_H
#define NAOEXERCISE_H

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <nao_msgs/JointAnglesWithSpeed.h>

#include <tf/transform_datatypes.h>

#include <math.h>

class NaoExercise {

private:
    enum ProgramStatus {PLAYBACK, EXERCISING, COMPLETE};
    ProgramStatus status;

    ros::NodeHandle n;
    nao_msgs::JointAnglesWithSpeed nao_joint_msg;
    ros::Subscriber myo_sub, playback_sub;
    ros::Publisher joint_pub, speech_pub, playback_trigger;
    
    const static int END_FLAG = -1337;

public:
    NaoExercise();
    void pubQuat2NaoArm(geometry_msgs::Quaternion);
    void myo_callback(geometry_msgs::Quaternion);
    void playback_callback(geometry_msgs::Quaternion);
    void begin();

}; 

#endif
