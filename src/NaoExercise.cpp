#include "../include/NaoExercise.h"

const int NaoExercise::END_FLAG = -1337;

ProgramStatus NaoExercise::getStatus() {
    return status;
}

void NaoExercise::myo_l_callback(geometry_msgs::Quaternion msg) {
    if(status == EXERCISING) {
        
        pubQuat2NaoArm(msg);
    }
}

void NaoExercise::myo_u_callback(geometry_msgs::Quaternion msg) {
    if(status == EXERCISING) {
    
        tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
        tf::Matrix3x3 m = JPCT_Math::quatToMatrix(q);
        
        tf::Vector3 vec(-1, 0, 0), origin, down(0, 0, -1);
        vec = JPCT_Math::rotate(vec, m);

        vec.setY(vec.y());
        vec.setZ(vec.z());

        ROS_INFO("{%f, %f, %f}", vec.getX(), vec.getY(), vec.getZ());

        float h = 1;
        
     //   ROS_INFO("%f", h);
    
        tf::Vector3 parallelToPlane(0, vec.getY(), -(sqrt(pow(h, 2) - pow(vec.getY(), 2))) );
        double roll, pitch, yaw;
    
      //  ROS_INFO("{%f, %f, %f}", parallelToPlane.getX(), parallelToPlane.getY(), parallelToPlane.getZ());
        
        pitch = JPCT_Math::calcAngle(parallelToPlane, vec);
        yaw = JPCT_Math::calcAngle(parallelToPlane, down); 
       
        pitch = pitch * (vec.x() > 0 ? 1 : -1);

        yaw = -yaw + 1.1f;

        ROS_INFO("y_%f p_%f r_%f", yaw, pitch, roll);

        /*
        yaw =   atan2((msg.z * msg.w + msg.x * msg.y), 1/2 - (pow(msg.y, 2) + pow(msg.z, 2)) ); 
        pitch = asin(-2*(msg.y*msg.w - msg.x * msg.z)) - 1.24;
        roll = atan2( (msg.y*msg.z + msg.x*msg.w), 1/2 - (pow(msg.z, 2) + pow(msg.w, 2)) );
        */

        nao_joint_msg.joint_angles[3] = yaw;
        nao_joint_msg.joint_angles[4] = pitch;

    }
}

void NaoExercise::progress_callback(std_msgs::Float64 progress) {
    if(status != COMPLETE && progress.data == 1) {
        status = COMPLETE;
        speech_pub.publish(complete_msg);

        std_srvs::Empty rest_srv;
        if(!rest_position_client.call(rest_srv)) {
            ROS_ERROR("failed to call rest service");
        }
    }
}

void NaoExercise::playback_u_callback(geometry_msgs::Quaternion msg) {
    
    myo_u_callback(msg);

}

void NaoExercise::playback_l_callback(geometry_msgs::Quaternion msg) {

    if(status == EXERCISING) {
        myo_l_sub.shutdown();
        myo_u_sub.shutdown();
        speech_pub.publish(prompt_msg);
        status = PROMPT;
    }

    if(msg.x == END_FLAG || msg.y == END_FLAG || msg.z == END_FLAG || msg.w == END_FLAG) {
        speech_pub.publish((status == PLAYBACK ? yourturn_msg : retry_msg));
        myo_l_sub = n.subscribe<geometry_msgs::Quaternion>("/myo/l/ort", 10, &NaoExercise::myo_l_callback, this);
        myo_u_sub = n.subscribe<geometry_msgs::Quaternion>("/myo/u/ort", 10, &NaoExercise::myo_u_callback, this);
        status = EXERCISING;
    } else {
        pubQuat2NaoArm(msg);
    }
}

void NaoExercise::pubQuat2NaoArm(geometry_msgs::Quaternion msg) {

    tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
    tf::Matrix3x3 m = JPCT_Math::quatToMatrix(q);
   
    tf::Vector3 yAxis(0, 1, 0), xAxis(1, 0, 0);

    
    tf::Matrix3x3 n;
    n.setIdentity();
    JPCT_Math::rotateAxis(yAxis, n, nao_joint_msg.joint_angles[4]);
    JPCT_Math::rotateAxis(xAxis, n, nao_joint_msg.joint_angles[3]);
    n = n.inverse();
    m *= n;

    tf::Vector3 vec(0, 1, 0), origin, down(0, 1, 0);
    vec = JPCT_Math::rotate(vec, m);

    //ROS_INFO("{%f, %f, %f}", vec.getX(), vec.getY(), vec.getZ());

    float h = 1;
        
     //   ROS_INFO("%f", h);
    
    tf::Vector3 parallelToPlane(0, vec.getY(), -(sqrt(pow(h, 2) - pow(vec.getY(), 2))) );
    double roll, pitch, yaw;

    //ROS_INFO("{%f, %f, %f}", parallelToPlane.getX(), parallelToPlane.getY(), parallelToPlane.getZ());
    
    pitch = JPCT_Math::calcAngle(parallelToPlane, vec);
    yaw = JPCT_Math::calcAngle(parallelToPlane, down); 

    pitch = -pitch + 1.5;
    pitch *= 1.3;
    yaw = -yaw + 2;
    yaw *= 1.3;
    /*
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaw = -yaw;
    roll = -M_PI*.4 -roll;

    if(yaw > 119.5 * M_PI/180) 
        yaw = 119.5 * M_PI/180;
    else if (yaw < 10 * M_PI/180) {
        if(fabs(10 * M_PI/180 - yaw) < fabs(-M_PI - yaw))
            yaw = 10 * M_PI/180;
        else
            yaw = 119.5 * M_PI/180; 
    }

    if(pitch > 88.5 * M_PI/180)
        pitch = 88.5 * M_PI/180;
    if(pitch < 2 * M_PI/180)
        pitch = 2 * M_PI/180;

    if(roll > 104.5 * M_PI/180)
       roll = 104.5 * M_PI/180;
    else if(roll < -104.5 * M_PI/180)
       roll = -104.5 * M_PI/180; 

*/

    nao_joint_msg.joint_angles[0] = yaw;
    nao_joint_msg.joint_angles[1] = pitch;
    nao_joint_msg.joint_angles[2] = roll;

    //ROS_INFO("y_%f p_%f r_%f", yaw, pitch, roll);

    joint_pub.publish(nao_joint_msg);
}


void NaoExercise::begin() {
    ROS_INFO("in begin");
    
    std_srvs::Empty stiffness_srv;
    if(!stiffness_client.call(stiffness_srv)) {
        ROS_ERROR("failed to call stiffen service");
    }
   
    nao_msgs::JointAnglesWithSpeed shoulder_msg;
    shoulder_msg.joint_names.push_back("RShoulderRoll");
    shoulder_msg.joint_angles.push_back(-.6);
    shoulder_msg.speed = 0.5;

    joint_pub.publish(shoulder_msg);

    speech_pub.publish(greeting_msg);
    
    std_msgs::Empty triggerer;
    playback_trigger.publish(triggerer);
    
    status = PLAYBACK;
}

NaoExercise::NaoExercise() {
    greeting_msg.data = "Hello, I am now doing the exercise for today"; 
    yourturn_msg.data = "Now it is your turn to exercise, I will copy your movement";
    prompt_msg.data = "You seem to be having a problem, let me demonstrate";
    retry_msg.data = "Ok, you can continue your exercise now.";
    complete_msg.data = "You have finished the exercise, great job!";

    nao_joint_msg.joint_names.push_back("RElbowYaw"); //yaw
    nao_joint_msg.joint_names.push_back("RElbowRoll"); //pitch
    nao_joint_msg.joint_names.push_back("RWristYaw"); //roll

    nao_joint_msg.joint_names.push_back("RShoulderPitch"); //yaw
    nao_joint_msg.joint_names.push_back("RShoulderRoll"); //pitch

    nao_joint_msg.joint_angles = std::vector<float>(5, 0);
    nao_joint_msg.joint_angles[3] = 1.1;
    nao_joint_msg.joint_angles[4] = -0.5981317008;
    nao_joint_msg.speed = 0.5;

    playback_l_sub = n.subscribe<geometry_msgs::Quaternion>("/exercise/l/playback", 10, &NaoExercise::playback_l_callback, this);
    playback_u_sub = n.subscribe<geometry_msgs::Quaternion>("/exercise/u/playback", 10, &NaoExercise::playback_u_callback, this);
    progress_sub = n.subscribe<std_msgs::Float64>("/exercise/progress", 10, &NaoExercise::progress_callback, this);
    playback_trigger = n.advertise<std_msgs::Empty>("/exercise/playback_trigger", 100);
    joint_pub = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);
    speech_pub = n.advertise<std_msgs::String>("/speech", 100);
    stiffness_client = n.serviceClient<std_srvs::Empty>("/body_stiffness/enable");
    rest_position_client = n.serviceClient<std_srvs::Empty>("/nao_robot/pose/rest");
    
    begin();
}
