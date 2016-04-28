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
        
        tf::Vector3 parallelToPlane(0, vec.getY(), -(sqrt(pow(h, 2) - pow(vec.getY(), 2))) );
        double roll, pitch, yaw;
    
        pitch = JPCT_Math::calcAngle(parallelToPlane, vec);
        yaw = JPCT_Math::calcAngle(parallelToPlane, down); 
       
        pitch = pitch * (vec.x() > 0 ? 1 : -1);

        yaw = -yaw + 1.1f;

        ROS_INFO("y_%f p_%f r_%f", yaw, pitch, roll);

        nao_joint_msg.joint_angles[3] = yaw;
        nao_joint_msg.joint_angles[4] = pitch;

    }
}

void NaoExercise::mode_callback(std_msgs::Int32 mode) {
    if(status == LOCKED && mode.data != 3)
        return;
    
    switch(mode.data) {
        case 0:
            begin();    
            break;

        case 1:
            status = EXERCISING;
            myo_l_sub = n.subscribe<geometry_msgs::Quaternion>("/myo/l/ort", 10, &NaoExercise::myo_l_callback, this);
            myo_u_sub = n.subscribe<geometry_msgs::Quaternion>("/myo/u/ort", 10, &NaoExercise::myo_u_callback, this);
            break;

        case 3: //unlock the program
            status = EXERCISING;
            posture_setup();
            break;

        case 4: //lock the program
            status = LOCKED;
            end();
            break;
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

    float h = 1;
    
    tf::Vector3 parallelToPlane(0, vec.getY(), -(sqrt(pow(h, 2) - pow(vec.getY(), 2))) );
    double roll, pitch, yaw;

    pitch = JPCT_Math::calcAngle(parallelToPlane, vec);
    yaw = JPCT_Math::calcAngle(parallelToPlane, down); 

    pitch = -pitch + 1.5;
    pitch *= 1.3;
    yaw = -yaw + 2;
    yaw *= 1.3;
    
    nao_joint_msg.joint_angles[0] = yaw;
    nao_joint_msg.joint_angles[1] = pitch;
    nao_joint_msg.joint_angles[2] = roll;

    joint_pub.publish(nao_joint_msg);
}

void NaoExercise::begin() {
    playback_l_sub = n.subscribe<geometry_msgs::Quaternion>("/exercise/l/playback", 10, &NaoExercise::playback_l_callback, this);
    playback_u_sub = n.subscribe<geometry_msgs::Quaternion>("/exercise/u/playback", 10, &NaoExercise::playback_u_callback, this);

    posture_setup();

    speech_pub.publish(greeting_msg);
    
    status = PLAYBACK;
}

void NaoExercise::posture_setup() {
    std_srvs::Empty stiffness_srv;
    if(!stiffness_enable_client.call(stiffness_srv)) {
        ROS_ERROR("failed to call stiffen service");
    }
   
    nao_msgs::JointAnglesWithSpeed shoulder_msg;
    shoulder_msg.joint_names.push_back("RShoulderRoll");
    shoulder_msg.joint_angles.push_back(-.6);
    shoulder_msg.speed = 0.5;

    joint_pub.publish(shoulder_msg);
}

void NaoExercise::end() {
    playback_l_sub.shutdown();
    playback_u_sub.shutdown();
    
    myo_l_sub.shutdown();
    myo_u_sub.shutdown();

    nao_joint_msg.joint_angles[0] = 0;
    nao_joint_msg.joint_angles[1] = 0;
    nao_joint_msg.joint_angles[2] = 0;
    nao_joint_msg.joint_angles[3] = M_PI/2;
    nao_joint_msg.joint_angles[4] = 0;
    
    joint_pub.publish(nao_joint_msg);

    sleep(0.5);

    std_srvs::Empty stiffness_srv;
    if(!stiffness_disable_client.call(stiffness_srv)) {
        ROS_ERROR("failed to call stiffen service");
    }
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

    mode_sub = n.subscribe<std_msgs::Int32>("/exercise/mode", 10, &NaoExercise::mode_callback, this);
    progress_sub = n.subscribe<std_msgs::Float64>("/exercise/progress", 10, &NaoExercise::progress_callback, this);
    joint_pub = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);
    speech_pub = n.advertise<std_msgs::String>("/speech", 100);
    stiffness_enable_client = n.serviceClient<std_srvs::Empty>("/body_stiffness/enable");
    stiffness_disable_client = n.serviceClient<std_srvs::Empty>("/body_stiffness/disable");
}
