
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <nao_msgs/JointAnglesWithSpeed.h>

#include <tf/transform_datatypes.h>

#include <ros/ros.h>
#include <math.h>

enum ProgramStatus {PLAYBACK, EXERCISING, PROMPT, COMPLETE};

void orthonormalize(tf::Matrix3x3& matrix) {

    float ux = 0.0F;
    float uy = 0.0F;
    float uz = 0.0F;
    float vx = 0.0F;
    float vy = 0.0F;
    float vz = 0.0F;

    for(int i = 0; i < 3; ++i) {
        for(int vt = 0; vt < i; ++vt) {
            ux = matrix[0][i];
            uy = matrix[1][i];
            uz = matrix[2][i];
            vx = matrix[0][vt];
            vy = matrix[1][vt];
            vz = matrix[2][vt];
            float ut = ux * vx + uy * vy + uz * vz;
            matrix[0][vt] -= ux * ut;
            matrix[1][vt] -= uy * ut;
            matrix[2][vt] -= uz * ut;
        }

        ux = matrix[0][i];
        uy = matrix[1][i];
        uz = matrix[2][i];
        float var10 = 1.0F / (float)sqrt((double)(ux * ux + uy * uy + uz * uz));
        matrix[0][i] *= var10;
        matrix[1][i] *= var10;
        matrix[2][i] *= var10;
    }
}

float lastRot, lastSin, lastCos;

tf::Matrix3x3 rotateAxis(tf::Vector3 axis, tf::Matrix3x3& original, float angle) {
        
    if(angle != lastRot) {
        lastRot = angle;
        lastSin = (float)sin((double)angle);
        lastCos = (float)cos((double)angle);
    }

    float c = lastCos;
    float s = lastSin;
    float t = 1.0F - c;
    //axis = axis.normalize(axis);
    float x = axis.x();
    float y = axis.y();
    float z = axis.z();
    
    tf::Matrix3x3 mat;
    mat.setIdentity();
    float sy = s * y;
    float sx = s * x;
    float sz = s * z;
    float txy = t * x * y;
    float txz = t * x * z;
    float tyz = t * y * z;
    mat[0][0] = t * x * x + c;
    mat[1][0] = txy + sz;
    mat[2][0] = txz - sy;
    mat[0][1] = txy - sz;
    mat[1][1] = t * y * y + c;
    mat[2][1] = tyz + sx;
    mat[0][2] = txz + sy;
    mat[1][2] = tyz - sx;
    mat[2][2] = t * z * z + c;
    orthonormalize(mat);
    original *= mat;
    
    return original; 
}

tf::Vector3 rotate(tf::Vector3 vec, tf::Matrix3x3& mat) {
    float xr = vec.getX() * mat[0][0] + vec.getY() * mat[1][0] + vec.getZ() * mat[2][0];
    float yr = vec.getX() * mat[0][1] + vec.getY() * mat[1][1] + vec.getZ() * mat[2][1];
    float zr = vec.getX() * mat[0][2] + vec.getY() * mat[1][2] + vec.getZ() * mat[2][2];
    vec.setX(xr);
    vec.setY(yr);
    vec.setZ(zr);

    return vec;
}              

float calcAngle(tf::Vector3 v1, tf::Vector3 v2) {
    float dot = v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
    float lt = v1.x() * v1.x() + v1.y() * v1.y() + v1.z() * v1.z();
    float lv = v2.x() * v2.x() + v2.y() * v2.y() + v2.z() * v2.z();
    dot /= sqrt(lt * lv);
    if(dot < -1.0f) {
        dot = -1.0f;
    } else if(dot > 1.0f) {
        dot = 1.0f;
    } 
    
    return acos(dot); 
}

float magnitudeSquared(tf::Quaternion q) {
    return q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
}

tf::Matrix3x3 quatToMatrix(tf::Quaternion q) {
    tf::Matrix3x3 matrix; 
    float norm = magnitudeSquared(q);
    float s = (double)norm > 0.0?2.0f / norm:0.0F;
    float xs = q.x() * s;
    float ys = q.y() * s;
    float zs = q.z() * s;
    float xx = q.x() * xs;
    float xy = q.x() * ys;
    float xz = q.x() * zs;
    float xw = q.w() * xs;
    float yy = q.y() * ys;
    float yz = q.y() * zs;
    float yw = q.w() * ys;
    float zz = q.z() * zs;
    float zw = q.w() * zs;
    matrix[0][0] = 1.0F - (yy + zz);
    matrix[1][0] = xy - zw;
    matrix[2][0] = xz + yw;
    matrix[0][1] = xy + zw;
    matrix[1][1] = 1.0F - (xx + zz);
    matrix[2][1] = yz - xw;
    matrix[0][2] = xz - yw;
    matrix[1][2] = yz + xw;
    matrix[2][2] = 1.0F - (xx + yy);

    return matrix;
}

class NaoExercise {

private:
    ProgramStatus status;

    ros::NodeHandle n;
    nao_msgs::JointAnglesWithSpeed nao_joint_msg;
    ros::Subscriber myo_l_sub, myo_u_sub, playback_sub, progress_sub;
    ros::Publisher joint_pub, playback_trigger, speech_pub;
    ros::ServiceClient stiffness_client, rest_position_client;

    std_msgs::String greeting_msg, yourturn_msg, retry_msg, prompt_msg, complete_msg;
    geometry_msgs::Quaternion upper_myo_msg;
    const static int END_FLAG;

public:
    NaoExercise();
    void pubQuat2NaoArm(geometry_msgs::Quaternion);
    void myo_l_callback(geometry_msgs::Quaternion);
    void myo_u_callback(geometry_msgs::Quaternion);
    void progress_callback(std_msgs::Float64);
    void playback_u_callback(geometry_msgs::Quaternion);
    void playback_l_callback(geometry_msgs::Quaternion);
    void begin();

    ProgramStatus getStatus();
};

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
        tf::Matrix3x3 m = quatToMatrix(q);
        
        tf::Vector3 vec(-1, 0, 0), origin, down(0, 0, -1);
        vec = rotate(vec, m);

        vec.setY(vec.y());
        vec.setZ(vec.z());

    //    ROS_INFO("{%f, %f, %f}", vec.getX(), vec.getY(), vec.getZ());

        float h = 1;
        
     //   ROS_INFO("%f", h);
    
        tf::Vector3 parallelToPlane(0, vec.getY(), -(sqrt(pow(h, 2) - pow(vec.getY(), 2))) );
        double roll, pitch, yaw;
    
      //  ROS_INFO("{%f, %f, %f}", parallelToPlane.getX(), parallelToPlane.getY(), parallelToPlane.getZ());
        
        pitch = calcAngle(parallelToPlane, vec);
        yaw = calcAngle(parallelToPlane, down); 
       
        pitch = -pitch;
        yaw = -yaw + 1.1f;

    //    ROS_INFO("y_%f p_%f r_%f", yaw, pitch, roll);
        /*
        if(2*fabs(msg.y*msg.w + msg.x*msg.z) >= 1)
            ROS_INFO("WAHHHH");

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
    tf::Matrix3x3 m = quatToMatrix(q);
   
    tf::Vector3 yAxis(0, 1, 0), xAxis(1, 0, 0);

    
    tf::Matrix3x3 n;
    n.setIdentity();
    rotateAxis(yAxis, n, nao_joint_msg.joint_angles[4]);
    rotateAxis(xAxis, n, nao_joint_msg.joint_angles[3]);
    n = n.inverse();
    m *= n;

        tf::Vector3 vec(0, 1, 0), origin, down(0, 1, 0);
        vec = rotate(vec, m);

    //ROS_INFO("{%f, %f, %f}", vec.getX(), vec.getY(), vec.getZ());

        float h = 1;
        
     //   ROS_INFO("%f", h);
    
        tf::Vector3 parallelToPlane(0, vec.getY(), -(sqrt(pow(h, 2) - pow(vec.getY(), 2))) );
        double roll, pitch, yaw;
    
        //ROS_INFO("{%f, %f, %f}", parallelToPlane.getX(), parallelToPlane.getY(), parallelToPlane.getZ());
        
        pitch = calcAngle(parallelToPlane, vec);
        yaw = calcAngle(parallelToPlane, down); 
    
        pitch = -pitch + 1.5;
        yaw = -yaw + 2.5;

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

    ROS_INFO("y_%f p_%f r_%f", yaw, pitch, roll);

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

    playback_sub = n.subscribe<geometry_msgs::Quaternion>("/exercise/playback", 10, &NaoExercise::playback_callback, this);
    progress_sub = n.subscribe<std_msgs::Float64>("/exercise/progress", 10, &NaoExercise::progress_callback, this);
    playback_trigger = n.advertise<std_msgs::Empty>("/exercise/playback_trigger", 100);
    joint_pub = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);
    speech_pub = n.advertise<std_msgs::String>("/speech", 100);
    stiffness_client = n.serviceClient<std_srvs::Empty>("/body_stiffness/enable");
    rest_position_client = n.serviceClient<std_srvs::Empty>("/nao_robot/pose/rest");
    
    begin();
}
