#ifndef JPCT_MATH_H
#define JPCT_MATH_H

//this code is mostly functions I (Christopher Munroe) have translated from the JPCT library from Java to C++
//for more details about these functions google JPCT's version

#include <math.h>
#include <tf/transform_datatypes.h>

class JPCT_Math {
private:
    static float lastRot, lastSin, lastCos;

public:
    static void orthonormalize(tf::Matrix3x3& matrix);
    static tf::Matrix3x3 rotateAxis(tf::Vector3 axis, tf::Matrix3x3& original, float angle);
    static tf::Vector3 rotate(tf::Vector3 vec, tf::Matrix3x3& mat);
    static float calcAngle(tf::Vector3 v1, tf::Vector3 v2);
    static float magnitudeSquared(tf::Quaternion q);
    static tf::Matrix3x3 quatToMatrix(tf::Quaternion q);
};

#endif
