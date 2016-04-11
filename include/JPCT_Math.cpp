#include "JPCT_Math.h"

float JPCT_Math::lastRot = 0, JPCT_Math::lastSin = 0, JPCT_Math::lastCos = 0;

void JPCT_Math::orthonormalize(tf::Matrix3x3& matrix) {

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

tf::Matrix3x3 JPCT_Math::rotateAxis(tf::Vector3 axis, tf::Matrix3x3& original, float angle) {
        
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

tf::Vector3 JPCT_Math::rotate(tf::Vector3 vec, tf::Matrix3x3& mat) {
    float xr = vec.getX() * mat[0][0] + vec.getY() * mat[1][0] + vec.getZ() * mat[2][0];
    float yr = vec.getX() * mat[0][1] + vec.getY() * mat[1][1] + vec.getZ() * mat[2][1];
    float zr = vec.getX() * mat[0][2] + vec.getY() * mat[1][2] + vec.getZ() * mat[2][2];
    vec.setX(xr);
    vec.setY(yr);
    vec.setZ(zr);

    return vec;
}              

float JPCT_Math::calcAngle(tf::Vector3 v1, tf::Vector3 v2) {
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

float JPCT_Math::magnitudeSquared(tf::Quaternion q) {
    return q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
}

tf::Matrix3x3 JPCT_Math::quatToMatrix(tf::Quaternion q) {
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
