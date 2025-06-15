#include "MathUtils.h"
#include <cmath> // explicitly included for trig functions

// Rotation matrix about X
Eigen::Matrix3f utils::rotX(float theta)
{
    Eigen::Matrix3f R_x;
    R_x << 1,          0,           0,
       0, cos(theta), -sin(theta),
       0, sin(theta),  cos(theta);

    return R_x;
}

// Rotation matrix about Y
Eigen::Matrix3f utils::rotY(float theta)
{
    Eigen::Matrix3f R_y;
    R_y << cos(theta),  0, sin(theta),
                  0,    1,         0,
          -sin(theta),  0, cos(theta);

    return R_y;
}

// Rotation matrix about Z
Eigen::Matrix3f utils::rotZ(float theta)
{
    Eigen::Matrix3f R_z;
    R_z << cos(theta), -sin(theta), 0,
           sin(theta),  cos(theta), 0,
                   0,           0, 1;

    return R_z;
}

// construct skew-symmetric matrix from a vector
Eigen::Matrix3f utils::skewSymmetric(const Eigen::Vector3f& v)
{
    Eigen::Matrix3f S;
    S <<    0,    -v(2),  v(1),
          v(2),     0,   -v(0),
         -v(1),   v(0),    0;
    return S;
}

float utils::Deg2Rad(float deg)
{
    return deg * M_PI / 180;
}

float utils::Rad2Deg(float rad)
{
    return rad * 180 / M_PI;
}

Vector3 utils::ConvertEigen2Raylib_Vector3(Eigen::Vector3f vector_Eigen)
{
    return Vector3{vector_Eigen(0), vector_Eigen(1), vector_Eigen(2)};
}

// Joint subspace for revolute joint about z axis
utils::Vector6f utils::unitTwist_revoluteZ()
{
    utils::Vector6f revoluteZ;
    revoluteZ << 0, 0, 1, 0, 0, 0;
    return revoluteZ;
}

// Joint subspace for prismatic joint about x axis
utils::Vector6f utils::unitTwist_prismaticZ()
{
    utils::Vector6f prismaticZ;
    prismaticZ << 0, 0, 0, 1, 0, 0;
    return prismaticZ;
}

// Spatial transform matrix for twists (velocities)
utils::Matrix6f utils::spatialTransformTwist(Eigen::Matrix3f R_ab, Eigen::Vector3f p_ab)
{
    Matrix6f X_ab = Matrix6f::Zero(); // initialize to all zeros

    // Populate corners with rotation matrix
    X_ab.block<3,3>(0,0) = R_ab;
    X_ab.block<3,3>(3,3) = R_ab;
    X_ab.block<3,3>(3,0) = utils::skewSymmetric(p_ab) * R_ab;

    return X_ab;
}

// Spatial transform matrix for wrench (force/torque)
utils::Matrix6f utils::spatialTransformWrench(Eigen::Matrix3f R_ab, Eigen::Vector3f p_ab)
{
    Matrix6f X_ab = Matrix6f::Zero(); // initialize to all zeros

    // Populate corners with rotation matrix
    X_ab.block<3,3>(0,0) = R_ab;
    X_ab.block<3,3>(3,3) = R_ab;
    X_ab.block<3,3>(0,3) = utils::skewSymmetric(p_ab) * R_ab;

    return X_ab;
}

// Spatial inertia matrix
utils::Matrix6f utils::spatialInertiaMatrix(Eigen::Matrix3f I_com, float mass, Eigen::Vector3f r_cg)
{
    Matrix6f I_spatial = Matrix6f::Zero(); // initialize to all zeros

    Eigen::Matrix3f r_cg_skew = utils::skewSymmetric(r_cg);

    I_spatial.block<3,3>(0,0) = I_com + mass * r_cg_skew * r_cg_skew; 
    I_spatial.block<3,3>(3,3) = mass*Eigen::Matrix3f::Identity(); 
    I_spatial.block<3,3>(0,3) = mass*r_cg_skew;
    I_spatial.block<3,3>(3,0) = -mass*r_cg_skew;

    return I_spatial;
}