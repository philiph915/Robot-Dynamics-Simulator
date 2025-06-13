#pragma once
#include <Eigen/Dense>
#include "MathUtils.h"

struct Link {

public:

// Joint states
float q_i   = 0.0f;   // joint angle (position)
float dq_i  = 0.0f;   // joint velocity
float ddq_i = 0.0f;   // joint acceleration

// Mass props
float mass = 1.0f;
Eigen::Matrix3f I_com   = Eigen::Matrix3f::Identity();  // Inertia tensor about the link COM
Eigen::Vector3f r_cg    = Eigen::Vector3f::Zero();      // Vector from link origin to link COM 
Eigen::Vector3f r_i_1   = Eigen::Vector3f::Zero();      // Vector from link origin to link 1+i origin
Eigen::Vector3f r_1_cg  = Eigen::Vector3f::Zero();      // Vector from link i+1 origin to link i COM

// Position and orientation
Eigen::Vector3f position = Eigen::Vector3f::Zero();       // absolute position of link i origin in world coordinates
Eigen::Matrix3f R_1_i    = Eigen::Matrix3f::Identity();   // R_i+1 to i
Eigen::Matrix3f R_i      = Eigen::Matrix3f::Identity();   // R_i-1 to i
Eigen::Matrix3f R_0_i    = Eigen::Matrix3f::Identity();   // Absolute position, i.e. rotation from frame 0 to i

// Velocities
Eigen::Vector3f omega_i = Eigen::Vector3f::Zero(); // angular velocity of link i

// Accelerations
Eigen::Vector3f alpha_i = Eigen::Vector3f::Zero();   // angular acceleration of link i
Eigen::Vector3f a_ci    = Eigen::Vector3f::Zero();   // linear acceleration of link i COM
Eigen::Vector3f a_ei    = Eigen::Vector3f::Zero();   // linear acceleration of the end of link i

// Forces and Torques
Eigen::Vector3f g_i      = Eigen::Vector3f::Zero(); // gravity force acting on link i
Eigen::Vector3f f_i      = Eigen::Vector3f::Zero(); // force exerted on link i by link i-1
Eigen::Vector3f f_iPlus1 = Eigen::Vector3f::Zero(); // force exerted on link i by link i+1
Eigen::Vector3f tau_i    = Eigen::Vector3f::Zero(); // torque exerted on link i by link i-1

// CRBA parameters
utils::Vector6f S_i;  // joint subspace (unit twist) — defines joint motion direction
utils::Matrix6f I_sp; // spatial inertia matrix of this link

// Additional Spatial parameters to be implemented later
utils::Vector6f v_i; // spatial velocity of link i
utils::Vector6f a_i; // spatial acceleration of link i

private:

};