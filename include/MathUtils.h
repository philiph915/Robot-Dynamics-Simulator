#include <Eigen/Dense>
#include <cmath>

namespace utils {

    // custom typedef's
    using Matrix6f = Eigen::Matrix<float, 6, 6>;
    using Vector6f = Eigen::Matrix<float, 6, 1>;

    // Joint subspaces
    Vector6f unitTwist_revoluteZ();
    Vector6f unitTwist_prismaticZ();

    // 3D Rotation matrices
    Eigen::Matrix3f rotX(float theta); 
    Eigen::Matrix3f rotY(float theta);
    Eigen::Matrix3f rotZ(float theta);

    // Skew symmetry and cross-products
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f &v);

    // Conversions
    float Deg2Rad(float deg);
    float Rad2Deg(float rad);

    // Spatial transforms and spatial inertia
    Matrix6f spatialTransformTwist(Eigen::Matrix3f R_ab, Eigen::Vector3f p_ab);
    Matrix6f spatialTransformWrench(Eigen::Matrix3f R_ab, Eigen::Vector3f p_ab);
    Matrix6f spatialInertiaMatrix(Eigen::Matrix3f I_com, float mass, Eigen::Vector3f r_cg);
};
