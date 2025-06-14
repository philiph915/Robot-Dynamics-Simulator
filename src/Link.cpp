#include "Link.h"

void Link::Render() {

    // Set some constants for drawing
    float sphereRadius = 5.0f;
    Color linkColor = PURPLE;
    Color jointColor = BLUE;

    // Convert Eigen::Vector3f to raylib Vector3
    Vector3 pos = {position(0), position(1), position(2)};

    Eigen::Vector3f nextPos_eigen = position + R_0_i*r_i_1;
    Vector3 nextPos = {nextPos_eigen(0), nextPos_eigen(1), nextPos_eigen(2)};
    
    // Draw a line along the link axis
    CameraUtils::DrawCylinderBetween(pos,nextPos,2.5f,linkColor,10);
    // DrawLine3D(pos, nextPos, linkColor);

    // Draw a sphere at the origin of the link
    DrawSphere(pos, sphereRadius, jointColor);

    DrawLocalAxes();

}

void Link::DrawLocalAxes()
{
     Vector3 pos = {position(0), position(1), position(2)};
     Eigen::Vector3f worldX;         Eigen::Vector3f localX;
     Eigen::Vector3f worldY;         Eigen::Vector3f localY;
     Eigen::Vector3f worldZ;         Eigen::Vector3f localZ;
     Eigen::Vector3f world_x_axis_moved;
     Eigen::Vector3f world_y_axis_moved;
     Eigen::Vector3f world_z_axis_moved;
     localX << 1,0,0;
     localY << 0,1,0;
     localZ << 0,0,1;

    float axisLength = 40.0f;

    // Rotate local unit vectors into the world frame
    worldX = R_0_i*localX;
    worldY = R_0_i*localY;
    worldZ = R_0_i*localZ;

    // scale by axis length and shift rotated vectors to the link origin
    world_x_axis_moved = position + worldX * axisLength;
    world_y_axis_moved = position + worldY * axisLength;
    world_z_axis_moved = position + worldZ * axisLength;

    Vector3 x_draw = {world_x_axis_moved(0), world_x_axis_moved(1), world_x_axis_moved(2)};
    Vector3 y_draw = {world_y_axis_moved(0), world_y_axis_moved(1), world_y_axis_moved(2)};
    Vector3 z_draw = {world_z_axis_moved(0), world_z_axis_moved(1), world_z_axis_moved(2)};

    DrawLine3D(pos, x_draw, RED);
    DrawLine3D(pos, y_draw, GREEN);
    DrawLine3D(pos, z_draw, BLUE);

}