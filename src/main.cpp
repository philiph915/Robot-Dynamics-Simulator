// main.cpp
#include <iostream>
#include <Eigen/Dense>
#include "raylib.h"
#include "raymath.h"
#include "Robot.h"

void UpdateFreeCamera(Camera3D* cam, float moveSpeed, float rotateSpeed)
{

    if (IsKeyPressed(KEY_ONE)) { // XY plane
        cam->position = (Vector3){ 0, 0, 500 };
        cam->up = (Vector3){ 0, 1, 0 };
    }
    if (IsKeyPressed(KEY_TWO)) { // XZ plane
        cam->position = (Vector3){ 0, 500, 0 };
        cam->up = (Vector3){ 0, 0, -1 };
    }
    if (IsKeyPressed(KEY_THREE)) { // YZ plane
        cam->position = (Vector3){ 500, 0, 0 };
        cam->up = (Vector3){ 0, 1, 0 };
    }


    // Move camera with arrow keys
    if (IsKeyDown(KEY_RIGHT)) cam->position.x += moveSpeed;
    if (IsKeyDown(KEY_LEFT))  cam->position.x -= moveSpeed;
    if (IsKeyDown(KEY_UP))    cam->position.z -= moveSpeed;
    if (IsKeyDown(KEY_DOWN))  cam->position.z += moveSpeed;

    // Move up/down with Q/E
    if (IsKeyDown(KEY_Q)) cam->position.y += moveSpeed;
    if (IsKeyDown(KEY_E)) cam->position.y -= moveSpeed;

    // Mouse rotation (hold right mouse button)
    if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
    {
        Vector2 delta = GetMouseDelta();

        float angleYaw   = -delta.x * rotateSpeed;
        float anglePitch = -delta.y * rotateSpeed;

        // Simple rotation around target
        Vector3 offset = Vector3Subtract(cam->position, cam->target);

        Matrix matYaw = MatrixRotateY(angleYaw);
        Matrix matPitch = MatrixRotateX(anglePitch);

        // Apply yaw first, then pitch
        offset = Vector3Transform(offset, matYaw);
        offset = Vector3Transform(offset, matPitch);

        cam->position = Vector3Add(cam->target, offset);
    }
}

void DrawThickAxis(Vector3 start, Vector3 end, Color color, float thickness)
{
    Vector3 axis = Vector3Subtract(end, start);
    float length = Vector3Length(axis);
    Vector3 center = Vector3Lerp(start, end, 0.5f);

    Vector3 size = {
        (fabs(axis.x) > 0.001f) ? length : thickness,
        (fabs(axis.y) > 0.001f) ? length : thickness,
        (fabs(axis.z) > 0.001f) ? length : thickness
    };

    DrawCube(center, size.x, size.y, size.z, color);
}


int main() {
    // // Define a 2x2 matrix of doubles
    // Eigen::Matrix2d mat;
    // mat(0,0) = 3;
    // mat(1,0) = 2.5;
    // mat(0,1) = -1;
    // mat(1,1) = mat(0,0) + mat(1,0);

    // std::cout << "Here is the matrix mat:\n" << mat << std::endl;

    // // Define a 2D vector
    // Eigen::Vector2d vec(1, 2);

    // std::cout << "Here is the vector vec:\n" << vec << std::endl;

    // // Matrix-vector multiplication
    // Eigen::Vector2d result = mat * vec;
    // std::cout << "Result of mat * vec:\n" << result << std::endl;

    // // Compute the inverse of mat (if invertible)
    // if (mat.determinant() != 0) {
    //     Eigen::Matrix2d mat_inv = mat.inverse();
    //     std::cout << "Inverse of mat:\n" << mat_inv << std::endl;
    // } else {
    //     std::cout << "Matrix is not invertible." << std::endl;
    // }

    std::string planeText = "";


    // Define a camera
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 200.0f, 200.0f, 200.0f }; // Camera position in world space
    camera.target   = (Vector3){ 0.0f, 0.0f, 0.0f };      // What the camera is looking at
    camera.up       = (Vector3){ 0.0f, 1.0f, 0.0f };      // Which way is "up"
    camera.fovy     = 300;                             // Field of view (vertical) in degrees
    camera.projection = CAMERA_ORTHOGRAPHIC;


    InitWindow(800, 600, "Robot Dynamics Simulator - Test Window");

    Robot robot = Robot();

    while (!WindowShouldClose())
    {
        UpdateFreeCamera(&camera, 5.0f, 0.01f);

        BeginDrawing();
        BeginMode3D(camera); // Start 3D drawing
        ClearBackground(RAYWHITE);
        DrawGrid(20, 10.0f);

        DrawThickAxis((Vector3){0,0,0}, (Vector3){100,0,0}, RED, 2.0f);   // X axis
        DrawThickAxis((Vector3){0,0,0}, (Vector3){0,100,0}, GREEN, 2.0f); // Y axis
        DrawThickAxis((Vector3){0,0,0}, (Vector3){0,0,100}, BLUE, 2.0f);  // Z axis

        // Sinusiodal joint motion
        float totalTime = GetTime();
        robot.links[0].q_i = utils::Deg2Rad(20*sin(2*totalTime)+80);
        robot.links[1].q_i = utils::Deg2Rad(50*sin(3*totalTime)-10);


        robot.ForwardKinematics();
        robot.Render();
        EndMode3D(); // End 3D drawing

        // Draw UI text
        DrawText("Robot Simulator", 300, 20, 20, LIGHTGRAY);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
