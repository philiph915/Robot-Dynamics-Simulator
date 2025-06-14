// main.cpp
#include <iostream>
#include <Eigen/Dense>
#include "raylib.h"
#include "raymath.h"
#include "Robot.h"
#include "CameraUtils.h"


int main() {
    std::string planeText = "";

    // Define a camera
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 200.0f, 200.0f, 200.0f }; // Camera position in world space
    camera.target   = (Vector3){ 0.0f, 0.0f, 0.0f };      // What the camera is looking at
    camera.up       = (Vector3){ 0.0f, 0.0f, 1.0f };      // Which way is "up"
    camera.fovy     = 300;                             // Field of view (vertical) in degrees
    camera.projection = CAMERA_ORTHOGRAPHIC;


    InitWindow(800, 600, "Robot Dynamics Simulator - Test Window");

    Robot robot = Robot();

    while (!WindowShouldClose())
    {
        CameraUtils::UpdateFreeCamera(&camera, 5.0f, 0.01f);

        BeginDrawing();
        BeginMode3D(camera); // Start 3D drawing
        ClearBackground(RAYWHITE);
        CameraUtils::DrawXYGrid(20, 10.0f);

        CameraUtils::DrawThickAxis((Vector3){0,0,0}, (Vector3){100,0,0}, RED, 2.0f);   // X axis
        CameraUtils::DrawThickAxis((Vector3){0,0,0}, (Vector3){0,100,0}, GREEN, 2.0f); // Y axis
        CameraUtils::DrawThickAxis((Vector3){0,0,0}, (Vector3){0,0,100}, BLUE, 2.0f);  // Z axis

        // Sinusiodal joint motion
        float totalTime = GetTime();
        // robot.links[0].q_i = utils::Deg2Rad(45); //0.5*totalTime;
        // robot.links[1].q_i = utils::Deg2Rad(90); //utils::Deg2Rad(20*sin(2*totalTime)+90);
        // robot.links[2].q_i = utils::Deg2Rad(45); //utils::Deg2Rad(50*sin(3*totalTime));


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
