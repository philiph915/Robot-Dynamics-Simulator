#include "CameraUtils.h"

void CameraUtils::UpdateFreeCamera(Camera3D *cam, float moveSpeed, float rotateSpeed)
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
    if (IsKeyPressed(KEY_ZERO)) { // Return to starting position
        cam->position = (Vector3){ 200.0f, 200.0f, 200.0f }; // Camera position in world space
        cam->target   = (Vector3){ 0.0f, 0.0f, 0.0f };      // What the camera is looking at
        cam->up       = (Vector3){ 0.0f, 0.0f, 1.0f };      // Which way is "up"
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
        float anglePitch = delta.y * rotateSpeed;

        // Simple rotation around target
        Vector3 offset = Vector3Subtract(cam->position, cam->target);

        Matrix matYaw = MatrixRotateZ(angleYaw);
        Matrix matPitch = MatrixRotateX(anglePitch);

        // Apply yaw first, then pitch
        offset = Vector3Transform(offset, matYaw);
        offset = Vector3Transform(offset, matPitch);

        cam->position = Vector3Add(cam->target, offset);
    }
}

void CameraUtils::DrawThickAxis(Vector3 start, Vector3 end, Color color, float thickness)
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

void CameraUtils::DrawXYGrid(int slices, float spacing)
{
    for (int i = -slices; i <= slices; i++)
    {
        Color color = (i == 0) ? DARKGRAY : (i % 5 == 0 ? GRAY : LIGHTGRAY);

        // Horizontal lines (along X)
        DrawLine3D(
            { -slices * spacing, i * spacing, 0.0f },
            {  slices * spacing, i * spacing, 0.0f },
            color
        );

        // Vertical lines (along Y)
        DrawLine3D(
            { i * spacing, -slices * spacing, 0.0f },
            { i * spacing,  slices * spacing, 0.0f },
            color
        );
    }
}


