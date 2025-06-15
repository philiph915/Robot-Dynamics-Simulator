#include "CameraUtils.h"
#include <iostream>

// Persistent orbit‐camera state 
static Quaternion orbitOrientation = { 0 };
static Vector3    orbitTarget      = { 0, 0, 0 };
static float      orbitDistance    = 200.0f;

void CameraUtils::InitOrbitCameraQuat(Vector3 target, float distance)
{
    orbitTarget   = target;
    orbitDistance = distance;
    orbitOrientation = {0.0925058, -0.226331, 0.366871, 0.897615};
    //QuaternionIdentity();   // “look along +X” initially
}

void CameraUtils::UpdateOrbitCameraQuat(Camera3D* cam, float rotateSpeed, float zoomSpeed)
{
    // — YAW around world Z —
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))
    {
        Quaternion q = QuaternionFromAxisAngle((Vector3){0,0,1}, +rotateSpeed);
        orbitOrientation = QuaternionMultiply(q, orbitOrientation);
    }
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT))
    {
        Quaternion q = QuaternionFromAxisAngle((Vector3){0,0,1}, -rotateSpeed);
        orbitOrientation = QuaternionMultiply(q, orbitOrientation);
    }

    // — COMPUTE LOCAL AXES —
    Vector3 forward  = Vector3Normalize(Vector3RotateByQuaternion((Vector3){1,0,0}, orbitOrientation));
    Vector3 cameraUp = Vector3Normalize(Vector3RotateByQuaternion((Vector3){0,0,1}, orbitOrientation));
    Vector3 right    = Vector3Normalize(Vector3CrossProduct(cameraUp, forward));

    // — PITCH around camera‐local right —
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))
    {
        Quaternion q = QuaternionFromAxisAngle(right, +rotateSpeed);
        orbitOrientation = QuaternionMultiply(q, orbitOrientation);
    }
    if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))
    {
        Quaternion q = QuaternionFromAxisAngle(right, -rotateSpeed);
        orbitOrientation = QuaternionMultiply(q, orbitOrientation);
    }

    // — ZOOM by FOV only —
    float dt     = GetFrameTime();
    float scroll = GetMouseWheelMove();

    if (IsKeyDown(KEY_Q)) cam->fovy -= zoomSpeed * dt;
    if (IsKeyDown(KEY_E)) cam->fovy += zoomSpeed * dt;
    if (scroll != 0.0f)   cam->fovy -= scroll * zoomSpeed * 0.05f;

    // Clamp the FOV to avoid extreme distortion
    cam->fovy = Clamp(cam->fovy, 10.0f, 600.0f);

    // — UPDATE CAMERA POSITION + UP —
    Vector3 baseOffset = { orbitDistance, 0, 0 };
    Vector3 offset     = Vector3RotateByQuaternion(baseOffset, orbitOrientation);

    cam->position = Vector3Add(orbitTarget, offset);
    cam->target   = orbitTarget;
    cam->up       = cameraUp;

    // Debugging: Print the quaternion orientation
    PrintOrbitOrientation();
}

void CameraUtils::UpdateFreeCamera(Camera3D *cam, float zoomSpeed, float rotateSpeed)
{
    if (IsKeyPressed(KEY_ONE)) { // XY plane
        cam->position = (Vector3){ 0, 0, 500 };
        cam->up = (Vector3){ 0, 1, 0 };
        cam->fovy     = 300; 
    }
    if (IsKeyPressed(KEY_TWO)) { // XZ plane
        cam->position = (Vector3){ 0, 500, 0 };
        cam->up = (Vector3){ 0, 0, 1 };
        cam->fovy     = 300; 
    }
    if (IsKeyPressed(KEY_THREE)) { // YZ plane
        cam->position = (Vector3){ 500, 0, 0 };
        cam->up = (Vector3){ 0, 0, 1 };
        cam->fovy     = 300; 
    }
    if (IsKeyPressed(KEY_ZERO) || IsKeyPressed(KEY_SPACE)) { // Return to starting position
        cam->position = (Vector3){ 200.0f, 200.0f, 200.0f }; // Camera position in world space
        cam->target   = (Vector3){ 0.0f, 0.0f, 0.0f };      // What the camera is looking at
        cam->up       = (Vector3){ 0.0f, 0.0f, 1.0f };      // Which way is "up"
        cam->fovy     = 300; 
    }


    CameraUtils::UpdateOrbitCamera(cam, rotateSpeed/20, zoomSpeed*100);

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

void CameraUtils::UpdateOrbitCamera(Camera3D* cam, float rotateSpeed = 0.02f, float zoomSpeed = 2.0f)
{
    Vector3 offset = Vector3Subtract(cam->position, cam->target);

    // Handle rotation around Z axis (Yaw)
    if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) {
        Matrix rotZ = MatrixRotateZ(-rotateSpeed);
        offset = Vector3Transform(offset, rotZ);
    }
    if (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A)) {
        Matrix rotZ = MatrixRotateZ(rotateSpeed);
        offset = Vector3Transform(offset, rotZ);
    }

    // Handle pitch rotation around X axis (in camera-local plane)
    Vector3 right = Vector3Normalize(Vector3CrossProduct((Vector3){0, 0, 1}, offset));
    if (IsKeyDown(KEY_UP) || IsKeyDown(KEY_W)) {
        Matrix rotPitch = MatrixRotate(right, rotateSpeed);
        offset = Vector3Transform(offset, rotPitch);
    }
    if (IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S)) {
        Matrix rotPitch = MatrixRotate(right, -rotateSpeed);
        offset = Vector3Transform(offset, rotPitch);
    }

     if (IsKeyDown(KEY_Q)) {
        cam->fovy -= zoomSpeed * GetFrameTime();
        cam->fovy = std::max(10.0f, cam->fovy);
    }
    if (IsKeyDown(KEY_E)) {
        cam->fovy += zoomSpeed * GetFrameTime();
        cam->fovy = std::min(600.0f, cam->fovy);
    }

    float scroll = GetMouseWheelMove();
    if (scroll != 0.0f) {
        cam->fovy -= scroll * zoomSpeed * 0.05f;
        cam->fovy = std::clamp(cam->fovy, 10.0f, 600.0f);
    }

    cam->position = Vector3Add(cam->target, offset);
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

// Helper function to draw a cylinder from pos to endPos
void CameraUtils::DrawCylinderBetween(Vector3 pos, Vector3 endPos, float radius, Color color, int slices)
{
    // Compute direction vector and length
    Vector3 dir = Vector3Subtract(endPos, pos);
    float length = Vector3Length(dir);

    if (length < 0.0001f) return;  // Avoid drawing zero-length cylinder

    // Midpoint between start and end
    Vector3 mid = Vector3Lerp(pos, endPos, 0.5f);

    // Default cylinder is aligned along +Y
    Vector3 up = { 0.0f, 1.0f, 0.0f };
    Vector3 axis = Vector3CrossProduct(up, dir);
    float angle = RAD2DEG * acosf(Vector3DotProduct(Vector3Normalize(up), Vector3Normalize(dir)));

    rlPushMatrix();
        rlTranslatef(mid.x, mid.y, mid.z);
        if (Vector3Length(axis) > 0.0001f) rlRotatef(angle, axis.x, axis.y, axis.z);
        DrawCylinderEx((Vector3){ 0, -length/2, 0 }, (Vector3){ 0, length/2, 0 }, radius, radius, slices, color);
    rlPopMatrix();
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

void CameraUtils::PrintOrbitOrientation()
{
    std::cout
    << "Orbit Orientation — "
    << "x: " << orbitOrientation.x << ", "
    << "y: " << orbitOrientation.y << ", "
    << "z: " << orbitOrientation.z << ", "
    << "w: " << orbitOrientation.w
    << std::endl;
}
