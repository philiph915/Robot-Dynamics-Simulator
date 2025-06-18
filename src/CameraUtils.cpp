#include "CameraUtils.h"
#include <iostream>

// Persistent orbit‐camera state 
static Quaternion orbitOrientation = { 0 };
static Vector3    orbitTarget      = { 0, 0, 0 };
static float      orbitDistance    = 500.0f;

void CameraUtils::InitOrbitCameraQuat(Camera3D &camera)
{
    // Use hard-coded values for initial orientation and field of view
    orbitOrientation = {0.0925058, -0.226331, 0.366871, 0.897615};
    camera.fovy     = 300; 

    // Calculate an analytical quaternion to get to the desired camera angle (THIS DOESN'T WORK)
    // Vector3 camPos  = { 200.0f, -200.0f, 200.0f };
    // Vector3 target  = { 0.0f, 0.0f, 0.0f };
    // Vector3 up      = { 0.0f, 0.0f, 1.0f };
    // // 1) Build the view matrix (world → camera)
    // Matrix view = MatrixLookAt(camPos, target, up);
    // // 2) Invert it to get the camera’s world-space transform
    // Matrix transform = MatrixInvert(view);
    // // 3) Extract the rotation quaternion
    // orbitOrientation = QuaternionFromMatrix(transform);

}

void CameraUtils::UpdateOrbitCameraQuat(Camera3D* cam, float rotateSpeed, float zoomSpeed)
{
    float rotateSpeed_standard = rotateSpeed;
    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
    {
        rotateSpeed = rotateSpeed_standard*2;
    }
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

    // COMPUTE LOCAL AXES 
    Vector3 forward  = Vector3Normalize(Vector3RotateByQuaternion((Vector3){1,0,0}, orbitOrientation));
    Vector3 cameraUp = Vector3Normalize(Vector3RotateByQuaternion((Vector3){0,0,1}, orbitOrientation));
    Vector3 right    = Vector3Normalize(Vector3CrossProduct(cameraUp, forward));

    // PITCH around camera‐local right 
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

    // ZOOM by FOV only
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

    // Debug Print
    if (0) {
        float time = GetTime();
        if (std::fmod(time, 1.0f) < dt) {
            // Debugging: Print the quaternion orientation
            PrintOrbitOrientation();

            // UpdateFreeCamera(cam, zoomSpeed, rotateSpeed);
            std::cout << "cam pos: " << cam->position.x <<"," << cam->position.y <<"," << cam->position.z << std::endl;
        }
    }
}

void CameraUtils::UpdateFreeCameraQuat(Camera3D* cam, float zoomSpeed, float rotateSpeed)
{
    // Snap views — adjust orbitOrientation and orbitDistance
    if (IsKeyPressed(KEY_ONE)) { // XY plane (Z up)
        Quaternion yaw = QuaternionFromAxisAngle((Vector3){0,0,1}, -PI/2);  // rotate -90° about Z  (Gets to XZ Plane)
        Quaternion pitch = QuaternionFromAxisAngle((Vector3){1,0,0},-PI/2); // rotate 90° about X  (Gets to XY Plane)
        Quaternion q = QuaternionMultiply(pitch,yaw);                       // Combine rotations
        orbitOrientation = q;
        std::cout << "Snapped to XY plane" << std::endl;
    }
    if (IsKeyPressed(KEY_TWO)) { // YZ plane (Z up)
        orbitOrientation = QuaternionFromAxisAngle((Vector3){0,0,1}, 0); // look down +X
        std::cout << "Snapped to YZ plane" << std::endl;
    }
    if (IsKeyPressed(KEY_THREE)) { // XZ plane (Z up)
        Quaternion q = QuaternionFromAxisAngle((Vector3){0,0,1}, -PI/2); // rotate -90° about Z  (Gets to XZ Plane)
        orbitOrientation = q;
        std::cout << "Snapped to XZ plane" << std::endl;
    }
    if (IsKeyPressed(KEY_ZERO) || IsKeyPressed(KEY_SPACE)) { // Reset to diag view
        InitOrbitCameraQuat(*cam);
        std::cout << "Reset view" << std::endl;
    }

    // Mouse drag to rotate
    if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
    {
        Vector2 delta = GetMouseDelta();
        float yaw   = -delta.x * rotateSpeed*5;
        float pitch = -delta.y * rotateSpeed*5;

        Quaternion qYaw   = QuaternionFromAxisAngle((Vector3){0,0,1}, yaw); // world Z
        Vector3 camRight  = Vector3Normalize(Vector3RotateByQuaternion((Vector3){0,1,0}, orbitOrientation)); // local right
        Quaternion qPitch = QuaternionFromAxisAngle(camRight, pitch);

        orbitOrientation = QuaternionMultiply(qPitch, QuaternionMultiply(qYaw, orbitOrientation));
    }

    // Call the orbit camera update to apply changes
    UpdateOrbitCameraQuat(cam, rotateSpeed, zoomSpeed);
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
