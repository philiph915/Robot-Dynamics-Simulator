#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <cmath>

namespace CameraUtils {
    void InitOrbitCameraQuat();
    Quaternion GetCameraOrientation(const Vector3 &camPos, const Vector3 &target, const Vector3 &worldUp);

    void UpdateOrbitCameraQuat(Camera3D* cam, float rotateSpeed, float zoomSpeed);
    void UpdateFreeCamera(Camera3D* cam, float moveSpeed, float rotateSpeed);
    void UpdateOrbitCamera(Camera3D *cam, float rotateSpeed, float zoomSpeed);
    void DrawThickAxis(Vector3 start, Vector3 end, Color color, float thickness);
    void DrawCylinderBetween(Vector3 pos, Vector3 endPos, float radius, Color color, int slices);
    void DrawXYGrid(int slices, float spacing);
    void PrintOrbitOrientation();
}