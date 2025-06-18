#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <cmath>

namespace CameraUtils {
    void InitOrbitCameraQuat(Camera3D &camera);
    void UpdateOrbitCameraQuat(Camera3D* cam, float rotateSpeed, float zoomSpeed);
    void UpdateFreeCameraQuat(Camera3D *cam, float zoomSpeed, float rotateSpeed);
    void DrawThickAxis(Vector3 start, Vector3 end, Color color, float thickness);
    void DrawCylinderBetween(Vector3 pos, Vector3 endPos, float radius, Color color, int slices);
    void DrawXYGrid(int slices, float spacing);
    void PrintOrbitOrientation();
}