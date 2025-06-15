#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>

namespace CameraUtils {
    void UpdateFreeCamera(Camera3D* cam, float moveSpeed, float rotateSpeed);
    void UpdateOrbitCamera(Camera3D *cam, float rotateSpeed, float zoomSpeed);
    void DrawThickAxis(Vector3 start, Vector3 end, Color color, float thickness);
    void DrawCylinderBetween(Vector3 pos, Vector3 endPos, float radius, Color color, int slices);
    void DrawXYGrid(int slices, float spacing);
}