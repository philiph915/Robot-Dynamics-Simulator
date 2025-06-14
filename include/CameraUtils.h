#pragma once
#include "raylib.h"
#include "raymath.h"

namespace CameraUtils {
    void UpdateFreeCamera(Camera3D* cam, float moveSpeed, float rotateSpeed);
    void DrawThickAxis(Vector3 start, Vector3 end, Color color, float thickness);
    void DrawXYGrid(int slices, float spacing);
}