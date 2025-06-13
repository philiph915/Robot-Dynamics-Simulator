// main.cpp
#include <iostream>
#include <Eigen/Dense>
#include "raylib.h"

int main() {
    // Define a 2x2 matrix of doubles
    Eigen::Matrix2d mat;
    mat(0,0) = 3;
    mat(1,0) = 2.5;
    mat(0,1) = -1;
    mat(1,1) = mat(0,0) + mat(1,0);

    std::cout << "Here is the matrix mat:\n" << mat << std::endl;

    // Define a 2D vector
    Eigen::Vector2d vec(1, 2);

    std::cout << "Here is the vector vec:\n" << vec << std::endl;

    // Matrix-vector multiplication
    Eigen::Vector2d result = mat * vec;
    std::cout << "Result of mat * vec:\n" << result << std::endl;

    // Compute the inverse of mat (if invertible)
    if (mat.determinant() != 0) {
        Eigen::Matrix2d mat_inv = mat.inverse();
        std::cout << "Inverse of mat:\n" << mat_inv << std::endl;
    } else {
        std::cout << "Matrix is not invertible." << std::endl;
    }

    InitWindow(800, 600, "Robot Dynamics Simulator - Test Window");

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("Hello, Raylib!", 190, 200, 20, LIGHTGRAY);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
