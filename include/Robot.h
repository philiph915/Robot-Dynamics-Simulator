#pragma once
#include <Eigen/Dense>
#include "MathUtils.h"
#include <Vector>
#include "Link.h"
#include "raylib.h"

class Robot {

public:

// Functions
Robot();

void InitLinks();
void ForwardKinematics();
void Render();
void AddLink(const Link& link);
void PrintJointState(const Link &link);

std::vector<Link> ComputeForwardKinematics(const std::vector<float>& q);
std::vector<float> GetJointAngles() const;

// Properties
std::vector<Link> links;

private:

};