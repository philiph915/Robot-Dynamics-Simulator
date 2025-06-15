#pragma once
#include <Eigen/Dense>
#include "MathUtils.h"
#include <Vector>
#include "Link.h"
#include "raylib.h"

class Robot {

public:

std::vector<Link> links;
Robot();
void InitLinks();
std::vector<float> GetJointAngles() const;
void ForwardKinematics();
std::vector<Link> ComputeForwardKinematics(const std::vector<float>& q);
void Render();
void AddLink(const Link& link);
void PrintJointState(const Link &link);

private:

};