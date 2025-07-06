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
void UpdateKinematics();
void Render();
void AddLink(const Link& link);
void PrintJointState(const Link &link);

std::vector<Link> ComputeForwardKinematics(const std::vector<float>& q);
std::vector<Link> ComputeLinkVelocities(const std::vector<float> &q, const std::vector<float> &dq );
std::vector<Link> ComputeLinkAccelerations(const std::vector<float> &q, const std::vector<float> &dq, const std::vector<float> &ddq  );
std::vector<float> GetJointAngles() const;
std::vector<float> GetJointVelocities() const;
std::vector<float> GetJointAccelerations() const;

// Properties
std::vector<Link> links;

private:

};