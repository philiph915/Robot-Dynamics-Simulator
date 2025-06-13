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
void ForwardKinematics();
void Render();
void AddLink(const Link& link);

private:

};