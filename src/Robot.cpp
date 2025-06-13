#include "Robot.h"
#include <iostream>

Robot::Robot()
{
    InitLinks();
}

void Robot::InitLinks()
{
    Link link1;
    link1.r_cg << 25, 0, 0;
    link1.r_i_1 <<50, 0, 0;
    link1.r_1_cg = link1.r_i_1 - link1.r_cg;
    link1.q_i = utils::Deg2Rad(30);
    AddLink(link1);

    Link link2;
    link2.r_cg << 25, 0, 0;
    link2.r_i_1 << 50, 0, 0;
    link2.r_1_cg = link2.r_i_1 - link2.r_cg;
    link2.q_i = utils::Deg2Rad(-50);
    AddLink(link2);

}

// Calculate all rotation matrices and positions
void Robot::ForwardKinematics()
{
    // Calculate link 1
    links[0].R_0_i = utils::rotZ(links[0].q_i);
    links[0].R_i   = utils::rotZ(links[0].q_i);
    links[0].position = Eigen::Vector3f::Zero();

    // shift the link 1 origin off the world origin
    links[0].position << 40, 0, 40;

    // std::cout<< "Link " << 0 << std::endl;
    // std::cout<<links[0].position<<std::endl;
    // std::cout<<links[0].R_0_i<<std::endl;
    // std::cout<< "q " << links[0].q_i*57.3 << std::endl;

    // Loop across links in the chain
    for (int ii = 1; ii < links.size(); ii++) {
        links[ii].R_i = utils::rotZ(links[ii].q_i); // rotation from link i-1 to i

        // Get the parent link
        Link& parent = links[ii-1];
        // parent.R_1_i = links[ii].R_i.transpose(); // rotation from i+1 to i (update for the parent link)

        links[ii].R_0_i = parent.R_0_i * links[ii].R_i; // absolute orientation (relative to base) of link i
        links[ii].position = parent.position + (parent.R_i * parent.r_i_1); // absolute position of link i

        // std::cout<< "Link " << ii << std::endl;
        // std::cout<<links[ii].R_0_i<<std::endl;
        // std::cout<<links[ii].position<<std::endl;
        // std::cout<< "q " << links[ii].q_i*57.3 << std::endl;
    }
}

void Robot::Render()
{
     // Set some constants for drawing
    float sphereRadius = 5.0f;
    Color linkColor = RED;
    Color jointColor = BLUE;

    for (size_t i = 0; i < links.size(); ++i)
    {
        Link& current = links[i];

        // Convert Eigen::Vector3f to raylib Vector3
        Vector3 pos = {
            current.position(0),
            current.position(1),
            current.position(2)
        };

        // Draw a sphere at the origin of the link
        DrawSphere(pos, sphereRadius, jointColor);

        // If not the last link, draw a line to the next link
        if (i < links.size() - 1)
        {
            Link& next = links[i+1];
            Vector3 nextPos = { next.position(0), next.position(1), next.position(2) };

            DrawLine3D(pos, nextPos, linkColor);
        }
        // Draw the last link 
        else 
        {
            Eigen::Vector3f nextPos_eigen = current.position + current.R_0_i*current.r_i_1;
            Vector3 nextPos = {nextPos_eigen(0), nextPos_eigen(1), nextPos_eigen(2)};
            DrawLine3D(pos, nextPos, linkColor);
            // Draw a sphere at the end of the last link
            DrawSphere(nextPos, sphereRadius, GREEN);
        }
    }
}

void Robot::AddLink(const Link& link)
{
    links.push_back(link);
}

