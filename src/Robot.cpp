#include "Robot.h"
#include <iostream>

Robot::Robot()
{
    InitLinks();
}

void Robot::InitLinks()
{
    Link defaultLink;
    Link newLink;
    
    newLink = defaultLink;
    newLink.linkNumber = 1;
    newLink.position << 40, 40, 0; //set first link origin WRT base frame
    newLink.r_cg << 0, 0, 0;
    newLink.r_i_1 << 40, 0, 0;
    newLink.r_1_cg = newLink.r_i_1 - newLink.r_cg;
    newLink.q_i = utils::Deg2Rad(45);
    newLink.alpha = utils::Deg2Rad(-90);
    AddLink(newLink);

    newLink = defaultLink;
    newLink.linkNumber = 2;
    newLink.r_cg << 25, 0, 0;
    newLink.r_i_1 << 50, 0, 0;
    newLink.r_1_cg = newLink.r_i_1 - newLink.r_cg;
    newLink.q_i = utils::Deg2Rad(-45);
    AddLink(newLink);

    newLink = defaultLink;
    newLink.linkNumber = 3;
    newLink.r_cg << 25, 0, 0;
    newLink.r_i_1 << 50, 0, 0;
    newLink.r_1_cg = newLink.r_i_1 - newLink.r_cg;
    newLink.q_i = utils::Deg2Rad(0);
    AddLink(newLink);

}

// Calculate all rotation matrices and positions
void Robot::ForwardKinematics()
{
    // Loop across links in the chain
    for (int ii = 0; ii < links.size(); ii++) {
        
        // get alpha from the parent link
        float alpha;
        if (ii == 0) {
            alpha = 0; //no alpha from base frame
        } else {
            alpha = links[ii-1].alpha;
        }

        // Rotation from i-1 to i: Rz(q_i) * Rx(alpha) (follows MDH conventions)
        Eigen::Matrix3f Rz = utils::rotZ(links[ii].q_i);
        Eigen::Matrix3f Rx = utils::rotX(alpha);
        links[ii].R_i = Rx * Rz;

        PrintJointState(links[ii]);

        // Special case for first link: previous frame is base frame
        if (ii==0) { 
            links[ii].R_0_i = links[ii].R_i;
            continue; 
        }

        // Get the parent link in order to calculate absolute position and orientation
        Link& parent = links[ii-1];

        links[ii].R_0_i = parent.R_0_i * links[ii].R_i; // absolute orientation (relative to base) of link i
        links[ii].position = parent.position + (parent.R_0_i * parent.r_i_1); // absolute position of link i
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
        // Render each link
        links[i].Render();

        // Render a sphere at the end effector
    }
}

void Robot::AddLink(const Link& link)
{
    links.push_back(link);
}

void Robot::PrintJointState(const Link& link)
{
    std::cout << "Link " << link.linkNumber << ":" << std::endl << std::endl;
    std::cout << "position: " << std::endl << link.position << std::endl << std::endl;
    std::cout << "q" << link.linkNumber << " = " << utils::Rad2Deg(link.q_i) << " degrees" << std::endl<< std::endl;
    std::cout << "R" << link.linkNumber <<": "<<std::endl << link.R_i << std::endl << std::endl;
    std::cout << "R0_" << link.linkNumber <<": "<<std::endl << link.R_0_i << std::endl << std::endl;
    std::cout << "Z axis for Link " << link.linkNumber << ": " << link.R_0_i.col(2).transpose() << "\n\n";
    std::cout << "alpha" << link.linkNumber << " = " << utils::Rad2Deg(link.alpha) << " degrees" << std::endl << std::endl;



}
// scraps: print states

// std::cout<< "Link " << 0 << std::endl;
// std::cout<<links[0].position<<std::endl;
// std::cout<<links[0].R_0_i<<std::endl;
// std::cout<< "q " << links[0].q_i*57.3 << std::endl;

// std::cout<< "Link " << ii << std::endl;
// std::cout<<links[ii].R_0_i<<std::endl;
// std::cout<<links[ii].position<<std::endl;
// std::cout<< "q " << links[ii].q_i*57.3 << std::endl;