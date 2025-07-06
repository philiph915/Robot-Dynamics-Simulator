#include "Robot.h"
#include <iostream>

Robot::Robot()
{
    InitLinks();
}

// Configure robot properties (Modified DH Convention)
void Robot::InitLinks()
{
    Link defaultLink;
    Link newLink;
    
    newLink = defaultLink;
    newLink.linkNumber = 1;
    newLink.position << 40, 40, 0; //set first link origin WRT base frame
    newLink.r_cg << 0, 0, 0;
    newLink.r_i_1 << 20, 0, 0;
    newLink.r_1_cg = newLink.r_i_1 - newLink.r_cg;
    newLink.q_i = utils::Deg2Rad(45);
    newLink.alpha = utils::Deg2Rad(90);
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

// Helper function to get the vector of joint angles
std::vector<float> Robot::GetJointAngles() const {  // add const here to indicate this is a read-only operation
    std::vector<float> q;
    for (const Link& link : links) {
        q.push_back(link.q_i);
    }
    return q;
}

std::vector<float> Robot::GetJointVelocities() const
{
    std::vector<float> dq;
    for (const Link& link : links) {
        dq.push_back(link.dq_i);
    }
    return dq;
}

// Update robot positions and rotation matrices via forward kinematics 
void Robot::ForwardKinematics()
{
    // Update link positions and rotation matrices using current joint angles
    links = ComputeForwardKinematics(GetJointAngles());

    // Debugging: Print joint states
    if (0) {
        for (const Link& link : links) {
            PrintJointState(link);
        }
    }
}

// Calculate all rotation matrices and positions
std::vector<Link> Robot::ComputeForwardKinematics(const std::vector<float> &q)
{
    std::vector<Link> newLinks = links; // Start from current link structure (copy joint metadata, alpha, r_i_1, etc.)

     // Loop across links in the chain
    for (int ii = 0; ii < newLinks.size(); ii++) {
        
        // get alpha from the parent link
        float alpha;
        if (ii == 0) {
            alpha = 0; //no alpha from base frame
        } else {
            alpha = newLinks[ii-1].alpha;
        }

        // Rotation from i-1 to i: Rz(q_i) * Rx(alpha) (follows MDH conventions)
        Eigen::Matrix3f Rz = utils::rotZ(newLinks[ii].q_i);
        Eigen::Matrix3f Rx = utils::rotX(alpha);
        newLinks[ii].R_i = Rx * Rz;

        // Get the parent link in order to calculate absolute position and orientation
        if (ii==0) { 
            newLinks[ii].R_0_i = newLinks[ii].R_i; // Special case for first link: previous frame is the base frame, so R_0_i is R_i
        } else {
            Link& parent = newLinks[ii-1];
            newLinks[ii].R_0_i = parent.R_0_i * newLinks[ii].R_i; // absolute orientation (relative to base) of link i
            newLinks[ii].position = parent.position + (parent.R_0_i * parent.r_i_1); // absolute position of link i
        }
    }
    return newLinks;
}

// Calculate link linear/angular velocities + update spatial velocity vectors
std::vector<Link> Robot::ComputeLinkVelocities(const std::vector<float> &q, const std::vector<float> &dq )
{
    std::vector<Link> newLinks = links; // Start from current link structure 

        // Loop across links in the chain; Calculate linear velocity of each link's CoM and origin
        for (int ii = 0; ii < newLinks.size(); ii++) {

            Link& link = newLinks[ii];
            
            // Get link angular velocity in the local frame
            Eigen::Vector3f omega_local;
            omega_local << 0,0,link.dq_i;

            // Transform link angular velocity to the world frame
            link.omega_i = link.R_0_i * omega_local;

            if (ii==0) {
                
                // base link: origin does not have linear velocity
                link.v_origin = Eigen::Vector3f::Zero();
            
            } else {
                
                // otherwise: get data from parent link
                const Link& parent = newLinks[ii-1];
                Eigen::Vector3f r_i_1 = parent.R_0_i * parent.r_i_1; // vector from the parent origin to the child origin, expressed in world coordinates

                // omega cross r
                Eigen::Vector3f cross_term = parent.omega_i.cross(r_i_1);

                // link origin velocity
                link.v_origin = parent.v_origin + cross_term;
            }

            // compute linear velocity of CoM
            link.v_ci = link.v_origin + link.omega_i.cross(link.r_cg);

            // Populate 6D spatial velocity
            link.v_i.head<3>() = link.omega_i;
            link.v_i.tail<3>() = link.v_origin;
        }

    return newLinks;
}

void Robot::Render()
{
     // Set some constants for drawing
    float sphereRadius = 5.0f;
    Color EEcolor    = RED;     // color of the end-effector

    for (size_t i = 0; i < links.size(); ++i)
    {
        // Render each link
        links[i].Render();
    }

    // Render a sphere at the end effector
    Link& endLink = links[links.size()-1];
    Vector3 EEpos = utils::ConvertEigen2Raylib_Vector3(endLink.GetEndOfLink());
    DrawSphere(EEpos,sphereRadius,EEcolor);
}

void Robot::AddLink(const Link& link)
{
    links.push_back(link);
}

// Function to print state variables of the input link
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