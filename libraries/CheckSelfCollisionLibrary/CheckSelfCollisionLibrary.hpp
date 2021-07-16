// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHECK_SELF_COLLISION_HPP__
#define __CHECK_SELF_COLLISION_HPP__

#include <array>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <cmath>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/distance.h>
namespace sharon
{

/**
 * @ingroup checkSelfCollision
 *
 * @brief checkSelfCollision
 */
class CheckSelfCollision
{
public:
    CheckSelfCollision(const KDL::Chain & t_chain, const KDL::JntArray & t_qmin, const KDL::JntArray & t_qmax, std::vector<fcl::CollisionObjectf> &t_collisionObjects, std::vector<std::array <float,3>> &t_offset): 
    chain(t_chain), qmin(t_qmin), qmax(t_qmax), collisionObjects(t_collisionObjects), offsetCenterCollisionObjects(t_offset)
    {
        printf("eo\n");
        if (chain.getNrOfJoints() <1)
            throw std::runtime_error("Empty chain");
        printf("No empty chain!\n");
        if(qmin.rows() == 0)
            throw std::runtime_error("Empty qmin");
        printf("No empty qmin\n");

        if(qmax.rows() == 0){
            throw std::runtime_error("Empty qmax");
        }
        printf("No empty qmax\n");

        if(chain.getNrOfJoints() != qmin.rows() || chain.getNrOfJoints() != qmax.rows()){
            throw std::runtime_error("Number of joints is not equal to qmin and qmax size");
        }

        KDL::Frame f =  chain.getSegment(0).getFrameToTip();
        for(int i=0; i< chain.getNrOfSegments(); i++){
                
            double x = chain.getSegment(i).getFrameToTip().p.x();
            double y = chain.getSegment(i).getFrameToTip().p.y();
            double z = chain.getSegment(i).getFrameToTip().p.z();
            double roll, pitch, yaw;
            chain.getSegment(i).getFrameToTip().M.GetRPY(roll, pitch, yaw);
            double norm = chain.getSegment(i).getFrameToTip().p.Norm();

            if (norm != 0){
                KDL::Frame frameLink;
                frameLink.p = KDL::Vector(x/2.0, y/2.0, z/2.0);
                
                centerLinksWrtJoints.push_back(std::make_pair(i, frameLink));
                printf("i: %d, x: %f, y: %f, z: %f\n", i, x/2, y/2, z/2);
                printf("norm: %f\n", norm);

            }
        }

        if(centerLinksWrtJoints.size()!= collisionObjects.size()){
            printf("%d %d", centerLinksWrtJoints.size(), collisionObjects.size());
            throw std::runtime_error("Error in the number of collision objects");
        }

        if(offsetCenterCollisionObjects.size() != collisionObjects.size()){
            throw std::runtime_error("Error in the number of offset center collision objects");
        }

        for(int i=0; i<centerLinksWrtJoints.size(); i++){
            centerLinksWrtJoints[i].second.p.data[0] += offsetCenterCollisionObjects[i][0];
            centerLinksWrtJoints[i].second.p.data[1] += offsetCenterCollisionObjects[i][1];
            centerLinksWrtJoints[i].second.p.data[2] += offsetCenterCollisionObjects[i][2];
        }

    }

    bool jointsInsideBounds(const KDL::JntArray &q);
    bool updateCollisionObjectsTransform(const KDL::JntArray &q);
    bool selfCollision();
    bool twoLinksCollide(const KDL::JntArray &q, int link1, int link2);
    double twoLinksDistance(const KDL::JntArray &q, int link1, int link2);
    double minDistance();

    KDL::JntArray jointsDeg2Rad(const KDL::JntArray &q);
    KDL::Chain chain;
    KDL::JntArray qmin;
    KDL::JntArray qmax;
    fcl::Transform3f tf1;
    using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryf>;
    // CollisionGeometryPtr_t tree_obj_;
    // fcl::CollisionObject(tree_obj_, tf1) test;

    std::vector<fcl::CollisionObjectf> collisionObjects;
    std::vector<std::array<float,3>> offsetCenterCollisionObjects;
    std::vector<std::pair<int,KDL::Frame>> centerLinksWrtJoints;


};

} // namespace sharon

#endif // __CHECK_SELF_COLLISION_HPP__
