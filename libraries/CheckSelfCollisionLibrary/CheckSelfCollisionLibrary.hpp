// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHECK_SELF_COLLISION_HPP__
#define __CHECK_SELF_COLLISION_HPP__

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <array>

#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>



namespace roboticslab
{

/**
 * @ingroup checkSelfCollision
 *
 * @brief checkSelfCollision
 */
class CheckSelfCollision
{
public:
    CheckSelfCollision(const KDL::Chain & _chain, const KDL::JntArray & _qmin, const KDL::JntArray & _qmax, std::vector<fcl::CollisionObject> &_collisionObjects): 
    chain(_chain), qmin(_qmin), qmax(_qmax), collisionObjects(_collisionObjects)
    {
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
                printf("norm: %f\n", norm);
            }
        }

        if(centerLinksWrtJoints.size()!= collisionObjects.size()){
            throw std::runtime_error("Error in the number of collision objects");
        }

    }

    bool jointsInsideBounds(const KDL::JntArray &q);
    bool updateCollisionObjectsTransform(const KDL::JntArray &q);
    bool selfCollision(const KDL::JntArray &q);
    bool twoLinksCollide(const KDL::JntArray &q, int link1, int link2);
    KDL::Chain chain;
    KDL::JntArray qmin;
    KDL::JntArray qmax;
    std::vector<fcl::CollisionObject> collisionObjects;
    std::vector<std::pair<int,KDL::Frame>> centerLinksWrtJoints;
    typedef std::shared_ptr <fcl::CollisionGeometry> CollisionGeometryPtr_t;


};

} // namespace roboticslab

#endif // __CHECK_SELF_COLLISION_HPP__
