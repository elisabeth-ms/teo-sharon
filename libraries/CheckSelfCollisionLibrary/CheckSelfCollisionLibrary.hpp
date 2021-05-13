// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHECK_SELF_COLLISION_HPP__
#define __CHECK_SELF_COLLISION_HPP__

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>


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
    CheckSelfCollision(const KDL::Chain & _chain, const KDL::JntArray & _qmin, const KDL::JntArray & _qmax): chain(_chain), qmin(_qmin), qmax(_qmax)
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

    }

    bool jointsInsideBounds(const KDL::JntArray &q);

    KDL::Chain chain;
    KDL::JntArray qmin;
    KDL::JntArray qmax;
};

} // namespace roboticslab

#endif // __CHECK_SELF_COLLISION_HPP__
