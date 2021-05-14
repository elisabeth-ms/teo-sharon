// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckSelfCollisionLibrary.hpp"


namespace roboticslab
{

    bool CheckSelfCollision::jointsInsideBounds(const KDL::JntArray& q){

        if(q.rows()!= qmin.rows() || q.rows()!=qmax.rows()){
            throw std::runtime_error("q size is not equal to qmin and qmax size");
        }
        printf("joints inside bounds");
        for(int i=0; i<qmax.rows(); i++){
            printf("qmax(%d): %f\n", i,qmax(i));
        }
        for(int i = 0; i<q.rows(); i++){
            printf("%f %f %f \n", q(i), qmin(i), qmax(i));
            if(q(i)< qmin(i) || q(i)>qmax(i)){
                return false;
            }
        }
        return true;
    }

    bool CheckSelfCollision::updateCollisionObjectsTransform(const KDL::JntArray &q){
        int kinematics_status;
        KDL::ChainFkSolverPos_recursive fksolver(chain);

        if (jointsInsideBounds(q)){
            printf("jointsInsideBounds\n");
            for(int i=0; i<centerLinksWrtJoints.size(); i++){
                KDL::Frame frameJoint; 
                kinematics_status = fksolver.JntToCart(q, frameJoint, centerLinksWrtJoints[i].first);
                printf("%d %f %f %f\n",centerLinksWrtJoints[i].first+1,frameJoint.p.x(), frameJoint.p.y(), frameJoint.p.z());

                KDL::Frame frameCenterLink = frameJoint * centerLinksWrtJoints[i].second;
                fcl::Vec3f translation(frameCenterLink.p[0], frameCenterLink.p[1], frameCenterLink.p[2]);
                double x, y, z, w;
                frameCenterLink.M.GetQuaternion(x, y, z, w);
                fcl::Quaternion3f rotation(w, x, y, z);
                collisionObjects[i].setTransform(rotation, translation);
            }

            return true;

        }
        return false;
    }
    bool CheckSelfCollision::twoLinksCollide(const KDL::JntArray &q, int link1, int link2){
        fcl::CollisionRequest requestType(1, false, 1, false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&collisionObjects[link1], &collisionObjects[link2], requestType, collisionResult);
        if (collisionResult.isCollision())
        {
            return true;
        }
        else{
            return false;
        }
    
    }



/************************************************************************/

/************************************************************************/

/************************************************************************/

} // namespace roboticslab
