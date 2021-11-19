// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckSelfCollisionLibrary.hpp"

namespace sharon
{

    bool CheckSelfCollision::jointsInsideBounds(const KDL::JntArray &q)
    {

        if (q.rows() != qmin.rows() || q.rows() != qmax.rows())
        {
            throw std::runtime_error("q size is not equal to qmin and qmax size");
        }
        // printf("joints inside bounds");
        // for(int i=0; i<qmax.rows(); i++){
        //     printf("qmax(%d): %f\n", i,qmax(i));
        // }
        for (int i = 0; i < q.rows(); i++)
        {
            // printf("q: %f qmin: %f qmax: %f \n", q(i), qmin(i), qmax(i));
            if (q(i) < (qmin(i)+MARGIN_BOUNDS) || q(i) > (qmax(i)-MARGIN_BOUNDS))
            {
                // printf("Joints outside bounds\n");
                return false;
            }
        }
        // printf("Joints inside bounds\n");
        return true;
    }

    KDL::JntArray CheckSelfCollision::jointsDeg2Rad(const KDL::JntArray &q)
    {
        KDL::JntArray qRad(q.rows());
        for (int i = 0; i < q.rows(); i++)
        {
            qRad(i) = q(i) * KDL::deg2rad;
            //printf("q(%d) = %f", i, q(i));
        }
        return qRad;
    }

    bool CheckSelfCollision::updateCollisionObjectsTransform(const KDL::JntArray &q)
    {

        int kinematics_status;
        KDL::ChainFkSolverPos_recursive fksolver(chain);
        KDL::JntArray qRad = jointsDeg2Rad(q);
        //printf("\n");
        if (jointsInsideBounds(q))
        {
            // printf("jointsInsideBounds\n");
            for (int i = 0; i < centerLinksWrtJoints.size(); i++)
            {
                KDL::Frame frameJoint;
                kinematics_status = fksolver.JntToCart(qRad, frameJoint, centerLinksWrtJoints[i].first);
                //printf("kinematic status %d\n", kinematics_status);
                // printf("%d %f %f %f\n",centerLinksWrtJoints[i].first,frameJoint.p.x(), frameJoint.p.y(), frameJoint.p.z());

                KDL::Frame frameCenterLink = frameJoint * centerLinksWrtJoints[i].second;
                fcl::Vector3f translation(frameCenterLink.p[0], frameCenterLink.p[1], frameCenterLink.p[2]);
                double x, y, z, w;
                frameCenterLink.M.GetQuaternion(x, y, z, w);
                fcl::Quaternionf rotation(w, x, y, z);

                // printf("trans: %d %f %f %f \n", centerLinksWrtJoints[i].first, translation[0], translation[1], translation[2]);
                // printf("rot: %f %f %f %f\n", x, y, z, w);
                collisionObjects[i].setTransform(rotation, translation);
            }
            return true;
        }
        return false;
    }
    bool CheckSelfCollision::twoLinksCollide(const KDL::JntArray &q, int link1, int link2)
    {
        fcl::CollisionRequestf requestType;
        fcl::CollisionResultf collisionResult;
        fcl::collide(&collisionObjects[link1], &collisionObjects[link2], requestType, collisionResult);
        //printf("contacts: %d\n", (int) collisionResult.numContacts());
        if (collisionResult.isCollision())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    bool CheckSelfCollision::linkTableCollide(const KDL::JntArray &q, int link){
        fcl::CollisionRequestf requestType;
        fcl::CollisionResultf collisionResult;
        // printf("linksTableCollide %f %f %f\n",tableCollision[0].getTranslation()[0], tableCollision[0].getTranslation()[1], tableCollision[0].getTranslation()[2]);
        // printf("Volume: %f\n",tableCollision[0].getCollisionGeometry()->computeVolume());
        fcl::Quaternionf quat = tableCollision[0].getQuatRotation();
        // printf("linksTableCollide %f %f %f %f\n", quat.x(), quat.y(), quat.z(), quat.w());
        // printf("link %d %f %f %f\n",link, collisionObjects[link].getTranslation()[0],collisionObjects[link].getTranslation()[1],collisionObjects[link].getTranslation()[2]);
        quat = collisionObjects[link].getQuatRotation();
        // printf("link %d %f %f %f %f\n",link, quat.x(), quat.y(), quat.z(), quat.w());
        
        fcl::collide(&collisionObjects[link], &tableCollision[0], requestType, collisionResult);
        if (collisionResult.isCollision())
        {
            // printf("collsion betwenn link %d and table\n", link);
            return true;
        }
        return false;
    }


    bool CheckSelfCollision::selfCollision()
    {
        // printf("SelfCollision()\n");
        fcl::CollisionRequestf requestType;
        fcl::CollisionResultf collisionResult;
        for (int link1 = 0; link1<collisionObjects.size()-1; link1++)
        {
            int link2 = link1 + 2;
            fcl::collide(&collisionObjects[link1], &tableCollision[0], requestType, collisionResult);
            if (collisionResult.isCollision())
            {
                // printf("collsion betwenn links %d and table\n", link1);
                return true;
            }
            while (link2 < collisionObjects.size()-1)
            {   
                // printf("Lets check links %d and %d\n", link1, link2);

                fcl::collide(&collisionObjects[link1], &collisionObjects[link2], requestType, collisionResult);
                if (collisionResult.isCollision())
                {
                    // printf("collsion betwenn links %d and %d\n", link1, link2);
                    return true;
                }
                link2++;
            }
        }
        // printf("SelfCollision() not collide\n");
        return false;
    }

    double CheckSelfCollision::minDistance(){
        fcl::DistanceRequestf request;
        request.enable_nearest_points = true;
        request.enable_signed_distance = true;
        fcl::DistanceResultf distanceResult;
        double minDistance = 1000;
        for (int link1 = 0; link1<collisionObjects.size(); link1++)
        {
            int link2 = link1 + 2;
            while (link2 < collisionObjects.size())
            {   
                fcl::distance(&collisionObjects[link1],&collisionObjects[link2], request, distanceResult);
                printf("link %d %d minDistance: %f", link1, link2, distanceResult.min_distance);
                if(distanceResult.min_distance<minDistance){
                    
                    minDistance = distanceResult.min_distance;
                }
                link2++;
            }

        }
        return minDistance;

    }

    double CheckSelfCollision::twoLinksDistance(const KDL::JntArray &q, int link1, int link2){
        fcl::DistanceRequestf request;
        request.enable_nearest_points = true;
        request.enable_signed_distance = true;
        fcl::DistanceResultf distanceResult;

        fcl::distance(&collisionObjects[link1],&collisionObjects[link2], request, distanceResult);
        // fcl::distance(collisionObjects[link1].computeAABB(), )
        // fcl::distance(&collisionObjects[link1], &collisionObjects[link2], requestType, distanceResult);
        return distanceResult.min_distance;
    }



    /************************************************************************/

    /************************************************************************/

    /************************************************************************/

} // namespace roboticslab
