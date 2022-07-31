// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckSelfCollisionLibrary.hpp"

// std::shared_ptr<fcl::CollisionObject> createCollisionObject(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr, const octomap::point3d& sensor_origin_wrt_world)
// {
//   // octomap octree settings
//   const double resolution = 0.01;
//   const double prob_hit = 0.9;
//   const double prob_miss = 0.1;
//   const double clamping_thres_min = 0.12;
//   const double clamping_thres_max = 0.98;

//   std::shared_ptr<octomap::OcTree> octomap_octree = std::make_shared<octomap::OcTree>(resolution);
//   octomap_octree->setProbHit(prob_hit);
//   octomap_octree->setProbMiss(prob_miss);
//   octomap_octree->setClampingThresMin(clamping_thres_min);
//   octomap_octree->setClampingThresMax(clamping_thres_max);

//   octomap::KeySet free_cells;
//   octomap::KeySet occupied_cells;

// #if defined(_OPENMP)
// #pragma omp parallel
// #endif
//   {
// #if defined(_OPENMP)
//     auto thread_id = omp_get_thread_num();
//     auto thread_num = omp_get_num_threads();
// #else
//     int thread_id = 0;
//     int thread_num = 1;
// #endif
//     int start_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * thread_id;
//     int end_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * (thread_id + 1);
//     if (thread_id == thread_num - 1)
//     {
//       end_idx = pointcloud_ptr->size();
//     }

//     octomap::KeySet local_free_cells;
//     octomap::KeySet local_occupied_cells;

//     for (auto i = start_idx; i < end_idx; i++)
//     {
//       octomap::point3d point((*pointcloud_ptr)[i].x, (*pointcloud_ptr)[i].y, (*pointcloud_ptr)[i].z);
//       octomap::KeyRay key_ray;
//       if (octomap_octree->computeRayKeys(sensor_origin_3d, point, key_ray))
//       {
//         local_free_cells.insert(key_ray.begin(), key_ray.end());
//       }

//       octomap::OcTreeKey tree_key;
//       if (octomap_octree->coordToKeyChecked(point, tree_key))
//       {
//         local_occupied_cells.insert(tree_key);
//       }
//     }

// #if defined(_OPENMP)
// #pragma omp critical
// #endif
//     {
//       free_cells.insert(local_free_cells.begin(), local_free_cells.end());
//       occupied_cells.insert(local_occupied_cells.begin(), local_occupied_cells.end());
//     }
//   }

//   // free cells only if not occupied in this cloud
//   for (auto it = free_cells.begin(); it != free_cells.end(); ++it)
//   {
//     if (occupied_cells.find(*it) == occupied_cells.end())
//     {
//       octomap_octree->updateNode(*it, false);
//     }
//   }

//   // occupied cells
//   for (auto it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
//   {
//     octomap_octree->updateNode(*it, true);
//   }

//   auto fcl_octree = std::make_shared<fcl::OcTree>(octomap_octree);
//   std::shared_ptr<fcl::CollisionGeometry> fcl_geometry = fcl_octree;
//   return std::make_shared<fcl::CollisionObject>(fcl_geometry);
// }

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
            if (q(i) < (qmin(i)) || q(i) > (qmax(i)))
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
                printf("kinematic status %d\n", kinematics_status);
                printf("%d %f %f %f\n",centerLinksWrtJoints[i].first,frameJoint.p.x(), frameJoint.p.y(), frameJoint.p.z());

                KDL::Frame frameCenterLink = frameJoint * centerLinksWrtJoints[i].second;
                fcl::Vector3f translation(frameCenterLink.p[0], frameCenterLink.p[1], frameCenterLink.p[2]);
                double x, y, z, w;
                frameCenterLink.M.GetQuaternion(x, y, z, w);
                fcl::Quaternionf rotation(w, x, y, z);

                printf("trans: %d %f %f %f \n", centerLinksWrtJoints[i].first, translation[0], translation[1], translation[2]);
                printf("rot: %f %f %f %f\n", x, y, z, w);
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
            printf("collsion betwenn link %d and table\n", link);
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

        if(link2 == (link1+1))
            return 0.0;
        fcl::distance(&collisionObjects[link1],&collisionObjects[link2], request, distanceResult);

        // printf("%f\n",distanceResult.min_distance);
        // fcl::distance(collisionObjects[link1].computeAABB(), )
        // fcl::distance(&collisionObjects[link1], &collisionObjects[link2], requestType, distanceResult);
        return distanceResult.min_distance;
    }

    bool CheckSelfCollision::getTransformation(const fcl::CollisionObjectf & collisionObject, std::vector<double> &transformation)
    {
        fcl::Quaternionf quat = collisionObject.getQuatRotation();
        fcl::Vector3f translation = collisionObject.getTranslation();

        printf("Quat: %f %f %f %f\n", quat.x(), quat.y(), quat.z(), quat.w());
        printf("Translation: %f %f %f\n", translation[0], translation[1], translation[2]);
        transformation.push_back(quat.x());
        transformation.push_back(quat.y());
        transformation.push_back(quat.z());
        transformation.push_back(quat.w());
        transformation.push_back(translation[0]);
        transformation.push_back(translation[1]);
        transformation.push_back(translation[2]);
        return true;
    }


    /************************************************************************/

    /************************************************************************/

    /************************************************************************/

} // namespace roboticslab
