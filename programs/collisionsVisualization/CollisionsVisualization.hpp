// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLLISIONS_VISUALIZATION_HPP__
#define __COLLISIONS_VISUALIZATION_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/dev/all.h>

#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/rosmsg/std_msgs/String.h>
#include <yarp/rosmsg/visualization_msgs/Marker.h>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>

#include "CheckSelfCollisionLibrary.hpp"

#define DEFAULT_DEVICE_NAME "trunkAndRightArm"
#define DEFAULT_ROBOT "teo"
#define AXIAL_SHOULDER_LINK_LENGTH 0.305
#define AXIAL_SHOULDER_LINK_RADIUS 0.075
#define FRONTAL_ELBOW_LINK_LENGTH 0.215
#define FRONTAL_ELBOW_LINK_RADIUS 0.07
#define FRONTAL_WRIST_LINK_LENGTH 0.16
#define FRONTAL_WRIST_LINK_WIDTH 0.17
#define FRONTAL_WRIST_LINK_HEIGHT 0.1

namespace sharon
{

/**
 * @ingroup collisionsVisualization
 *
 * @brief collisionsVisualization
 */
class CollisionsVisualization : public yarp::os::RFModule
{
public:
    virtual bool configure(yarp::os::ResourceFinder & rf) override;
    bool openDevices();

protected:
    virtual bool interruptModule() override;
    virtual double getPeriod() override;
    virtual bool updateModule() override;
    std::string m_robot;
    std::string m_deviceName;
    std::string m_frameId;
    yarp::dev::PolyDriver m_device;
    yarp::dev::IEncoders *m_iEncoders;
    yarp::dev::IControlLimits *m_iControlLimits;
    int m_numJoints;

    KDL::Chain m_chain;
    KDL::JntArray m_qmin;
    KDL::JntArray m_qmax;
    std::vector<fcl::CollisionObjectf> m_collisionObjects;
    std::vector<std::array<float, 3>> m_offsetCollisionObjects;
    std::vector<fcl::CollisionObjectf> m_tableCollision;
    std::vector<std::array<float, 3>> m_boxSizes;
    std::vector<int>m_collisionSegments;


            
    std::vector<KDL::Frame> m_centerLinkWrtJoint;

    typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
    sharon::CheckSelfCollision *m_checkSelfCollision;


private:
    //Rviz visualization
    yarp::os::Node * m_rosNode;
    yarp::rosmsg::visualization_msgs::MarkerArray m_markerArray;

    bool addMarker(const int numberLink, const std::vector<double>& transformation, const std::array<float,3> & boxSize);
    yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>* m_collisionObjectsTopic;



};

} // namespace sharon

#endif // __COLLISIONS_VISUALIZATION_HPP__