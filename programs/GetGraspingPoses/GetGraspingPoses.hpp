// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __GET_GRASPING_POSES_HPP__
#define __GET_GRASPING_POSES_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/Node.h>
#include <yarp/rosmsg/sensor_msgs/PointCloud2.h>
#include <yarp/os/Publisher.h>
#include <yarp/rosmsg/std_msgs/String.h>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>

#include <yarp/sig/all.h>
#include <yarp/sig/IntrinsicParams.h>

#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"
#include <KdlVectorConverter.hpp>

#include <yarp/sig/PointCloud.h>
#include <yarp/pcl/Pcl.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>




#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_RGBD_DEVICE "RGBDSensorClient"
#define DEFAULT_RGBD_LOCAL "/getGraspingPoses"
#define DEFAULT_RGBD_REMOTE "/xtion" // /teoSim/camera or /xtion
#define DEFAULT_WATCHDOG    1       // [s]
#define DEFAULT_RATE_MS 400
#define DEFAULT_PREFIX "/getGraspingPoses"

constexpr auto DEFAULT_ROBOT = "/teo"; // /teo or /teoSim

namespace sharon
{

/**
 * @ingroup getGraspingPoses
 *
 * @brief getGraspingPoses
 */
class GetGraspingPoses : public yarp::os::RFModule, private yarp::os::PortReader
{
public:
    virtual bool configure(yarp::os::ResourceFinder & rf) override;
    bool  openHeadDevice();
    bool  openTrunkDevice();
    bool read(yarp::os::ConnectionReader &connection);


protected:
    virtual bool interruptModule() override;
    virtual double getPeriod() override;
    virtual bool updateModule() override;


private:


    yarp::dev::PolyDriver dd;
    yarp::dev::IRGBDSensor *iRGBDSensor;

    yarp::dev::PolyDriver headDevice;
    yarp::dev::IEncoders *iHeadEncoders;

    yarp::dev::PolyDriver trunkDevice;
    yarp::dev::IEncoders *iTrunkEncoders;

    yarp::dev::PolyDriver trunkAndHeadSolverDevice;
    roboticslab::ICartesianSolver * trunkAndHeadSolverDeviceICartesianSolver;
    yarp::dev::IControlLimits *headIControlLimits;
    yarp::dev::IControlLimits *trunkIControlLimits;


    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outRgbImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > outDepthImgPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inMarchingObjectDataPort;
    // yarp::os::BufferedPort<yarp::os::Bottle> outGraspingPosesPort;   
    yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZ>> outPointCloudPort;


    yarp::os::RpcServer rpcServer;
    yarp::sig::IntrinsicParams intrinsics;

  // For publishing the transformed point cloud in ROS. Just for visualization in rviz.
    yarp::os::Node * rosNode;
    yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>* pointCloud_outTopic;
    yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>* graspingPoses_outTopic;
    std::string robot;

    int rateMs;
    void rosComputeGraspingPosesArrowAndSend(const std::string &frame_id, std::vector<pcl::PointXYZ> &centroids, std::vector<KDL::Vector> & normals);
    void rosComputeAndSendPc(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, std::string frame_id);
    bool transformPointCloud(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>::Ptr & transformed_cloud);
    Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);
    void computeGraspingPosesMilk(std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &centroids, std::vector<pcl::PointXYZ> & maxPoints,
                                    std::vector<pcl::PointXYZ> &minPoints, std::vector<std::vector<double>> & graspingPoses,
                                    std::vector<KDL::Vector> & y_vectors, std::vector<KDL::Vector> &x_vectors);


    void computeGraspingPosesCereal(std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &centroids, 
                                                std::vector<pcl::PointXYZ> & maxPoints, std::vector<pcl::PointXYZ> &minPoints, 
                                                std::vector<std::vector<double>> & graspingPoses);
                                          
    void computeGraspingPosesWaterNesquick(pcl::PointXYZ & centroid, std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &points, 
                                    std::vector<std::vector<double>> & graspingPoses, double (&cylinderShape)[2]);
    void getMinimumBoundingBoxPointCLoud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, pcl::PointXYZ & maxPoint, pcl::PointXYZ & minPoint);


    double watchdog;

    double milkBoxShape[3] = {0.065,0.075,0.225};
    double cerealBoxShape[3] = {0.08, 0.23, 0.33};
    double waterShape[2] = {0.04, 0.27}; //radius, length
    double nesquickShape[2] = {0.06, 0.14}; //radius, length



};

} // namespace sharon

#endif // __GET_GRASPING_POSES_HPP__
