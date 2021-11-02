// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/all.h>

#include <yarp/dev/all.h>
#include <yarp/dev/IRGBDSensor.h>

#include <yarp/sig/all.h>
#include <yarp/sig/IntrinsicParams.h>


#include <sstream>

#include <yarp/cv/Cv.h>

#include <yarp/sig/PointCloud.h>
#include <yarp/sig/PointCloudUtils.h>
#include <yarp/pcl/Pcl.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"
#include <KdlVectorConverter.hpp>


#include <yarp/os/Node.h>
#include <yarp/rosmsg/sensor_msgs/PointCloud2.h>
#include <yarp/os/Publisher.h>
#include <yarp/rosmsg/std_msgs/String.h>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>
#include <fstream>
#include <random>
#include <math.h> 
#define DEFAULT_RATE_MS 400
#define BACKGROUND_CROP false
constexpr auto DEFAULT_ROBOT = "/teo"; // teo or teoSim
using namespace std;
using namespace cv;
using namespace roboticslab::KinRepresentation;
using namespace roboticslab::KdlVectorConverter;
using namespace roboticslab;


namespace sharon {


class SegmentorThread : public yarp::os::PeriodicThread, private yarp::os::PortReader
{
public:
    SegmentorThread() : PeriodicThread(DEFAULT_RATE_MS*0.001) {}

    void setIRGBDSensor(yarp::dev::IRGBDSensor * _iRGBDSensor);
    void setOutRgbImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutRgbImgPort);
    void setOutDepthImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > * _pOutDepthImgPort);
    void setInMarchingObjectDataPort(yarp::os::BufferedPort<yarp::os::Bottle>* _pInMarchingObjectDataPort);
    void setOutPointCloudPort(yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZ>> * _pOutPointCloudPort);
    void setRpcServer(yarp::os::RpcServer *_pRpcServer);
    bool read(yarp::os::ConnectionReader &connection);

    // void setOutGraspingPosesPort(yarp::os::BufferedPort<yarp::os::Bottle>* _pOutGraspingPosesPort);

    bool init(yarp::os::ResourceFinder &rf);
    void setHeadIEncoders(yarp::dev::IEncoders *_iEncoders);
    void setTrunkIEncoders(yarp::dev::IEncoders *_iEncoders);
    void setICartesianSolver( ICartesianSolver * _iCartesianSolver);
    Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);

    void getMinimumBoundingBoxPointCLoud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, pcl::PointXYZ & maxPoint, pcl::PointXYZ & minPoint);

    void computeGraspingPosesMilk(std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &centroids, std::vector<pcl::PointXYZ> & maxPoints,
                                    std::vector<pcl::PointXYZ> &minPoints, std::vector<std::vector<double>> & graspingPoses,
                                    std::vector<KDL::Vector> & y_vectors, std::vector<KDL::Vector> &x_vectors);

    void computeGraspingPosesCereal(std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &centroids, 
                                                std::vector<pcl::PointXYZ> & maxPoints, std::vector<pcl::PointXYZ> &minPoints, 
                                                std::vector<std::vector<double>> & graspingPoses);
                                             
    void computeGraspingPosesWaterNesquick(pcl::PointXYZ & centroid, std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &points, 
                                    std::vector<std::vector<double>> & graspingPoses, double (&cylinderShape)[2]);

    void addGraspingPointsAndNormalsToViewer(std::vector<pcl::PointXYZ> &centroids, std::vector<KDL::Vector> & normals, pcl::visualization::PCLVisualizer::Ptr viewer);

    void rosComputeAndSendPc(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, std::string frame_id);
    bool transformPointCloud(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>::Ptr & transformed_cloud);
    void rosComputeGraspingPosesArrowAndSend(const std::string &frame_id, std::vector<pcl::PointXYZ> &centroids, std::vector<KDL::Vector> & normals);


private:
    void run() override; // The periodical function

    yarp::sig::IntrinsicParams intrinsics;

    yarp::dev::IEncoders *iHeadEncoders;
    yarp::dev::IEncoders *iTrunkEncoders;

    ICartesianSolver *trunkAndHeadICartesianSolver;

    yarp::dev::IRGBDSensor *iRGBDSensor;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutRgbImgPort;  // Port for sending the cropped image of the rgb frames
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > *pOutDepthImgPort;  // Port for sending the cropped image of the depth frames
    yarp::os::BufferedPort<yarp::os::Bottle> *pInMarchingObjectDataPort;
    yarp::os::RpcServer * pRpcServer;

    // yarp::os::BufferedPort<yarp::os::Bottle> *pOutGraspingPosesPort;
    // yarp::os::Port *pOutPort;
    yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZ>> *pOutPointCloudPort;
    double milkBoxShape[3] = {0.065,0.075,0.225};
    double cerealBoxShape[3] = {0.08, 0.23, 0.33};
    double waterShape[2] = {0.04, 0.27}; //radius, length
    double nesquickShape[2] = {0.06, 0.14}; //radius, length

    pcl::visualization::PCLVisualizer::Ptr viewer; 
    // For publishing the transformed point cloud in ROS. Just for visualization in rviz.
    yarp::os::Node * rosNode;
    yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>* pointCloud_outTopic;
    yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>* graspingPoses_outTopic;


    std::mutex mutexCloud;

};
}
#endif //__SEGMENTOR_THREAD_HPP__
