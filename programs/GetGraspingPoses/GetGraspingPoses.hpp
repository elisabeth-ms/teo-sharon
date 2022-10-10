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
#include <kdl/utilities/utility.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <yarp/sig/PointCloud.h>
#include <yarp/pcl/Pcl.h>
#include <yarp/eigen/Eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <stdlib.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>

// SuperquadricLib
#include <SuperquadricLibModel/superquadricEstimator.h>
#include <yarp/math/Math.h>

#define DEFAULT_CROP_SELECTOR 0 // 1=true
#define DEFAULT_RGBD_DEVICE "RGBDSensorClient"
#define DEFAULT_RGBD_LOCAL "/getGraspingPoses"
#define DEFAULT_RGBD_REMOTE "/realsense2" // /teoSim/camera or /xtion
#define DEFAULT_WATCHDOG 1.0                 // [s]
#define DEFAULT_RATE_MS 400
#define DEFAULT_PREFIX "/getGraspingPoses"
#define DEFAULT_MIN_NPOINTS 100
#define DEFAULT_MIN_POINTS_TABLE 150
#define MAX_OBJECT_WIDTH_GRASP 0.16

constexpr auto DEFAULT_ROBOT = "/teoSim"; // /teo or /teoSim
namespace sharon
{
    struct LabelRGB
    {
        int label;
        uint32_t r;
        uint32_t g;
        uint32_t b;
    };

    struct Object
    {
        pcl::PointCloud<pcl::PointXYZRGBA> object_cloud;
        int label; /**< label assigned by the lccp algorithm*/
        uint32_t r;
        uint32_t g;
        uint32_t b;
    };

    struct ObjectSuperquadric
    {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        int label;
        std::vector<SuperqModel::Superquadric> superqs;
    };

    struct BoundingBox2d
    {
        int label; /**< label assigned by the lccp algorithm*/
        int tlx;
        int tly;
        int brx;
        int bry;
    };

    /**
     * @ingroup getGraspingPoses
     *
     * @brief getGraspingPoses
     */
    class GetGraspingPoses : public yarp::os::RFModule, private yarp::os::PortReader
    {
    public:
        virtual bool configure(yarp::os::ResourceFinder &rf) override;
        bool openHeadDevice();
        bool openTrunkDevice();
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
        roboticslab::ICartesianSolver *trunkAndHeadSolverDeviceICartesianSolver;
        yarp::dev::IControlLimits *headIControlLimits;
        yarp::dev::IControlLimits *trunkIControlLimits;

        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outRgbImgPort;
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> outDepthImgPort;
        yarp::os::BufferedPort<yarp::os::Bottle> inMarchingObjectDataPort;
        // yarp::os::BufferedPort<yarp::os::Bottle> outGraspingPosesPort;
        yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> outPointCloudPort;

        yarp::os::RpcServer rpcServer;
        yarp::sig::IntrinsicParams intrinsics;
        yarp::os::Port detectionsPort;
        yarp::os::Bottle prevDetections;
        yarp::os::Bottle currentDetections;

        yarp::os::Port mUpdateDataPort;

        // For publishing the transformed point cloud in ROS. Just for visualization in rviz.
        yarp::os::Node *rosNode;
        yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2> *pointCloud_outTopic;
        yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2> *pointCloudWithoutPlannarSurfaceTopic;
        yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2> *pointCloud_objectTopic;
        yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2> *pointCloudLccpTopic;
        yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2> *pointCloudFillingObjectsTopic;
        yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray> *bbox3d_topic;

        yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray> *graspingPoses_outTopic;
        std::string robot;
        std::string strRGBDRemote;

        int rateMs;
        void rosComputeGraspingPosesArrowAndSend(const std::string &frame_id, std::vector<pcl::PointXYZRGBA> &centroids, std::vector<KDL::Vector> &normals);
        void rosComputeAndSendPc(const yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> &pc, std::string frame_id, const yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2> &PointCloudTopic);
        bool transformPointCloud(const yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> &pc, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &transformed_cloud, bool from_camera_to_trunk);
        Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);
        
        void getMinimumBoundingBoxPointCLoud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSegmented, pcl::PointXYZRGBA &maxPoint, pcl::PointXYZRGBA &minPoint, KDL::Vector normal);
        void getMinimumBoundingBoxPointCLoud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSegmented, pcl::PointXYZRGBA &maxPoint, pcl::PointXYZRGBA &minPoint);
        void getMinimumOrientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSegmented, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &bbox);

        bool removeHorizontalSurfaceFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &wholePointCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &withoutHorizontalSurfacePointCloud);

        bool supervoxelOversegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputPointCloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);
        void XYZLPointCloudToRGBAPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud, const yarp::os::Bottle &bottle_detections, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &lccp_colored_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud);
        bool getTransformMatrix(const bool &from_camera_to_trunk, Eigen::Matrix4f &transform);
        void getPixelCoordinates(const pcl::PointXYZ &p, int &xpixel, int &ypixel);
        bool computeIntersectionOverUnion(std::array<int, 4> detection_bbox, std::array<int, 4> cluster_bbox, float &IoU);

        void fullObjectPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> &object_point_cloud, const std::string &category,
                                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud, int rgb[3]);
        void completeBox(std::vector<KDL::Vector> &normals, std::vector<pcl::PointXYZRGBA> &centroids,
                         std::vector<pcl::PointXYZRGBA> &maxPoints, std::vector<pcl::PointXYZRGBA> &minPoints,
                         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud, int rgb[3], float box_shape[3]);
        void fillBoxPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud, const pcl::PointXYZRGBA &centroid,
                               const KDL::Vector &normal, float box_sizes[3], int rgb[3]);



        void updateDetectedObjectsPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);
        bool PclPointCloudToSuperqPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &object_cloud, SuperqModel::PointCloud &point_cloud);
        void GetSuperquadricFromPointCloud(SuperqModel::PointCloud point_cloud,std::vector<SuperqModel::Superquadric> &superqs);
        void createPointCloudFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudSuperquadric, int indexDetectedObjects);
        void createGraspingPosesFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs);
        bool createBoundingBox2DFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs, std::array<int, 4> &bbox);
        void computeGraspingPoses(const std::vector<SuperqModel::Superquadric> &superqs, std::vector<std::vector<double>> &graspingPoses);
        void rosSendGraspingPoses(const std::string &frame_id, const std::vector<std::vector<double>> &graspingPoses);
        
        void rosComputeGraspingPosesArrowAndSend(const std::string &frame_id, std::vector<pcl::PointXYZRGBA> &centroids, std::vector<KDL::Vector> &xaxis, std::vector<KDL::Vector> &yaxis, std::vector<KDL::Vector> &normals);
        double watchdog;

        float milkBoxShape[3] = {0.065, 0.075, 0.225};
        float cerealBoxShape[3] = {0.08, 0.2, 0.33};
        float sugar1Shape[3] = {0.12, 0.1, 0.11};
        float sugar2Shape[3] = {0.07, 0.09, 0.16};
        float sugar3Shape[3] = {0.07, 0.07, 0.23};
        // Add sliced-bread

        float waterShape[2] = {0.04, 0.27};    // radius, length
        float nesquickShape[2] = {0.06, 0.14}; // radius, length
        float m_resolution_filling_cloud = 0.001;
        float m_grasp_width = 0.1;

        int m_width, m_height;

        yarp::os::Bottle m_category_rgb;
        yarp::os::Bottle m_bGraspingPoses;
        yarp::os::Bottle m_bObject;

        bool m_update_data = true;
        bool m_aux_update = true;

        std::vector<Object> detected_objects;
        int th_points = DEFAULT_MIN_NPOINTS;

        std::map<std::string,double> sq_model_params;
        SuperqModel::SuperqEstimatorApp estim;

        bool single_superq;
        std::string object_class;

        std::vector<ObjectSuperquadric> m_superquadric_objects;

    };

} // namespace sharon

#endif // __GET_GRASPING_POSES_HPP__
