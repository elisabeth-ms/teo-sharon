// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GetGraspingPoses.hpp"
#include <pcl/common/centroid.h>

#include <cstdio>

#include <mutex>

#include <chrono>
using namespace std::chrono;

std::mutex mtx;

using namespace sharon;
using SuperVoxelAdjacencyList = pcl::LCCPSegmentation<pcl::PointXYZRGBA>::SupervoxelAdjacencyList;

constexpr auto VOCAB_FAIL = yarp::os::createVocab32('f', 'a', 'i', 'l');
constexpr auto VOCAB_CMD_PAUSE = yarp::os::createVocab32('p', 'a', 'u', 's');
constexpr auto VOCAB_CMD_RESUME = yarp::os::createVocab32('r', 's', 'm');
constexpr auto VOCAB_CMD_GET_SUPERQUADRICS_BBOXES = yarp::os::createVocab32('g', 's', 'b', 'b');
constexpr auto VOCAB_CMD_GET_GRASPING_POSES = yarp::os::createVocab32('g', 'g', 'p');
constexpr auto VOCAB_CMD_GET_SUPERQUADRICS = yarp::os::createVocab32('g', 's', 'u', 'p');
constexpr auto VOCAB_CMD_REMOVE_SUPERQUADRIC = yarp::os::createVocab32('r', 's', 'u', 'p');

/************************************************************************/

bool GetGraspingPoses::configure(yarp::os::ResourceFinder &rf)
{
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    strRGBDRemote = DEFAULT_RGBD_REMOTE;
    watchdog = DEFAULT_WATCHDOG; // double

    robot = rf.check("robot", yarp::os::Value(DEFAULT_ROBOT), "name of /robot to be used").asString();

    m_update_data = true;
    printf("GetGraspingPoses options:\n");
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    printf("\t--RGBDDevice (device we create, default: \"%s\")\n", strRGBDDevice.c_str());
    printf("\t--RGBDLocal (if accesing remote, local port name, default: \"%s\")\n", strRGBDLocal.c_str());
    printf("\t--RGBDRemote (if accesing remote, remote port name, default: \"%s\")\n", strRGBDRemote.c_str());
    printf("\t--watchdog ([s] default: \"%f\")\n", watchdog);

    if (rf.check("RGBDDevice"))
        strRGBDDevice = rf.find("RGBDDevice").asString();
    if (rf.check("RGBDLocal"))
        strRGBDLocal = rf.find("RGBDLocal").asString();
    if (rf.check("RGBDRemote"))
        strRGBDRemote = rf.find("RGBDRemote").asString();
    if (rf.check("watchdog"))
        watchdog = rf.find("watchdog").asFloat64();

    printf("RgbdDetection using RGBDDevice: %s, RGBDLocal: %s, RGBDRemote: %s.\n",
           strRGBDDevice.c_str(), strRGBDLocal.c_str(), strRGBDRemote.c_str());
    printf("RgbdDetection using watchdog: %f.\n", watchdog);

    yarp::os::Property options;
    options.fromString(rf.toString());    //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
    options.put("device", strRGBDDevice); //-- Important to override in case there is a "device" in the future
    options.put("localImagePort", strRGBDLocal + "/rgbImage:i");
    options.put("localDepthPort", strRGBDLocal + "/depthImage:i");
    options.put("localRpcPort", strRGBDLocal + "/rpc:o");
    options.put("remoteImagePort", strRGBDRemote + "/rgbImage:o");
    options.put("remoteDepthPort", strRGBDRemote + "/depthImage:o");
    options.put("remoteRpcPort", strRGBDRemote + "/rpc:i");
    // if(rf.check("noMirror")) options.put("noMirror",1);  //-- Replaced by options.fromString( rf.toString() );

    if (!dd.open(options))
    {
        yError("Bad RGBDDevice \"%s\"...\n", strRGBDDevice.c_str());
        return false;
    }
    yInfo("RGBDDevice available.\n");

    if (!dd.view(iRGBDSensor))
    {
        yError("RGBDDevice bad view.\n");
        return false;
    }
    yInfo("RGBDDevice ok view.\n");

    //-----------------OPEN LOCAL PORTS------------//
    std::string portPrefix(DEFAULT_PREFIX);
    portPrefix += strRGBDRemote;
    if (!outRgbImgPort.open(portPrefix + "/croppedImg:o"))
    {
        yError("Bad outRgbImgPort.open\n");
        return false;
    }
    if (!outDepthImgPort.open(portPrefix + "/croppedDepthImg:o"))
    {
        yError("Bad outDepthImgPort.open\n");
        return false;
    }

    if (!outPointCloudPort.open(portPrefix + "/pointCloud:o"))
    {
        yError("Bad outPointCloudPort.open\n");
        return false;
    }

    if (robot == "/teoSim")
        openHeadDevice();

    openTrunkDevice();

    yarp::os::Bottle qrMin;
    yarp::os::Bottle qrMax;

    for (unsigned int joint = 0; joint < 2; joint++)
    {
        double min, max;
        trunkIControlLimits->getLimits(joint, &min, &max);
        qrMin.addFloat64(min);
        qrMax.addFloat64(max);
        yInfo("Joint %d limits: [%f,%f]", joint, min, max);
    }

    if (robot == "/teoSim")
    {
        for (unsigned int joint = 0; joint < 2; joint++)
        {
            double min, max;
            headIControlLimits->getLimits(joint, &min, &max);
            qrMin.addFloat64(min);
            qrMax.addFloat64(max);
            yInfo("Joint %d limits: [%f,%f]", joint, min, max);
        }
    }
    else
    {

        for (unsigned int joint = 0; joint < 2; joint++)
        {
            if (strRGBDRemote == "/realsense2")
            {
                qrMin.addFloat64(-60);
                qrMax.addFloat64(60.0);
            }
            else
            {
                qrMin.addFloat64(-28);
                qrMax.addFloat64(28.0);
            }
        }
    }
    yarp::os::Property trunkAndHeadSolverOptions;
    rf.setDefaultContext("kinematics"); // context to find kinematic config files
    std::string trunkHeadKinPath = rf.findFileByName("teo-trunk-head.ini");
    if (strRGBDRemote == "/realsense2")
    {
        std::cout << "Using teo-trunk-realsense-tripode.ini" << std::endl;
        trunkHeadKinPath = rf.findFileByName("teo-trunk-realsense-tripode.ini");
    }

    trunkAndHeadSolverOptions.fromConfigFile(trunkHeadKinPath);
    trunkAndHeadSolverOptions.put("device", "KdlSolver");
    trunkAndHeadSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    trunkAndHeadSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    trunkAndHeadSolverOptions.put("ik", "st"); // to use screw theory IK

    trunkAndHeadSolverDevice.open(trunkAndHeadSolverOptions);

    if (!trunkAndHeadSolverDevice.isValid())
    {
        yError() << "KDLSolver solver device for trunk and head is not valid";
        return false;
    }

    if (!trunkAndHeadSolverDevice.view(trunkAndHeadSolverDeviceICartesianSolver))
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    std::string prefix(DEFAULT_PREFIX);

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        yError() << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }

    // if (!detectionsPort.open(portPrefix + "/rgbdObjectDetection/state:i"))
    // {
    //     yError() << "Unable to open " << detectionsPort.getName();
    //     return false;
    // }

    // if (!yarp::os::Network::connect("/rgbdObjectDetection/state:o", detectionsPort.getName()))
    // {
    //     yError() << "Unable to connect /rgbdObjectDetection/state:o to " << detectionsPort.getName();
    //     return false;
    // }

    // if(!mUpdateDataPort.open(portPrefix+"/updateData/command:i")){
    //     yError() << "Unable to open "<<mUpdateDataPort.getName();
    //     return false;
    // }

    // if(!yarp::os::Network::connect("/updateData/command:o", mUpdateDataPort.getName())){
    //     yError() << "Unable to connect /updateData/command:o to "<<mUpdateDataPort.getName();
    //     return false;
    // }

    yarp::os::Property depthIntrinsicParams;

    if (!iRGBDSensor->getDepthIntrinsicParam(depthIntrinsicParams))
    {
        yError("Cannot get depth params\n");
        return false;
    }

    yarp::os::Property colorIntrinsicParams;
    if (!iRGBDSensor->getRgbIntrinsicParam(colorIntrinsicParams))
    {
        yError("Cannot get color params\n");
        return false;
    }

    if (strRGBDRemote == "/realsense2")
    {
        // Realsense camera aligns depth frame and color frame, so we need to use the intrinic
        // params for the color. Then, these params are used for computing the pointcloud.
        intrinsics.fromProperty(colorIntrinsicParams);
    }
    else
    {
        intrinsics.fromProperty(depthIntrinsicParams);
    }

    yInfo() << "colorIntrinsicParams: " << colorIntrinsicParams.toString();
    yInfo() << "depthIntrinsicParams: " << depthIntrinsicParams.toString();

    rosNode = new yarp::os::Node(DEFAULT_PREFIX);

    pointCloud_outTopic = new yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>;

    if (pointCloud_outTopic->topic("/depthToPointCloud") == false)
    {
        yError("Error opening depthToPointCloud topic.\n");
    }
    else
    {
        yInfo("Opening depthToPointCloud topic succesfully.\n");
    }

    pointCloudFillingObjectsTopic = new yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>;

    if (pointCloudFillingObjectsTopic->topic("/pointCloudFillingObjects") == false)
    {
        yError("Error opening pointCloudFillingObjects topic.\n");
    }
    else
    {
        yInfo("Opening pointCloudFillingObjects topic succesfully.\n");
    }

    pointCloudWithoutPlannarSurfaceTopic = new yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>;

    if (pointCloudWithoutPlannarSurfaceTopic->topic("/PointCloudWithoutPlannarSurface") == false)
    {
        yError("Error opening PointCloudWithoutPlannarSurface topic.\n");
    }
    else
    {
        yInfo("Opening PointCloudWithoutPlannarSurface topic succesfully.\n");
    }

    pointCloudLccpTopic = new yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>;

    if (pointCloudLccpTopic->topic("/PointCloudLccp") == false)
    {
        yError("Error opening PointCloudLccp topic.\n");
    }
    else
    {
        yInfo("Opening PointCloudLccp topic succesfully.\n");
    }

    pointCloud_objectTopic = new yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>;

    if (pointCloud_objectTopic->topic("/pointCloudObject") == false)
    {
        yError("Error opening pointCloudObject topic.\n");
    }
    else
    {
        yInfo("Opening pointCloudObject topic succesfully.\n");
    }

    graspingPoses_outTopic = new yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>;

    if (graspingPoses_outTopic->topic("/graspingPoses") == false)
    {
        yError("Error opening graspingPoses topic.\n");
    }
    else
    {
        yInfo("Opening graspingPoses topic succesfully.\n");
    }

    bbox3d_topic = new yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>;
    if (bbox3d_topic->topic("/bbox3d") == false)
    {
        yError("Error opening bbox3d topic.\n");
    }
    else
    {
        yInfo("Opening bbox3d topic succesfully.\n");
    }

    /* ------ Set Superquadric Model parameters ------ */

    int print_level_superq = rf.check("print_level_superq", yarp::os::Value(0)).asInt32();
    object_class = rf.check("object_class", yarp::os::Value("default")).toString();
    single_superq = rf.check("single_superq", yarp::os::Value(true)).asBool();
    sq_model_params["tol"] = rf.check("tol_superq", yarp::os::Value(1e-6)).asFloat64();
    sq_model_params["optimizer_points"] = rf.check("optimizer_points", yarp::os::Value(200)).asInt32();
    sq_model_params["random_sampling"] = rf.check("random_sampling", yarp::os::Value(false)).asBool();
    sq_model_params["max_iter"] = rf.check("max_iter", yarp::os::Value(10000000)).asInt32();
    sq_model_params["merge_model"] = rf.check("merge_model", yarp::os::Value(true)).asBool();
    sq_model_params["minimum_points"] = rf.check("minimum_points", yarp::os::Value(200)).asInt32();
    sq_model_params["fraction_pc"] = rf.check("fraction_pc", yarp::os::Value(2)).asInt32();
    sq_model_params["threshold_axis"] = rf.check("tol_threshold_axissuperq", yarp::os::Value(0.7)).asFloat64();
    sq_model_params["threshold_section1"] = rf.check("threshold_section1", yarp::os::Value(0.6)).asFloat64();
    sq_model_params["threshold_section2"] = rf.check("threshold_section2", yarp::os::Value(0.03)).asFloat64();

    estim.SetNumericValue("tol", sq_model_params["tol"]);
    estim.SetIntegerValue("print_level", print_level_superq);
    estim.SetStringValue("object_class", object_class);
    estim.SetIntegerValue("optimizer_points", int(sq_model_params["optimizer_points"]));
    estim.SetBoolValue("random_sampling", bool(sq_model_params["random_sampling"]));

    estim.SetBoolValue("merge_model", sq_model_params["merge_model"]);
    estim.SetIntegerValue("minimum_points", sq_model_params["minimum_points"]);
    estim.SetIntegerValue("fraction_pc", sq_model_params["fraction_pc"]);
    estim.SetNumericValue("threshold_axis", sq_model_params["threshold_axis"]);
    estim.SetNumericValue("threshold_section1", sq_model_params["threshold_section1"]);
    estim.SetNumericValue("threshold_section2", sq_model_params["threshold_section2"]);

    this->detected_objects.resize(0);

    rpcServer.setReader(*this);

    yInfo("--- end: configure\n");

    return true;
}

/************************************************************************/

double GetGraspingPoses::getPeriod()
{
    return watchdog; // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool GetGraspingPoses::updateModule()
{
    printf("GetGraspingPoses alive...\n");

    if (m_update_data)
    {
        yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

        bool depth_ok = iRGBDSensor->getDepthImage(depthFrame);
        if (depth_ok == false)
        {
            yError() << "getDepthImage failed";
            return false;
        }
        yarp::sig::FlexImage rgbImage;
        bool rgb_ok = iRGBDSensor->getRgbImage(rgbImage);
        if (rgb_ok == false)
        {
            yError() << "getRgbImage failed";
            return false;
        }
        yarp::sig::ImageOf<yarp::sig::PixelRgb> colorFrame;
        colorFrame.copy(rgbImage);

        int min_x = 0;
        int min_y = 0;
        int max_x = depthFrame.width();
        int max_y = depthFrame.height();
        m_width = depthFrame.width();
        m_height = depthFrame.height();

        // mutexCloud.lock();
        yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> pcFiltered;
        pcFiltered.resize(depthFrame.width(), depthFrame.height());

        for (size_t i = 0, u = min_x; u < max_x; i++, u++)
        {
            for (size_t j = 0, v = min_y; v < max_y; j++, v++)
            {
                if (depthFrame.pixel(u, v) < 1.2)
                { // TODO: PARAMETER MAX DISTANCE FILTER
                    pcFiltered(i, j).x = depthFrame.pixel(u, v);
                    pcFiltered(i, j).y = -(u - intrinsics.principalPointX) / intrinsics.focalLengthX * depthFrame.pixel(u, v);
                    pcFiltered(i, j).z = -(v - intrinsics.principalPointY) / intrinsics.focalLengthY * depthFrame.pixel(u, v);

                    yarp::sig::PixelRgb rgb = colorFrame.pixel(u, j);
                    pcFiltered(i, j).r = rgb.r;
                    pcFiltered(i, j).g = rgb.g;
                    pcFiltered(i, j).b = rgb.b;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());

        auto start = high_resolution_clock::now();
        if (transformPointCloud(pcFiltered, transformed_cloud, true))
        {
            //-------------------For testing----------------------------------------------------------//
            yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloud;
            yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(*transformed_cloud, yarpCloud);

            // yInfo()<<"Point cloud transformed";
            rosComputeAndSendPc(yarpCloud, "waist", *pointCloud_outTopic);

            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
            sor.setInputCloud(transformed_cloud);
            sor.setLeafSize(0.005f, 0.005f, 0.005f);
            sor.filter(*transformed_cloud_filtered);

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr without_horizontal_surface_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            bool table_removed = removeHorizontalSurfaceFromPointCloud(transformed_cloud_filtered, without_horizontal_surface_cloud);

            if (table_removed)
            {

                yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloudWithoutHorizontalSurface;
                yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(*without_horizontal_surface_cloud, yarpCloudWithoutHorizontalSurface);

                rosComputeAndSendPc(yarpCloudWithoutHorizontalSurface, "waist", *pointCloudWithoutPlannarSurfaceTopic);

                pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
                supervoxelOversegmentation(without_horizontal_surface_cloud, lccp_labeled_cloud);

                yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloudLccp;
                yarp::pcl::fromPCL<pcl::PointXYZL, yarp::sig::DataXYZRGBA>(*lccp_labeled_cloud, yarpCloudLccp);
                rosComputeAndSendPc(yarpCloudLccp, "waist", *pointCloudLccpTopic);

                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr lccp_colored_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

                yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloudSuperquadric;
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);

                m_superquadric_objects.clear();
                if (lccp_labeled_cloud->points.size() != 0)
                {

                    updateDetectedObjectsPointCloud(lccp_labeled_cloud);
                    std::vector<std::vector<double>> graspingPoses;

                    for (unsigned int idx = 0; idx < detected_objects.size(); idx++)
                    {
                        SuperqModel::PointCloud point_cloud;
                        PclPointCloudToSuperqPointCloud(detected_objects[idx].object_cloud, point_cloud);
                        std::vector<SuperqModel::Superquadric> superqs;
                        GetSuperquadricFromPointCloud(point_cloud, superqs);
                        yInfo() << "create";

                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr auxCloudSuperquadric(new pcl::PointCloud<pcl::PointXYZRGBA>);

                        createPointCloudFromSuperquadric(superqs, auxCloudSuperquadric, idx);
                        *cloudSuperquadric += *auxCloudSuperquadric;

                        ObjectSuperquadric object_superquadric;
                        object_superquadric.label = detected_objects[idx].label;
                        object_superquadric.superqs = superqs;
                        object_superquadric.cloud = *auxCloudSuperquadric;

                        m_superquadric_objects.push_back(object_superquadric);
                        // computeGraspingPoses(superqs, graspingPoses);
                    }
                    yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(*cloudSuperquadric, yarpCloudSuperquadric);
                    yInfo() << yarpCloudSuperquadric.size();
                    rosComputeAndSendPc(yarpCloudSuperquadric, "waist", *pointCloudFillingObjectsTopic);
                }
            }
        }
    }
    return true;
}

/************************************************************************/

bool GetGraspingPoses::interruptModule()
{
    printf("GetGraspingPoses closing...\n");
    // segmentorThread.stop();
    outRgbImgPort.interrupt();
    outDepthImgPort.interrupt();
    inMarchingObjectDataPort.interrupt();
    detectionsPort.interrupt();
    outPointCloudPort.interrupt();

    dd.close();
    outRgbImgPort.close();
    outDepthImgPort.close();
    inMarchingObjectDataPort.close();
    detectionsPort.close();
    outPointCloudPort.interrupt();
    return true;
}

/************************************************************************/

bool GetGraspingPoses::openHeadDevice()
{
    yarp::os::Property headOptions;
    headOptions.put("device", "remote_controlboard");
    headOptions.put("remote", robot + "/head");
    headOptions.put("local", DEFAULT_PREFIX + robot + "/head");
    headDevice.open(headOptions);

    if (!headDevice.isValid())
    {
        printf("head remote_controlboard instantiation not worked.\n");
        return false;
    }

    if (!headDevice.view(iHeadEncoders))
    {
        printf("view(iEncoders) not worked.\n");
        return false;
    }

    if (!headDevice.view(headIControlLimits))
    {
        printf("Could not view iControlLimits.\n");
        return false;
    }
    return true;
}
/************************************************************************/
bool GetGraspingPoses::openTrunkDevice()
{
    yarp::os::Property trunkOptions;
    trunkOptions.put("device", "remote_controlboard");
    trunkOptions.put("remote", robot + "/trunk");
    trunkOptions.put("local", DEFAULT_PREFIX + robot + "/trunk");
    trunkDevice.open(trunkOptions);

    if (!trunkDevice.isValid())
    {
        printf("trunk remote_controlboard instantiation not worked.\n");
        return false;
    }

    if (!trunkDevice.view(iTrunkEncoders))
    {
        printf("view(iEncoders) not worked.\n");
        return false;
    }

    if (!trunkDevice.view(trunkIControlLimits))
    {
        printf("Could not view iControlLimits.\n");
        return false;
    }
    return true;
}

bool GetGraspingPoses::getTransformMatrix(const bool &from_camera_to_trunk, Eigen::Matrix4f &transform)
{

    yInfo() << "Lets get transform matrix";
    // yInfo()<<"Lets check if the head encoders are read";
    int numHeadJoints = 2;
    if (robot == "/teoSim")
        iHeadEncoders->getAxes(&numHeadJoints);

    std::vector<double> currentHeadQ(numHeadJoints);

    if (robot == "/teoSim")
    {
        if (!iHeadEncoders->getEncoders(currentHeadQ.data()))
        {
            yError() << "getEncoders() failed";
            return false;
        }
    }
    else
    {
        if (strRGBDRemote == "/realsense2")
        {
            currentHeadQ[0] = 0.0;
            currentHeadQ[1] = -0.0;
        }
        else
        {
            currentHeadQ[0] = 0.0;
            currentHeadQ[1] = -28.0;
        }
    }

    int numTrunkJoints;
    if (!iTrunkEncoders->getAxes(&numTrunkJoints))
    {
        yError() << "getAxes() failed";
        return false;
    }

    std::vector<double> currentTrunkQ(numTrunkJoints);

    if (!iTrunkEncoders->getEncoders(currentTrunkQ.data()))
    {
        yError() << "getEncoders() failed";
        return false;
    }
    yInfo() << "CurrentTrunkQ: " << currentTrunkQ[0] << " " << currentTrunkQ[1];

    /** --------------------------------------------------------------------------- **/

    std::vector<double> currentQ(numTrunkJoints + numHeadJoints);
    for (int i = 0; i < numTrunkJoints; i++)
    {
        currentQ[i] = currentTrunkQ[i];
    }
    for (int i = 0; i < numHeadJoints; i++)
    {
        currentQ[i + 2] = -currentHeadQ[i];
    }

    std::vector<double> currentX;
    if (!trunkAndHeadSolverDeviceICartesianSolver->fwdKin(currentQ, currentX))
    {
        yError() << "fwdKin failed";
        return false;
    }

    KDL::Frame frame_head_trunk = roboticslab::KdlVectorConverter::vectorToFrame(currentX);
    KDL::Frame frame;

    if (DEFAULT_ROBOT == "/teo")
    {
        if (strRGBDRemote == "/xtion")
        {
            KDL::Frame frame_camera_head;
            frame_camera_head = frame_camera_head * KDL::Frame(KDL::Vector(0, 0, 0.059742));
            frame_camera_head = frame_camera_head * KDL::Frame(KDL::Vector(0.10, 0, 0.0));
            // frame_camera_head = frame_camera_head*KDL::Frame(KDL::Rotation::RotZ(-M_PI_2));
            // frame_camera_head = frame_camera_head*KDL::Frame(KDL::Rotation::RotX(-M_PI_2));
            frame_camera_head = frame_camera_head * KDL::Frame(KDL::Vector(0.0, /*-0.018*/ -0.026, 0.0));
            frame = frame_head_trunk * frame_camera_head;
        }
        else if (strRGBDRemote == "/realsense2")
        {
            std::cout << "/realsense2" << std::endl;
            frame = frame_head_trunk;
        }
    }
    else if (DEFAULT_ROBOT == "/teoSim")
    {
        frame = frame_head_trunk;
    }

    if (!from_camera_to_trunk)
        frame = frame.Inverse();

    transform = KDLToEigenMatrix(frame);

    return true;
}

/************************************************************************/
bool GetGraspingPoses::transformPointCloud(const yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> &pc, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &transformed_cloud, bool from_camera_to_trunk)
{

    Eigen::Matrix4f transform;
    if (!getTransformMatrix(from_camera_to_trunk, transform))
    {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (yarp::pcl::toPCL<yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(pc, *cloud))
    {

        // pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        // sor.setInputCloud(cloud);
        // sor.setLeafSize(0.005f, 0.005f, 0.005f);
        // sor.filter(*cloud);

        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
        return true;
    }
    else
    {
        yError() << "Could not transform pointcloud";
        return false;
    }
}
/************************************************************************/

Eigen::Matrix4f GetGraspingPoses::KDLToEigenMatrix(const KDL::Frame &p)
{
    Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            b(i, j) = p.M(i, j);
        }
        b(i, 3) = p.p(i);
    }
    //    std::cout << "Here is the matrix m:\n" << b << std::endl;
    return b;
}
/************************************************************************/
void GetGraspingPoses::rosComputeAndSendPc(const yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> &pc, std::string frame_id, const yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2> &PointCloudTopic)
{
    yInfo() << "Compute and send pointcloud in ROS topic: " << PointCloudTopic.getName();
    yarp::rosmsg::sensor_msgs::PointCloud2 rosPC_data;
    static int counter = 0;
    rosPC_data.header.stamp.nsec = 0;
    rosPC_data.header.stamp.sec = 0;
    rosPC_data.header.seq = counter++;
    rosPC_data.header.frame_id = frame_id;

    rosPC_data.fields.resize(4);
    rosPC_data.fields[0].name = "x";
    rosPC_data.fields[0].offset = 0;   // offset in bytes from start of each point
    rosPC_data.fields[0].datatype = 7; // 7 = FLOAT32
    rosPC_data.fields[0].count = 1;    // how many FLOAT32 used for 'x'

    rosPC_data.fields[1].name = "y";
    rosPC_data.fields[1].offset = 4;   // offset in bytes from start of each point
    rosPC_data.fields[1].datatype = 7; // 7 = FLOAT32
    rosPC_data.fields[1].count = 1;    // how many FLOAT32 used for 'y'

    rosPC_data.fields[2].name = "z";
    rosPC_data.fields[2].offset = 8;   // offset in bytes from start of each point
    rosPC_data.fields[2].datatype = 7; // 7 = FLOAT32
    rosPC_data.fields[2].count = 1;    // how many FLOAT32 used for 'z'

    rosPC_data.fields[3].name = "rgb";
    rosPC_data.fields[3].offset = 16;
    rosPC_data.fields[3].datatype = 7;
    rosPC_data.fields[3].count = 1;

#if defined(YARP_BIG_ENDIAN)
    rosPC_data.is_bigendian = true;
#elif defined(YARP_LITTLE_ENDIAN)
    rosPC_data.is_bigendian = false;
#else
#error "Cannot detect endianness"
#endif

#if 0
    rosPC_data.height=1;
    rosPC_data.width=pc.size();
#else
    rosPC_data.height = pc.height();
    rosPC_data.width = pc.width();
#endif

    rosPC_data.point_step = sizeof(yarp::sig::DataXYZRGBA);
    std::vector<unsigned char> vec(pc.getRawData(), pc.getRawData() + pc.dataSizeBytes());
    rosPC_data.data = vec;
    rosPC_data.width = pc.width() * pc.height();
    rosPC_data.height = 1;
    rosPC_data.is_dense = pc.isDense(); // what this field actually means?? When is it false??
    rosPC_data.point_step = sizeof(yarp::sig::DataXYZRGBA);
    rosPC_data.row_step = static_cast<std::uint32_t>(sizeof(yarp::sig::DataXYZRGBA) * rosPC_data.width);

    // yCDebug(LASER_FROM_POINTCLOUD)<<elem <<yelem;
    yInfo("Publish...\n");
    PointCloudTopic.write(rosPC_data);
}

bool GetGraspingPoses::removeHorizontalSurfaceFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &wholePointCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &withoutHorizontalSurfacePointCloud)
{

    // Coefficients and inliners objects for tge ransac plannar model
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(2000);
    seg.setDistanceThreshold(0.02);
    seg.setAxis(Eigen::Vector3f::UnitX());
    seg.setEpsAngle(0.01);

    seg.setInputCloud(wholePointCloud);
    seg.segment(*inliers, *coefficients);
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    extract.setInputCloud(wholePointCloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // Extract the inliers
    extract.filter(*withoutHorizontalSurfacePointCloud);

    if ((wholePointCloud->size() - withoutHorizontalSurfacePointCloud->size()) > DEFAULT_MIN_POINTS_TABLE)
        return true;
    return false;
}

bool GetGraspingPoses::supervoxelOversegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputPointCloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud)
{

    // ------------------------------- Compute normals of the the input cloud ------------------------------------------------- //

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(inputPointCloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    ne.setSearchMethod(tree);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm (TODO: Pass as parameter)
    ne.setRadiusSearch(0.03);
    // Compute the features
    ne.compute(*input_normals_ptr);

    // yInfo("Input cloud has %d normals", input_normals_ptr->size());

    // TODO: Change to yarp params
    float voxel_resolution = 0.01f;
    float seed_resolution = 0.02f;
    float color_importance = 0.0f;
    float spatial_importance = 2.0f;
    float normal_importance = 2.0f;
    bool use_single_cam_transform = false;
    bool use_supervoxel_refinement = false;

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold = 20;
    float smoothness_threshold = 0.2;
    std::uint32_t min_segment_size = 10;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = true;

    unsigned int k_factor = 0;
    if (use_extended_convexity)
        k_factor = 1;

    pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(use_single_cam_transform);
    super.setInputCloud(inputPointCloud);
    super.setNormalCloud(input_normals_ptr);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);
    std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;

    if (use_supervoxel_refinement)
    {
        // PCL_INFO ("Refining supervoxels\n");
        super.refineSupervoxels(2, supervoxel_clusters);
    }

    // PCL_INFO ("Extracting supervoxels\n");
    super.extract(supervoxel_clusters);

    std::stringstream temp;
    temp << "  Nr. Supervoxels: " << supervoxel_clusters.size() << "\n";
    PCL_INFO(temp.str().c_str());
    yInfo() << "Max label: " << super.getMaxLabel();
    // PCL_INFO ("Getting supervoxel adjacency\n");
    std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();

    /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZRGBA>::makeSupervoxelNormalCloud(supervoxel_clusters);

    // pcl::io::savePCDFile ("svcloud.pcd", *sv_centroid_normal_cloud, true);

    PCL_INFO("Starting Segmentation\n");
    yInfo() << "supervoxel clusters: " << supervoxel_clusters.size();

    pcl::LCCPSegmentation<pcl::PointXYZRGBA> lccp;
    lccp.reset();
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSanityCheck(use_sanity_criterion);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor(k_factor);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();
    SuperVoxelAdjacencyList sv_adjacency_list;
    lccp.getSVAdjacencyList(sv_adjacency_list); // Needed for visualization

    // lccp.getSegmentAdjacencyMap(supervoxel_adjacency);

    // PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");

    // PCL_INFO ("Get labeled cloud done\n");

    lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    // PCL_INFO ("makeShared\n");

    lccp.relabelCloud(*lccp_labeled_cloud);

    yInfo() << "supervoxel clusters: " << supervoxel_clusters.size();

    return true;

    // PCL_INFO ("relabel\n");
}

void rgb_color_code(int rgb[])
{
    int i;
    for (i = 0; i < 3; i++)
    {
        rgb[i] = rand() % 256;
    }
}
void GetGraspingPoses::getPixelCoordinates(const pcl::PointXYZ &p, int &xpixel, int &ypixel)
{
    // yInfo() << "p: " << p.x << " " << p.y << " " << p.z;
    // yInfo() << "intrinsics.focalLengthX:" << intrinsics.focalLengthX << " intrinsics.principalPointX: " << intrinsics.principalPointX;
    // yInfo() << "intrinsics.focalLengthY:" << intrinsics.focalLengthY << " intrinsics.principalPointY: " << intrinsics.principalPointY;

    xpixel = -p.y / p.x * intrinsics.focalLengthX + intrinsics.principalPointX;
    ypixel = -p.z / p.x * intrinsics.focalLengthY + intrinsics.principalPointY;
    // yInfo() << "xpixel: " << xpixel << "ypixel: " << ypixel;
    if (xpixel > m_width)
        xpixel = m_width;
    if (ypixel > m_height)
        ypixel = m_height;
    if (xpixel < 0)
        xpixel = 0;
    if (ypixel < 0)
        ypixel = 0;
}

bool GetGraspingPoses::computeIntersectionOverUnion(std::array<int, 4> detection_bbox, std::array<int, 4> cluster_bbox, float &IoU)
{

    // First we need to know if the two bboxes overlap
    // yInfo()<<"detection_bbox: "<<detection_bbox[0]<<" "<<detection_bbox[1]<<" "<<detection_bbox[2]<<" "<<detection_bbox[3];
    // yInfo()<<"cluster_bbox: "<<cluster_bbox[0]<<" "<<cluster_bbox[1]<<" "<<cluster_bbox[2]<<" "<<cluster_bbox[3];

    if (detection_bbox[0] < cluster_bbox[2] && detection_bbox[2] > cluster_bbox[0] &&
        detection_bbox[1] < cluster_bbox[3] && detection_bbox[3] > cluster_bbox[1])
    {
        // if(detection_bbox[0] == detection_bbox[2] || detection_bbox[1] == detection_bbox[3] || cluster_bbox[0] == cluster_bbox[2] || cluster_bbox[1] == cluster_bbox[3])
        // {
        //     //One of the bounding boxes has area 0
        //     IoU = 0;
        //     yInfo()<<"One of the bounding boxes has area 0";
        //     return false;
        // }
        // if(detection_bbox[0] >= cluster_bbox[2] ||  cluster_bbox[0]>= detection_bbox[0]){
        //     //
        //     IoU = 0;
        //     yInfo()<<"One bbox is on the left side of the other";

        //     return false;
        // }
        // if(detection_bbox[1] >= cluster_bbox[3] ||  cluster_bbox[1]>= detection_bbox[3]){
        //     // One bbox is on the above of the other
        //     yInfo()<<"One bbox is on the above of the other";
        //     IoU = 0;
        //     return false;
        // }

        float xA = detection_bbox[0];
        if (cluster_bbox[0] > xA)
        {
            xA = cluster_bbox[0];
        }
        float yA = detection_bbox[1];
        if (cluster_bbox[1] > yA)
        {
            yA = cluster_bbox[1];
        }
        float xB = detection_bbox[2];
        if (cluster_bbox[2] < xB)
        {
            xB = cluster_bbox[2];
        }
        float yB = detection_bbox[3];
        if (cluster_bbox[3] < yB)
        {
            yB = cluster_bbox[3];
        }
        float inter_area = (xB - xA) * (yB - yA);
        // yInfo()<<"Inter area: "<<inter_area;

        float detection_bbox_area = (detection_bbox[2] - detection_bbox[0]) * (detection_bbox[3] - detection_bbox[1]);
        // yInfo()<<"detection bbox area: "<<detection_bbox_area;
        float cluster_bbox_area = (cluster_bbox[2] - cluster_bbox[0]) * (cluster_bbox[3] - cluster_bbox[1]);
        // yInfo()<<"cluster bbox area: "<<cluster_bbox_area;

        IoU = inter_area / (detection_bbox_area + cluster_bbox_area - inter_area);
        yInfo() << "IoU: " << IoU;

        return true;
    }
    else
    {
        IoU = 0;
        return false;
    }
}

void GetGraspingPoses::fullObjectPointCloud(pcl::PointCloud<pcl::PointXYZRGBA> &object_point_cloud, const std::string &category,
                                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud, int rgb[3])
{
    yInfo() << "Category fullObjectPointCloud: " << category;
    if ((category.find(std::string("cereals")) != std::string::npos) || (category.find(std::string("milk")) != std::string::npos) || (category.find(std::string("sugar")) != std::string::npos))
    {

        // yInfo()<<"PointCloud corresponds to cereal!";
        std::vector<std::vector<double>> graspingPoses;

        std::vector<pcl::PointXYZRGBA> maxPoints;
        std::vector<pcl::PointXYZRGBA> minPoints;
        std::vector<KDL::Vector> normals;
        std::vector<pcl::PointXYZRGBA> centroids;
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(2000);
        seg.setDistanceThreshold(0.012);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

        int i = 0, nr_points = (int)object_point_cloud.size();
        pcl::PCDWriter writer;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr copy_object_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);
        copyPointCloud(object_point_cloud, *copy_object_cloud);

        while (copy_object_cloud->size() > 0.05 * nr_points)
        {
            // printf("Plane %d\n", i);
            // Segment the largest planar component from the remaining cloud
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

            seg.setInputCloud(copy_object_cloud);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                // printf("Could not estimate a planar model for the given dataset.\n");
                break;
            }

            // printf("Model coefficients: %f %f %f %f\n",coefficients->values[0],coefficients->values[1],coefficients->values[2], coefficients->values[3]);
            double lengthNormal = sqrt(coefficients->values[0] * coefficients->values[0] + coefficients->values[1] * coefficients->values[1] + coefficients->values[2] * coefficients->values[2]);
            KDL::Vector n(coefficients->values[0] / lengthNormal, coefficients->values[1] / lengthNormal, coefficients->values[2] / lengthNormal);

            printf("Normal vector to plane: %f %f %f\n", n[0], n[1], n[2]);

            // Extract the inliers
            extract.setInputCloud(copy_object_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_p);

            // // aqui
            // yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloudPlane;
            // yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(*cloud_p, yarpCloudPlane);
            // rosComputeAndSendPc(yarpCloudPlane, "waist", *pointCloudLccpTopic);

            // yarp::os::Time::delay(5.0);

            // printf("PointCloud representing the planar component: %d data points\n",cloud_p->width * cloud_p->height);

            pcl::CentroidPoint<pcl::PointXYZRGBA> centroid;

            for (auto &point : *cloud_p)
            {
                centroid.add(point);
            }

            pcl::PointXYZRGBA c1;
            centroid.get(c1);
            // printf("Centroid %f %f %f\n", c1.x, c1.y, c1.z);
            std::stringstream ss;
            // ss << "plane_scene_" << i << ".pcd";
            // printf("plane_scene_%d\n", i);

            // writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_p, false);

            // printf("Valid plane i: %d\n",i);
            pcl::PointXYZRGBA maxPoint;
            pcl::PointXYZRGBA minPoint;

            getMinimumBoundingBoxPointCLoud(cloud_p, maxPoint, minPoint, n);

            // printf("Normal: %f %f %f\n", n[0], n[1], n[2]);
            printf("Max point: %f %f %f, Min Point: %f %f %f\n", maxPoint.x, maxPoint.y, maxPoint.z, minPoint.x, minPoint.y, minPoint.z);

            normals.push_back(n);
            maxPoints.push_back(maxPoint);
            minPoints.push_back(minPoint);
            centroids.push_back(c1);

            // Create the filtering object
            extract.setNegative(true);
            extract.filter(*cloud_f);
            (copy_object_cloud).swap(cloud_f);
            i++;
        }
        if (normals.size() == 0)
        {
            printf("No planes detected!\n");
        }
        else
        {
            // printf("One or more Planes are detected!\n");
            m_bObject.clear();
            m_bObject.addString(category);
            if (category == "cereals1" || category == "cereals2" || category == "cereals3")
            {
                completeBox(normals, centroids, maxPoints, minPoints, filling_cloud, rgb, cerealBoxShape);
                // computeGraspingPosesCereal(normals, centroids, maxPoints, minPoints, graspingPoses);
                // rosComputeGraspingPosesArrowAndSend("waist", centroids, normals);
            }
            if (category == "milk1" || category == "milk2")
            {
                completeBox(normals, centroids, maxPoints, minPoints, filling_cloud, rgb, milkBoxShape);
                // computeGraspingPosesCereal(normals, centroids, maxPoints, minPoints, graspingPoses);
                // rosComputeGraspingPosesArrowAndSend("waist", centroids, normals);
            }
            // if (category == "sugar1")
            // {
            //     completeBox(normals, centroids, maxPoints, minPoints, filling_cloud, rgb, sugar1Shape);
            // }
            // if (category == "sugar2")
            // {
            //     completeBox(normals, centroids, maxPoints, minPoints, filling_cloud, rgb, sugar2Shape);
            // }

            // if (category == "sugar3")
            // {
            //     completeBox(normals, centroids, maxPoints, minPoints, filling_cloud, rgb, sugar3Shape);
            // }
            m_bGraspingPoses.addList() = m_bObject;
        }
    }
}

void GetGraspingPoses::updateDetectedObjectsPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud)
{
    detected_objects.clear();
    for (int i = 0; i < lccp_labeled_cloud->points.size(); ++i)
    {

        uint32_t idx = lccp_labeled_cloud->points[i].label;

        // in this way we enlarges the vector everytime we encounter a greater label. So we don't need to pass all
        //  labeeld point cloud to see what is the greater label, and then to resize the vector.
        if (idx >= detected_objects.size()) // keep in mind that there is also the label 0!
        {
            detected_objects.resize(idx + 1);
        }
        // if (detected_objects[idx].object_cloud.empty())
        // {
        //     detected_objects[idx].r = rand() % 256;
        //     detected_objects[idx].g = rand() % 256;
        //     detected_objects[idx].b = rand() % 256;
        // }
        pcl::PointXYZRGBA tmp_point_rgb;
        tmp_point_rgb.x = lccp_labeled_cloud->points[i].x;
        tmp_point_rgb.y = lccp_labeled_cloud->points[i].y;
        tmp_point_rgb.z = lccp_labeled_cloud->points[i].z;
        tmp_point_rgb.r = rand() % 256;
        tmp_point_rgb.g = rand() % 256;
        tmp_point_rgb.b = rand() % 256;

        detected_objects[idx].object_cloud.points.push_back(tmp_point_rgb);
        detected_objects[idx].label = (int)idx;
    }

    // remove segments with too few points
    // it will removes te ones with few points or the ones with no points (these are created because of the labels of lccp)
    int size = detected_objects.size();
    int i = 0;
    while (i < size)
    {
        if (detected_objects[i].object_cloud.size() < this->th_points)
        {
            detected_objects.erase(detected_objects.begin() + i);
            size = detected_objects.size();
        }
        else
        {
            detected_objects[i].object_cloud.width = detected_objects[i].object_cloud.size();
            detected_objects[i].object_cloud.height = 1;
            i++;
        }
    }
}

bool GetGraspingPoses::PclPointCloudToSuperqPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &object_cloud, SuperqModel::PointCloud &point_cloud)
{
    std::vector<yarp::sig::Vector> acquired_points;
    std::vector<std::vector<unsigned char>> acquired_colors;

    yarp::sig::Vector point(3);
    std::vector<unsigned char> c;
    c.resize(3);

    for (size_t idx = 0; idx < object_cloud.size(); idx++)
    {
        point(0) = object_cloud[idx].x;
        point(1) = object_cloud[idx].y;
        point(2) = object_cloud[idx].z;

        c[0] = object_cloud[idx].r;
        c[1] = object_cloud[idx].g;
        c[2] = object_cloud[idx].b;

        acquired_points.push_back(point);
        acquired_colors.push_back(c);
    }

    std::deque<Eigen::Vector3d> eigen_points;

    for (size_t i = 0; i < acquired_points.size(); i++)
    {
        eigen_points.push_back(yarp::eigen::toEigen(acquired_points[i]));
    }

    if (eigen_points.size() >= sq_model_params["minimum_points"])
    {
        point_cloud.setPoints(eigen_points);
        point_cloud.setColors(acquired_colors);
        return true;
    }
    else
        return false;
}

void GetGraspingPoses::GetSuperquadricFromPointCloud(SuperqModel::PointCloud point_cloud, std::vector<SuperqModel::Superquadric> &superqs)
{

    /*  ------------------------------  */
    /*  ------> Compute superq <------  */
    /*  ------------------------------  */

    yInfo() << "[computeSuperqAndGrasp]: compute superq";
    if (single_superq || object_class != "default")
    {
        estim.SetStringValue("object_class", object_class);
        superqs = estim.computeSuperq(point_cloud);
    }
}

void GetGraspingPoses::computeGraspingPoses(const std::vector<SuperqModel::Superquadric> &superqs, std::vector<std::vector<double>> &graspingPoses)
{
    auto params = superqs[0].getSuperqParams();

    Eigen::AngleAxisf rollAngle(params[8], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(params[9], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(params[10], Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;

    KDL::Rotation rot_aux = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
    KDL::Frame frame_object_wrt_world(rot_aux);
    frame_object_wrt_world.p[0] = params[5];
    frame_object_wrt_world.p[1] = params[6];
    frame_object_wrt_world.p[2] = params[7];

    KDL::Frame frame_grasping_wrt_world;
    std::cout << "axes length: " << 2 * params[0] << " " << 2 * params[1] << ": " << 2 * params[2] << std::endl;

    float step = 0.01;
    if (2 * params[2] <= MAX_OBJECT_WIDTH_GRASP)
    {
        KDL::Vector zobject;
        KDL::Vector yobject;
        KDL::Vector xobject;
        KDL::Rotation rot;
        KDL::Frame frameTCP, frame_grasping_wrt_object;
        std::vector<double> tcpX;
        for (float xaxes = -1.0; xaxes <= 1.0; xaxes += 2)
        {
            zobject = KDL::Vector(xaxes, 0, 0);
            for (float y = -params[1] / 2.0; y <= params[1] / 2.0; y += step)
            {
                for (float yaxes = -1.0; yaxes <= 1.0; yaxes += 2)
                {
                    yobject = KDL::Vector(0, yaxes, 0.0);
                    xobject = yobject * zobject;
                    rot = KDL::Rotation(xobject, yobject, zobject);
                    frameTCP = KDL::Frame(rot);
                    frame_grasping_wrt_object = frameTCP;
                    frame_grasping_wrt_object.p[0] = -xaxes * params[0];
                    frame_grasping_wrt_object.p[1] = y;
                    frame_grasping_wrt_object.p[2] = 0;
                    frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
                    // Check if is trying to grasp from the bottom of the objet
                    KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
                    KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
                    KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

                    KDL::Vector axesz(0, 0, 1);
                    float angle = atan2((unitz * axesz).Norm(), dot(unitz, axesz));
                    // printf("angle: %f\n", angle * M_1_PI / 180.0);
                    if (abs(angle) > 10 * M_1_PI / 180.0)
                    {
                        tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_grasping_wrt_world);
                        // printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
                        graspingPoses.push_back(tcpX);
                    }
                }
            }
        }

        for (float yaxes = -1.0; yaxes <= 1.0; yaxes += 2)
        {
            zobject = KDL::Vector(0, yaxes, 0);
            for (float x = -params[0] / 2.0; x <= params[0] / 2.0; x += step)
            {
                for (float xaxes = -1.0; xaxes <= 1.0; xaxes += 2)
                {
                    yobject = KDL::Vector(xaxes, 0.0, 0.0);
                    xobject = yobject * zobject;
                    rot = KDL::Rotation(xobject, yobject, zobject);
                    frameTCP = KDL::Frame(rot);
                    frame_grasping_wrt_object = frameTCP;
                    frame_grasping_wrt_object.p[0] = x;
                    frame_grasping_wrt_object.p[1] = -yaxes * params[1];
                    frame_grasping_wrt_object.p[2] = 0;
                    frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
                    // Check if is trying to grasp from the bottom of the objet
                    KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
                    KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
                    KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

                    KDL::Vector axesz(0, 0, 1);
                    float angle = atan2((unitz * axesz).Norm(), dot(unitz, axesz));
                    // printf("angle: %f\n", angle * M_1_PI / 180.0);

                    if (abs(angle) > 10 * M_1_PI / 180.0)
                    {
                        tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_grasping_wrt_world);
                        // printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
                        graspingPoses.push_back(tcpX);
                    }
                }
            }
        }
    }

    if (2 * params[1] <= MAX_OBJECT_WIDTH_GRASP)
    {
        printf("Aquiiiiiiiii\n");
        KDL::Vector zobject;
        KDL::Vector yobject;
        KDL::Vector xobject;
        KDL::Rotation rot;
        KDL::Frame frameTCP, frame_grasping_wrt_object;
        std::vector<double> tcpX;
        // for (float xaxes = -1.0; xaxes <= 1.0; xaxes += 2)
        // {
        //     zobject = KDL::Vector(xaxes, 0, 0);
        //     for (float y = -params[2] / 2.0; y <= params[2] / 2.0; y += step)
        //     {
        //         for (float yaxes = -1.0; yaxes <= 1.0; yaxes += 2)
        //         {
        //             yobject = KDL::Vector(0, yaxes, 0.0);
        //             xobject = yobject * zobject;
        //             rot = KDL::Rotation(xobject, yobject, zobject);
        //             frameTCP = KDL::Frame(rot);
        //             frame_grasping_wrt_object = frameTCP;
        //             frame_grasping_wrt_object.p[0] = 0;
        //             frame_grasping_wrt_object.p[1] = y;
        //             frame_grasping_wrt_object.p[2] = -xaxes * params[0];
        //             frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
        //             // Check if is trying to grasp from the bottom of the objet
        //             KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
        //             KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
        //             KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

        //             KDL::Vector axesz(0,0,1);
        //             float angle = atan2((unitz*axesz).Norm(), dot(unitz,axesz));
        //             printf("angle: %f\n", angle*M_1_PI/180.0);
        //             if(abs(angle)>10*M_1_PI/180.0){
        //                 tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_grasping_wrt_world);
        //                 printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
        //                 graspingPoses.push_back(tcpX);
        //             }
        //         }
        //     }
        // }

        for (float yaxes = -1.0; yaxes <= 1.0; yaxes += 2)
        {
            zobject = KDL::Vector(0, 0, yaxes);
            for (float x = -params[0] / 2.0; x <= params[0] / 2.0; x += step)
            {
                for (float xaxes = -1.0; xaxes <= 1.0; xaxes += 2)
                {
                    yobject = KDL::Vector(xaxes, 0.0, 0.0);
                    xobject = yobject * zobject;
                    rot = KDL::Rotation(xobject, yobject, zobject);
                    frameTCP = KDL::Frame(rot);
                    frame_grasping_wrt_object = frameTCP;
                    frame_grasping_wrt_object.p[0] = x;
                    frame_grasping_wrt_object.p[1] = 0;
                    frame_grasping_wrt_object.p[2] = -yaxes * params[2];
                    frame_grasping_wrt_world = frame_object_wrt_world * frame_grasping_wrt_object;
                    // Check if is trying to grasp from the bottom of the objet
                    KDL::Vector unitx = frame_grasping_wrt_world.M.UnitX();
                    KDL::Vector unity = frame_grasping_wrt_world.M.UnitY();
                    KDL::Vector unitz = frame_grasping_wrt_world.M.UnitZ();

                    KDL::Vector axesz(0, 0, 1);
                    float angle = atan2((unitz * axesz).Norm(), dot(unitz, axesz));
                    // printf("angle: %f\n", angle * M_1_PI / 180.0);

                    if (abs(angle) > 10 * M_1_PI / 180.0)
                    {
                        tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_grasping_wrt_world);
                        // printf("%f %f %f %f %f %f\n", tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
                        graspingPoses.push_back(tcpX);
                    }
                }
            }
        }
    }

}

bool GetGraspingPoses::createBoundingBox2DFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs, std::array<int, 4> &bbox)
{
    auto params = superqs[0].getSuperqParams();

    // Bbox 3d wrt object's frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox3d(new pcl::PointCloud<pcl::PointXYZ>);
    for (float x = -1; x <= 1; x += 2)
    {
        for (float y = -1; y <= 1; y += 2)
        {
            for (float z = -1; z <= 1; z += 2)
            {
                pcl::PointXYZ p;

                p.x = x * params[0];
                p.y = y * params[1];
                p.z = z * params[2];

                bbox3d->push_back(*(pcl::PointXYZ *)(&p));
            }
        }
    }

    // Bbox 3d wrt world frame
    Eigen::AngleAxisf rollAngle(params[8], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(params[9], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(params[10], Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();

    Eigen::Vector3f v(params[5], params[6], params[7]);
    Eigen::Matrix4f transform = Eigen::Affine3f(Eigen::Translation3f(v)).matrix();

    transform.block<3, 3>(0, 0) = rotationMatrix;
    pcl::transformPointCloud(*bbox3d, *bbox3d, transform);

    yarp::rosmsg::visualization_msgs::Marker marker;
    yarp::rosmsg::visualization_msgs::MarkerArray markerArray;

    marker.header.frame_id = "waist";
    static int counter = 0;
    marker.header.stamp.nsec = 0;
    marker.header.stamp.sec = 0;

    // marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
    // markerArray.markers.push_back(marker);

    // if (bbox3d_topic)
    // {
    //     yInfo("Publish...\n");
    //     bbox3d_topic->write(markerArray);
    // }

    marker.type = yarp::rosmsg::visualization_msgs::Marker::LINE_LIST;
    marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
    marker.header.frame_id = "waist";
    marker.header.stamp.nsec = 0;
    marker.header.stamp.sec = 0;
    marker.header.seq = 0;

    yInfo() << "Bbox has " << bbox3d->points.size() << "points";
    for (int idx = 0; idx <= bbox3d->points.size() - 1; idx += 1)
    {
        if (idx % 2 == 0)
        {
            // marker.id = idx;
            yInfo() << "Line " << idx << " and " << idx + 1;
            yarp::rosmsg::geometry_msgs::Point pointRos;
            pointRos.x = bbox3d->points[idx].x;
            pointRos.y = bbox3d->points[idx].y;
            pointRos.z = bbox3d->points[idx].z;
            marker.points.push_back(pointRos);

            pointRos.x = bbox3d->points[idx + 1].x;
            pointRos.y = bbox3d->points[idx + 1].y;
            pointRos.z = bbox3d->points[idx + 1].z;
            marker.points.push_back(pointRos);

            marker.pose.orientation.w = 1.0;
        }
        if (idx <= 1 || ((idx >= 4) && (idx < 6)))
        {
            yarp::rosmsg::geometry_msgs::Point pointRos;
            pointRos.x = bbox3d->points[idx].x;
            pointRos.y = bbox3d->points[idx].y;
            pointRos.z = bbox3d->points[idx].z;
            marker.points.push_back(pointRos);

            pointRos.x = bbox3d->points[idx + 2].x;
            pointRos.y = bbox3d->points[idx + 2].y;
            pointRos.z = bbox3d->points[idx + 2].z;
            marker.points.push_back(pointRos);

            marker.pose.orientation.w = 1.0;
        }
        if (idx <= 3)
        {
            yarp::rosmsg::geometry_msgs::Point pointRos;

            pointRos.x = bbox3d->points[idx].x;
            pointRos.y = bbox3d->points[idx].y;
            pointRos.z = bbox3d->points[idx].z;
            marker.points.push_back(pointRos);

            pointRos.x = bbox3d->points[idx + 4].x;
            pointRos.y = bbox3d->points[idx + 4].y;
            pointRos.z = bbox3d->points[idx + 4].z;
            marker.points.push_back(pointRos);

            marker.pose.orientation.w = 1.0;
        }
    }
    marker.scale.x = 0.005;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    markerArray.markers.push_back(marker);

    if (bbox3d_topic)
    {
        yInfo("Publish...\n");
        bbox3d_topic->write(markerArray);
    }

    Eigen::Matrix4f transform_world_to_camera;
    getTransformMatrix(false, transform_world_to_camera);

    pcl::transformPointCloud(*bbox3d, *bbox3d, transform_world_to_camera);

    int tlx1 = m_width, tly1 = m_height, brx1 = 0, bry1 = 0;
    for (size_t idx = 0; idx < bbox3d->size(); idx++)
    {
        pcl::PointXYZ p(bbox3d->points[idx].x, bbox3d->points[idx].y, bbox3d->points[idx].z);
        // yInfo()<<"p: "<<p.x<<" "<<p.y<<" "<<p.z;
        int xpixel, ypixel;
        // yInfo()<<"xpixel: "<<xpixel<<" ypixel: "<<ypixel;
        getPixelCoordinates(p, xpixel, ypixel);
        if (xpixel < tlx1)
        {
            tlx1 = xpixel;
        }
        if (xpixel > brx1)
        {
            brx1 = xpixel;
        }
        if (ypixel < tly1)
        {
            tly1 = ypixel;
        }
        if (ypixel > bry1)
        {
            bry1 = ypixel;
        }
    }
    bbox[0] = tlx1;
    bbox[1] = tly1;
    bbox[2] = brx1;
    bbox[3] = bry1;

    return true;
}

void GetGraspingPoses::createPointCloudFromSuperquadric(const std::vector<SuperqModel::Superquadric> &superqs, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudSuperquadric, int indexDetectedObjects)
{
    double step = 0.005;
    auto params = superqs[0].getSuperqParams();

    std::cout << "a0: " << params[0] << " a1: " << params[1] << " a2: " << params[3] << std::endl;

    for (double x = 0; x <= params[0]; x += step)
    {
        for (double y = 0; y <= params[1]; y += step)
        {
            for (double z = 0; z <= params[2]; z += step)
            {
                double f = pow(abs(pow(abs(x / params[0]), 2.0 / params[4]) + pow(abs(y / params[1]), 2.0 / params[4])), params[4] / params[3]) + pow(abs(z / params[2]), 2.0 / params[3]);
                if (f <= 1.0)
                {
                    pcl::PointXYZRGBA p;

                    p.x = x;
                    p.y = y;
                    p.z = z;
                    p.r = detected_objects[indexDetectedObjects].object_cloud.points[0].r;
                    p.g = detected_objects[indexDetectedObjects].object_cloud.points[0].g;
                    p.b = detected_objects[indexDetectedObjects].object_cloud.points[0].b;
                    p.a = 1;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                    p.z = -z;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                    p.x = -x;
                    p.y = y;
                    p.z = z;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));
                    p.z = -z;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                    p.x = x;
                    p.y = -y;
                    p.z = z;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));
                    p.z = -z;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                    p.x = -x;
                    p.y = -y;
                    p.z = z;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));

                    p.z = -z;
                    cloudSuperquadric->push_back(*(pcl::PointXYZRGBA *)(&p));
                }
            }
        }
    }

    Eigen::AngleAxisf rollAngle(params[8], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(params[9], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(params[10], Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();

    Eigen::Vector3f v(params[5], params[6], params[7]);
    Eigen::Matrix4f transform = Eigen::Affine3f(Eigen::Translation3f(v)).matrix();

    transform.block<3, 3>(0, 0) = rotationMatrix;
    pcl::transformPointCloud(*cloudSuperquadric, *cloudSuperquadric, transform);
}

void GetGraspingPoses::XYZLPointCloudToRGBAPointCloud(const pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud, const yarp::os::Bottle &bottle_detections,
                                                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &lccp_colored_cloud,
                                                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud)
{

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_object_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_bboxes(new pcl::PointCloud<pcl::PointXYZRGBA>);

    updateDetectedObjectsPointCloud(lccp_labeled_cloud);

    Eigen::Matrix4f transform;
    getTransformMatrix(false, transform);
    yarp::os::Bottle label_category;
    for (int i = 0; i < detected_objects.size(); i++)
    {
        // yInfo()<<"Cloud: "<<i;
        std::stringstream ss;
        ss << "object_" << i << ".pcd";
        printf("object_%d\n", i);
        pcl::PCDWriter writerPCD;
        writerPCD.write<pcl::PointXYZRGBA>(ss.str(), detected_objects[i].object_cloud, false);

        yarp::os::Bottle b;
        Eigen::Matrix4f transform_now;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::transformPointCloud(detected_objects[i].object_cloud, *aux_cloud, transform);
        *transformed_object_cloud += detected_objects[i].object_cloud;

        // pcl::PointXYZRGBA maxPoint, minPoint;
        // getMinimumBoundingBoxPointCLoud(aux_cloud, maxPoint, minPoint);
        // yInfo() << "MaxPoint: " << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z;
        // yInfo() << "MinPoint: " << minPoint.x << " " << minPoint.y << " " << minPoint.z;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr bbox(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // getMinimumOrientedBoundingBox(aux_cloud, bbox);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aux_bbox(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // pcl::PointXYZ maxPointCloud(-1000, -1000, -1000), minPointCloud(1000, 1000, 1000);
        // for (size_t idx = 0; idx < aux_cloud->size(); idx++)
        // {
        //     pcl::PointXYZ p(aux_cloud->points[idx].x, aux_cloud->points[idx].y, aux_cloud->points[idx].z);
        //     if (p.x > maxPointCloud.x)
        //         maxPointCloud.x = p.x;
        //     if (p.y > maxPointCloud.y)
        //         maxPointCloud.y = p.y;
        //     if (p.z > maxPointCloud.z)
        //         maxPointCloud.z = p.z;
        //     if (p.x < minPointCloud.x)
        //         minPointCloud.x = p.x;
        //     if (p.y < minPointCloud.y)
        //         minPointCloud.y = p.y;
        //     if (p.z < minPointCloud.z)
        //         minPointCloud.z = p.z;
        //     // yInfo()<<"p: "<<p.x<<" "<<p.y<<" "<<p.z;
        // }
        pcl::PointXYZRGBA maxPoint;
        pcl::PointXYZRGBA minPoint;
        pcl::getMinMax3D(*aux_cloud, minPoint, maxPoint);

        // yInfo() << "maxPointCloud: " << maxPointCloud.x << " " << maxPointCloud.y << " " << maxPointCloud.z;
        // yInfo() << "minPointCloud: " << minPointCloud.x << " " << minPointCloud.y << " " << minPointCloud.z;
        bbox->clear();
        pcl::PointXYZRGBA auxp;
        auxp.x = maxPoint.x;
        auxp.y = maxPoint.y;
        auxp.z = maxPoint.z;
        auxp.r = 255;
        bbox->points.push_back(auxp);
        auxp.y = minPoint.y;
        bbox->points.push_back(auxp);
        auxp.z = minPoint.z;
        bbox->points.push_back(auxp);
        auxp.y = maxPoint.y;
        bbox->points.push_back(auxp);

        auxp.x = minPoint.x;
        bbox->points.push_back(auxp);
        auxp.y = minPoint.y;
        bbox->points.push_back(auxp);
        auxp.z = minPoint.z;
        bbox->points.push_back(auxp);
        auxp.y = maxPoint.y;
        bbox->points.push_back(auxp);

        // pcl::transformPointCloud(*bbox, *bbox, transform.inverse());

        *transformed_bboxes += *bbox;
        yInfo() << "transformed bboxes done!";

        // yInfo()<<"Cloud: "<<i;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::transformPointCloud(detected_objects[i].object_cloud, *transformed_source_cloud, transform);
        // int tlx = m_width, tly = m_height, brx = 0, bry = 0;
        // for (size_t idx = 0; idx < transformed_source_cloud->size(); idx++)
        // {
        //     pcl::PointXYZ p(transformed_source_cloud->points[idx].x, transformed_source_cloud->points[idx].y, transformed_source_cloud->points[idx].z);
        //     // yInfo()<<"p: "<<p.x<<" "<<p.y<<" "<<p.z;
        //     int xpixel, ypixel;
        //     // yInfo()<<"xpixel: "<<xpixel<<" ypixel: "<<ypixel;
        //     getPixelCoordinates(p, xpixel, ypixel);
        //     if (xpixel < tlx)
        //     {
        //         tlx = xpixel;
        //     }
        //     if (xpixel > brx)
        //     {
        //         brx = xpixel;
        //     }
        //     if (ypixel < tly)
        //     {
        //         tly = ypixel;
        //     }
        //     if (ypixel > bry)
        //     {
        //         bry = ypixel;
        //     }
        // }
        // yInfo() << "tlx: " << tlx << " tly: " << tly << " brx: " << brx << " bry: " << bry;

        int tlx1 = m_width, tly1 = m_height, brx1 = 0, bry1 = 0;
        for (size_t idx = 0; idx < bbox->size(); idx++)
        {
            pcl::PointXYZ p(bbox->points[idx].x, bbox->points[idx].y, bbox->points[idx].z);
            // yInfo()<<"p: "<<p.x<<" "<<p.y<<" "<<p.z;
            int xpixel, ypixel;
            // yInfo()<<"xpixel: "<<xpixel<<" ypixel: "<<ypixel;
            getPixelCoordinates(p, xpixel, ypixel);
            if (xpixel < tlx1)
            {
                tlx1 = xpixel;
            }
            if (xpixel > brx1)
            {
                brx1 = xpixel;
            }
            if (ypixel < tly1)
            {
                tly1 = ypixel;
            }
            if (ypixel > bry1)
            {
                bry1 = ypixel;
            }
        }
        yInfo() << "tlx1: " << tlx1 << " tly1: " << tly1 << " brx1: " << brx1 << " bry1: " << bry1;

        std::array<int, 4> cluster_bbox = {tlx1, tly1, brx1, bry1};
        float iou = 0;
        float best_iou = 0;
        std::string current_category;
        for (unsigned int i = 0; i < bottle_detections.size(); i++)
        {
            yarp::os::Bottle b;
            int brx_detection = bottle_detections.get(i).find("brx").asInt32();
            if (brx_detection > m_width)
                brx_detection = m_width;

            int bry_detection = bottle_detections.get(i).find("bry").asInt32();
            if (bry_detection > m_height)
                bry_detection = m_height;

            int tlx_detection = bottle_detections.get(i).find("tlx").asInt32();
            if (tlx_detection < 0)
                tlx_detection = 0;
            int tly_detection = bottle_detections.get(i).find("tly").asInt32();
            if (tly_detection < 0)
                tly_detection = 0;
            std::string category_detection = bottle_detections.get(i).find("category").asString();
            yInfo() << "category: " << category_detection << " brx:" << brx_detection << " bry: " << bry_detection << " tlx: " << tlx_detection << " tly: " << tly_detection;
            std::array<int, 4> detection_bbox = {tlx_detection, tly_detection, brx_detection, bry_detection};
            computeIntersectionOverUnion(detection_bbox, cluster_bbox, iou);
            if (iou > best_iou)
            {
                current_category = category_detection;
                best_iou = iou;
            }
        }
        // Check what we have in the bottle
        bool add_new = true;
        for (unsigned int i = 0; i < label_category.size(); i++)
        {
            yarp::os::Bottle *b = label_category.get(i).asList();
            if (b->get(0).find("category").asString() == current_category)
            {
                if (b->get(0).find("iou").asFloat64() <= best_iou)
                {
                    b->clear();
                    break;
                }
                else
                {
                    add_new = false;
                    break;
                }
            }
        }
        if (best_iou == 0)
            add_new = false;
        if (add_new)
        {
            int label = i;
            b.addDict() = {
                {"category", yarp::os::Value(current_category)},
                {"iou", yarp::os::Value(best_iou)},
                {"label", yarp::os::Value(label)},
            };
            label_category.addList() = b;
        }
    }

    yInfo() << label_category.toString();

    for (unsigned int i = 0; i < label_category.size(); i++)
    {
        yarp::os::Bottle *b = label_category.get(i).asList();
        std::string category = b->get(0).find("category").asString();
        bool already_seen = false;
        int rgb[3] = {0, 0, 0};
        for (unsigned int j = 0; j < m_category_rgb.size(); j++)
        {
            yarp::os::Bottle *b_rgb = m_category_rgb.get(j).asList();
            if (b_rgb->get(0).find("category").asString() == b->get(0).find("category").asString())
            {
                already_seen = true;
                rgb[0] = b_rgb->get(0).find("r").asInt32();
                rgb[1] = b_rgb->get(0).find("g").asInt32();
                rgb[2] = b_rgb->get(0).find("b").asInt32();
                break;
            }
        }
        if (!already_seen)
        {
            yarp::os::Bottle b_rgb;
            rgb[0] = rand() % 256;
            rgb[1] = rand() % 256;
            rgb[2] = rand() % 256;
            b_rgb.addDict() = {
                {"category", yarp::os::Value(b->get(0).find("category"))},
                {"r", yarp::os::Value(rgb[0])},
                {"g", yarp::os::Value(rgb[1])},
                {"b", yarp::os::Value(rgb[2])},
            };
            m_category_rgb.addList() = b_rgb;
        }
        int label_cloud = b->get(0).find("label").asInt32();
        // std::map<int, int>::iterator it = map_labels.find(label_cloud);
        // if(it != map_labels.end()){
        pcl::PointXYZRGBA point;
        point.r = rgb[0];
        point.g = rgb[1];
        point.b = rgb[2];
        point.a = 0;
        for (size_t pidx = 0; pidx < detected_objects[i].object_cloud.points.size(); pidx++)
        {
            point.x = detected_objects[i].object_cloud.points[pidx].x;
            point.y = detected_objects[i].object_cloud.points[pidx].y;
            point.z = detected_objects[i].object_cloud.points[pidx].z;
            lccp_colored_cloud->push_back(*(pcl::PointXYZRGBA *)(&point));
        }
        fullObjectPointCloud(detected_objects[i].object_cloud, category, filling_cloud, rgb);

        // }
    }
    // yInfo() << m_bGraspingPoses.toString();

    // for (unsigned int i = 0; i < label_category.size(); i++)
    // {
    //     yarp::os::Bottle *b = label_category.get(i).asList();
    //     std::string category = b->get(0).find("category").asString();
    //     bool already_seen = false; //
    //     int rgb[3] = {0, 0, 0};
    //     for (unsigned int j = 0; j < m_category_rgb.size(); j++)
    //     {
    //         yarp::os::Bottle *b_rgb = m_category_rgb.get(j).asList();
    //         if (b_rgb->get(0).find("category").asString() == b->get(0).find("category").asString())
    //         {
    //             already_seen = true;
    //             rgb[0] = 1;
    //             rgb[1] = 0;
    //             rgb[2] = 0;
    //             break;
    //         }
    //     }
    //     if (!already_seen)
    //     {
    //         yarp::os::Bottle b_rgb;
    //         rgb[0] = rand() % 256;
    //         rgb[1] = rand() % 256;
    //         rgb[2] = rand() % 256;
    //         b_rgb.addDict() = {
    //             {"category", yarp::os::Value(b->get(0).find("category"))},
    //             {"r", yarp::os::Value(rgb[0])},
    //             {"g", yarp::os::Value(rgb[1])},
    //             {"b", yarp::os::Value(rgb[2])},
    //         };
    //         m_category_rgb.addList() = b_rgb;
    //     }
    // detected_objects[i].object_cloud.points.
    // int label_cloud = b->get(0).find("label").asInt32();
    // std::map<int, int>::iterator it = map_labels.find(label_cloud);
    // if (it != map_labels.end())
    // {

    //     pcl::PointXYZRGBA point;
    //     point.r = rgb[0];
    //     point.g = rgb[1];
    //     point.b = rgb[2];
    //     point.a = 0;
    //     for (size_t pidx = 0; pidx < sourceClouds[it->second]->points.size(); pidx++)
    //     {
    //         point.x = sourceClouds[it->second]->points[pidx].x;
    //         point.y = sourceClouds[it->second]->points[pidx].y;
    //         point.z = sourceClouds[it->second]->points[pidx].z;
    //         lccp_colored_cloud->push_back(*(pcl::PointXYZRGBA *)(&point));
    //     }
    //     // fullObjectPointCloud(sourceClouds[it->second], category, filling_cloud, rgb);
    // }
    // }
    // }

    // yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> &yarpCloudFilling = outPointCloudPort.prepare();
    // yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(*transformed_object_cloud, yarpCloudFilling);

    // rosComputeAndSendPc(yarpCloudFilling, "waist", *pointCloudFillingObjectsTopic);

    // yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloudWithoutHorizontalSurface;
    // yInfo() << "Point cloud transformed";

    // yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(*transformed_bboxes, yarpCloudWithoutHorizontalSurface);
    // yInfo() << "Point cloud transformed";

    // rosComputeAndSendPc(yarpCloudWithoutHorizontalSurface, "waist", *pointCloudWithoutPlannarSurfaceTopic);
    yInfo() << "ALL SEND";
    // int current_label = -1;

    // // Lets count the number of clusters
    // int count = 1;
    // std::vector<int> number_points_for_label;

    // // lccp_labeled_cloud->points.
    // for (size_t i = 0; i < lccp_labeled_cloud->size(); i++)
    // {
    //     // yInfo()<<"lccp_labeled_cloud->points[i].label: "<<lccp_labeled_cloud->points[i].label;
    //     if (lccp_labeled_cloud->points[i].label >= count)
    //     {
    //         count = lccp_labeled_cloud->points[i].label;
    //         number_points_for_label.resize(count + 1);
    //     }
    //     number_points_for_label[lccp_labeled_cloud->points[i].label] += 1;
    // }
    // // yInfo()<<"able to get the number of points per label";

    // int min_point_per_label = 50;
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // pcl::ExtractIndices<pcl::PointXYZL> extract;
    // bool enough_points[count];
    // for (size_t i = 0; i < lccp_labeled_cloud->size(); i++)
    // {
    //     if (number_points_for_label[lccp_labeled_cloud->points[i].label] < min_point_per_label)
    //         inliers->indices.push_back(i);
    // }
    // extract.setInputCloud(lccp_labeled_cloud);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // pcl::PointCloud<pcl::PointXYZL>::Ptr only_labels_with_enough_points(new pcl::PointCloud<pcl::PointXYZL>);
    // extract.filter(*only_labels_with_enough_points);

    // count = 0;
    // std::vector<int> labels;
    // std::map<int, int> map_labels;
    // for (unsigned int i = 0; i < number_points_for_label.size(); i++)
    // {
    //     if (number_points_for_label[i] >= min_point_per_label)
    //     {
    //         map_labels.insert(std::pair<int, int>(i, count));
    //         count++;
    //     }
    // }

    // // We need to extract each cluster from the pointcloud
    // std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZL>::Ptr>> sourceClouds;
    // for (int i = 0; i < count; i++)
    // {
    //     pcl::PointCloud<pcl::PointXYZL>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZL>);
    //     sourceClouds.push_back(sourceCloud);
    // }
    // for (size_t i = 0; i < only_labels_with_enough_points->size(); i++)
    // {
    //     // yInfo()<<"lccp_labeled_cloud->points[i].label: "<<lccp_labeled_cloud->points[i].label;
    //     std::map<int, int>::iterator it = map_labels.find(only_labels_with_enough_points->points[i].label);
    //     if (it != map_labels.end())
    //         sourceClouds[it->second]->push_back(*(pcl::PointXYZL *)(&only_labels_with_enough_points->points[i]));
    // }

    // // So now we have each cluster in a point cloud in the vector of sourceClouds. We need to transform the pointcloud to camera frame
    // Eigen::Matrix4f transform;
    // getTransformMatrix(false, transform);
    // yarp::os::Bottle label_category;
    // for (int i = 0; i < sourceClouds.size(); i++)
    // {
    //     // yInfo()<<"Cloud: "<<i;
    //     yarp::os::Bottle b;
    //     pcl::PointCloud<pcl::PointXYZL>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    //     pcl::transformPointCloud(*sourceClouds[i], *transformed_source_cloud, transform);
    //     int tlx = m_width, tly = m_height, brx = 0, bry = 0;
    //     for (size_t idx = 0; idx < transformed_source_cloud->size(); idx++)
    //     {
    //         pcl::PointXYZ p(transformed_source_cloud->points[idx].x, transformed_source_cloud->points[idx].y, transformed_source_cloud->points[idx].z);
    //         // yInfo()<<"p: "<<p.x<<" "<<p.y<<" "<<p.z;
    //         int xpixel, ypixel;
    //         // yInfo()<<"xpixel: "<<xpixel<<" ypixel: "<<ypixel;
    //         getPixelCoordinates(p, xpixel, ypixel);
    //         if (xpixel < tlx)
    //         {
    //             tlx = xpixel;
    //         }
    //         if (xpixel > brx)
    //         {
    //             brx = xpixel;
    //         }
    //         if (ypixel < tly)
    //         {
    //             tly = ypixel;
    //         }
    //         if (ypixel > bry)
    //         {
    //             bry = ypixel;
    //         }
    //     }
    //     yInfo() << "tlx: " << tlx << " tly: " << tly << " brx: " << brx << " bry: " << bry;
    //     std::array<int, 4> cluster_bbox = {tlx, tly, brx, bry};
    //     float iou = 0;
    //     float best_iou = 0;
    //     std::string current_category;
    //     for (unsigned int i = 0; i < bottle_detections.size(); i++)
    //     {
    //         yarp::os::Bottle b;
    //         int brx_detection = bottle_detections.get(i).find("brx").asInt32();
    //         if (brx_detection > m_width)
    //             brx_detection = m_width;

    //         int bry_detection = bottle_detections.get(i).find("bry").asInt32();
    //         if (bry_detection > m_height)
    //             bry_detection = m_height;

    //         int tlx_detection = bottle_detections.get(i).find("tlx").asInt32();
    //         if (tlx_detection < 0)
    //             tlx_detection = 0;
    //         int tly_detection = bottle_detections.get(i).find("tly").asInt32();
    //         if (tly_detection < 0)
    //             tly_detection = 0;
    //         std::string category_detection = bottle_detections.get(i).find("category").asString();
    //         yInfo() << "category: " << category_detection << " brx:" << brx_detection << " bry: " << bry_detection << " tlx: " << tlx_detection << " tly: " << tly_detection;
    //         std::array<int, 4> detection_bbox = {tlx_detection, tly_detection, brx_detection, bry_detection};
    //         computeIntersectionOverUnion(detection_bbox, cluster_bbox, iou);
    //         if (iou > best_iou)
    //         {
    //             current_category = category_detection;
    //             best_iou = iou;
    //         }
    //     }
    //     // Check what we have in the bottle
    //     bool add_new = true;
    //     for (unsigned int i = 0; i < label_category.size(); i++)
    //     {
    //         yarp::os::Bottle *b = label_category.get(i).asList();
    //         if (b->get(0).find("category").asString() == current_category)
    //         {
    //             if (b->get(0).find("iou").asFloat64() <= best_iou)
    //             {
    //                 b->clear();
    //                 break;
    //             }
    //             else
    //             {
    //                 add_new = false;
    //                 break;
    //             }
    //         }
    //     }
    //     if (best_iou == 0)
    //         add_new = false;
    //     if (add_new)
    //     {
    //         int label = transformed_source_cloud->points[0].label;
    //         b.addDict() = {
    //             {"category", yarp::os::Value(current_category)},
    //             {"iou", yarp::os::Value(best_iou)},
    //             {"label", yarp::os::Value(label)},
    //         };
    //         label_category.addList() = b;
    //     }
    // }

    // for (unsigned int i = 0; i < label_category.size(); i++)
    // {
    //     yarp::os::Bottle *b = label_category.get(i).asList();
    //     std::string category = b->get(0).find("category").asString();
    //     bool already_seen = false; //
    //     int rgb[3] = {0, 0, 0};
    //     for (unsigned int j = 0; j < m_category_rgb.size(); j++)
    //     {
    //         yarp::os::Bottle *b_rgb = m_category_rgb.get(j).asList();
    //         if (b_rgb->get(0).find("category").asString() == b->get(0).find("category").asString())
    //         {
    //             already_seen = true;
    //             rgb[0] = b_rgb->get(0).find("r").asInt32();
    //             rgb[1] = b_rgb->get(0).find("g").asInt32();
    //             rgb[2] = b_rgb->get(0).find("b").asInt32();
    //             break;
    //         }
    //     }
    //     if (!already_seen)
    //     {
    //         yarp::os::Bottle b_rgb;
    //         rgb[0] = rand() % 256;
    //         rgb[1] = rand() % 256;
    //         rgb[2] = rand() % 256;
    //         b_rgb.addDict() = {
    //             {"category", yarp::os::Value(b->get(0).find("category"))},
    //             {"r", yarp::os::Value(rgb[0])},
    //             {"g", yarp::os::Value(rgb[1])},
    //             {"b", yarp::os::Value(rgb[2])},
    //         };
    //         m_category_rgb.addList() = b_rgb;
    //     }
    //     int label_cloud = b->get(0).find("label").asInt32();
    //     std::map<int, int>::iterator it = map_labels.find(label_cloud);
    //     if (it != map_labels.end())
    //     {
    //         pcl::PointXYZRGBA point;
    //         point.r = rgb[0];
    //         point.g = rgb[1];
    //         point.b = rgb[2];
    //         point.a = 0;
    //         for (size_t pidx = 0; pidx < sourceClouds[it->second]->points.size(); pidx++)
    //         {
    //             point.x = sourceClouds[it->second]->points[pidx].x;
    //             point.y = sourceClouds[it->second]->points[pidx].y;
    //             point.z = sourceClouds[it->second]->points[pidx].z;
    //             lccp_colored_cloud->push_back(*(pcl::PointXYZRGBA *)(&point));
    //         }
    //         fullObjectPointCloud(sourceClouds[it->second], category, filling_cloud, rgb);
    //     }
    // }
    // yInfo() << m_bGraspingPoses.toString();
}

bool GetGraspingPoses::read(yarp::os::ConnectionReader &connection)
{

    yarp::os::Bottle reply;
    auto *writer = connection.getWriter();

    yarp::os::Bottle command;
    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }

    if (command.size() == 0)
    {
        yWarning() << "Got empty bottle";
        yarp::os::Bottle reply{yarp::os::Value(VOCAB_FAIL, true)};
        return reply.write(*writer);
    }

    yarp::os::Bottle *b = command.get(0).asList();
    yInfo() << command.toString();

    switch (command.get(0).asVocab32())
    {
    case VOCAB_CMD_PAUSE:
    {
        m_update_data = false;
        yarp::os::Bottle reply{yarp::os::Value(VOCAB_OK, true)};
        return reply.write(*writer);
    }
    case VOCAB_CMD_RESUME:
    {
        m_update_data = true;
        yarp::os::Bottle reply{yarp::os::Value(VOCAB_OK, true)};
        return reply.write(*writer);
    }
    case VOCAB_CMD_GET_SUPERQUADRICS_BBOXES:
    {
        mtx.lock();
        yarp::rosmsg::visualization_msgs::Marker marker;
        yarp::rosmsg::visualization_msgs::MarkerArray markerArray;

        marker.header.frame_id = "waist";
        static int counter = 0;
        marker.header.stamp.nsec = 0;
        marker.header.stamp.sec = 0;

        marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
        marker.points.clear();
        markerArray.markers.push_back(marker);

        if (bbox3d_topic)
        {
            yInfo("Publish...\n");
            bbox3d_topic->write(markerArray);
        }

        for (unsigned int i = 0; i < m_superquadric_objects.size(); i++)
        {
            std::array<int, 4> bbox;
            createBoundingBox2DFromSuperquadric(m_superquadric_objects[i].superqs, bbox);
            yarp::os::Property bboxDict;
            bboxDict.put("label_idx", m_superquadric_objects[i].label);
            bboxDict.put("tlx", yarp::os::Value(bbox[0]));
            bboxDict.put("tly", yarp::os::Value(bbox[1]));
            bboxDict.put("brx", yarp::os::Value(bbox[2]));
            bboxDict.put("bry", yarp::os::Value(bbox[3]));
            reply.addDict() = bboxDict;
        }
        mtx.unlock();
        return reply.write(*writer);
    }
    case VOCAB_CMD_GET_GRASPING_POSES:
    {
        mtx.lock();
        yInfo() << command.toString();
        bool found = false;
        int i;
        for (i = 0; i < m_superquadric_objects.size(); i++)
        {
            if (m_superquadric_objects[i].label == command.get(1).asVocab32())
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            mtx.unlock();
            yarp::os::Bottle reply{yarp::os::Value(VOCAB_FAIL, true)};
            return reply.write(*writer);
        }
        else
        {
            std::vector<std::vector<double>> graspingPoses;
            computeGraspingPoses(m_superquadric_objects[i].superqs, graspingPoses);
            yarp::os::Bottle bPose;
            yarp::os::Bottle bObject;
            for (int i = 0; i < graspingPoses.size(); i++)
            {
                bPose.clear();
                for (int j = 0; j < 6; j++)
                {
                    bPose.addFloat64(graspingPoses[i][j]);
                }
                reply.addList() = bPose;
            }
            mtx.unlock();
            yInfo() << "graspingPoses.size(): " << graspingPoses.size();
            if (graspingPoses.size() > 0)
                rosSendGraspingPoses("waist", graspingPoses);
            return reply.write(*writer);
        }
    }
    case VOCAB_CMD_GET_SUPERQUADRICS:
    {
        yInfo() << "Getting superquadrics";
        for (int i = 0; i < m_superquadric_objects.size(); i++)
        {
            auto params = m_superquadric_objects[i].superqs[0].getSuperqParams();
            yarp::os::Property bboxDict;
            bboxDict.put("label_idx", m_superquadric_objects[i].label);
            bboxDict.put("axes0", yarp::os::Value(params[0]));
            bboxDict.put("axes1", yarp::os::Value(params[1]));
            bboxDict.put("axes2", yarp::os::Value(params[2]));
            bboxDict.put("e1", yarp::os::Value(params[3]));
            bboxDict.put("e2", yarp::os::Value(params[4]));
            bboxDict.put("x", yarp::os::Value(params[5]));
            bboxDict.put("y", yarp::os::Value(params[6]));
            bboxDict.put("z", yarp::os::Value(params[7]));
            bboxDict.put("roll", yarp::os::Value(params[8]));
            bboxDict.put("pitch", yarp::os::Value(params[9]));
            bboxDict.put("yaw", yarp::os::Value(params[10]));
            reply.addDict() = bboxDict;
        }
        return reply.write(*writer);
    }
    case VOCAB_CMD_REMOVE_SUPERQUADRIC:
    {
        yInfo() << "Removing superquadric";

        mtx.lock();
        yInfo() << command.toString();
        bool found = false;
        int i;
        for (i = 0; i < m_superquadric_objects.size(); i++)
        {
            if (m_superquadric_objects[i].label == command.get(1).asVocab32())
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            mtx.unlock();
            yarp::os::Bottle reply{yarp::os::Value(VOCAB_FAIL, true)};
            return reply.write(*writer);
        }
        else
        {
            m_superquadric_objects.erase(m_superquadric_objects.begin() + i);
            yarp::os::Bottle reply{yarp::os::Value(VOCAB_OK, true)};
            return reply.write(*writer);
        }
    }

    default:
        yarp::os::Bottle reply{yarp::os::Value(VOCAB_FAIL, true)};
        return reply.write(*writer);
    }

    // if (b->get(0).toString().find("update") != std::string::npos)
    // {
    //     reply.addString("update changed");
    //     mtx.lock();
    //     m_aux_update = b->get(0).find("update").asInt32();
    //     yInfo() << m_aux_update;
    //     mtx.unlock();
    // }
    // if (b->get(0).check("category"))
    // {
    //     yInfo() << "category: " << b->get(0).find("category").asString();
    //     std::string current_category = b->get(0).find("category").asString();
    //     for (int i = 0; i < m_bGraspingPoses.size(); i++)
    //     {
    //         yarp::os::Bottle *graspObject = m_bGraspingPoses.get(i).asList();
    //         yInfo() << current_category;
    //         yInfo() << graspObject->get(0).toString();
    //         if (current_category == graspObject->get(0).toString())
    //         {
    //             yInfo() << "found";
    //             reply.addList() = *graspObject;
    //             yInfo() << "------------------------------";
    //         }
    //     }
    // }

    // return reply.write(*writer);
}

void GetGraspingPoses::completeBox(std::vector<KDL::Vector> &normals, std::vector<pcl::PointXYZRGBA> &centroids,
                                   std::vector<pcl::PointXYZRGBA> &maxPoints, std::vector<pcl::PointXYZRGBA> &minPoints,
                                   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud, int rgb[3], float box_shape[3])
{

    yInfo() << "Complete Box";
    // First,we just keep the data from the front plane, from all the planes received
    KDL::Vector normal_to_keep = normals[0];
    pcl::PointXYZRGBA centroid_to_keep = centroids[0];
    pcl::PointXYZRGBA maxPoint_to_keep = maxPoints[0];
    pcl::PointXYZRGBA minPoint_to_keep = minPoints[0];
    if (normals.size() > 1)
    {
        for (unsigned int i = 1; i < normals.size(); i++)
        {
            if (fabs(normals[i][0]) > fabs(normal_to_keep[0]))
            {
                normal_to_keep = normals[i];
                centroid_to_keep = centroids[i];
                maxPoint_to_keep = maxPoints[i];
                minPoint_to_keep = minPoints[i];
            }
        }
    }

    normals.clear();
    centroids.clear();
    maxPoints.clear();
    minPoints.clear();

    normals.push_back(normal_to_keep);
    centroids.push_back(centroid_to_keep);
    maxPoints.push_back(maxPoint_to_keep);
    minPoints.push_back(minPoint_to_keep);

    // Here, we just have the data from the front plane

    if (normals.size() == 1)
    {
        // yInfo()<<"Max points: "<< maxPoints[0].x << maxPoints[0].y<<  maxPoints[0].z;
        // yInfo()<<"Min points: "<< minPoints[0].x << minPoints[0].y<<  minPoints[0].z;
        float cerealSizes[3] = {maxPoints[0].x - minPoints[0].x, maxPoints[0].y - minPoints[0].y, maxPoints[0].z - minPoints[0].z};
        // yInfo()<<"Cereal size: "<<cerealSizes[0] <<cerealSizes[1]<<cerealSizes[2];
        float marginShape = 0.05;
        float box_sizes[3] = {0.0, 0, 0};
        if (cerealSizes[1] >= (box_shape[2] - marginShape) && (cerealSizes[1] <= (box_shape[2] + marginShape)))
        {
            box_sizes[0] = box_shape[2];
            // yInfo()<<"X Widest";
            if (cerealSizes[2] >= (box_shape[1] - marginShape) && (cerealSizes[2] <= (box_shape[1] + marginShape)))
            {
                // yInfo()<<"Z Middle Wide";
                box_sizes[1] = box_shape[1];
                box_sizes[2] = box_shape[0];
            }
            else
            {
                // yInfo()<<"Z least wide";
                box_sizes[1] = box_shape[0];
                box_sizes[2] = box_shape[1];
            }

            fillBoxPointCloud(filling_cloud, centroids[0], normals[0], box_sizes, rgb);
        }
        if (cerealSizes[1] >= (box_shape[1] - marginShape) && (cerealSizes[1] <= (box_shape[1] + marginShape)))
        {
            box_sizes[0] = box_shape[1];

            // yInfo()<<"X Middle wide";
            if (cerealSizes[2] >= (box_shape[2] - marginShape) && (cerealSizes[2] <= (box_shape[2] + marginShape)))
            {
                // yInfo()<<"Z widest";
                box_sizes[1] = box_shape[2];
                box_sizes[2] = box_shape[0];
            }
            else
            {
                // yInfo()<<"Z least wide";
                box_sizes[1] = box_shape[0];
                box_sizes[2] = box_shape[2];
            }
            fillBoxPointCloud(filling_cloud, centroids[0], normals[0], box_sizes, rgb);
        }

        if ((cerealSizes[1] >= (box_shape[0] - marginShape)) && (cerealSizes[1] <= (box_shape[0] + marginShape)))
        {
            // yInfo()<<"X least wide";
            box_sizes[0] = box_shape[0];
            if (cerealSizes[2] >= (box_shape[2] - marginShape) && (cerealSizes[2] <= (box_shape[2] + marginShape)))
            {
                // yInfo()<<"Z widest";
                box_sizes[1] = box_shape[2];
                box_sizes[2] = box_shape[1];
            }
            else
            {
                // yInfo()<<"Z middle wide";
                box_sizes[1] = box_shape[1];
                box_sizes[2] = box_shape[2];
            }
            fillBoxPointCloud(filling_cloud, centroids[0], normals[0], box_sizes, rgb);
        }
        // This computes the grasping poses for all the detected objects. Maybe is better to compute it for the detected object.

        std::vector<KDL::Vector> zaxes;
        std::vector<KDL::Vector> xaxes;
        std::vector<KDL::Vector> yaxes;
        std::vector<pcl::PointXYZRGBA> centers;
        if (box_sizes[0] <= m_grasp_width)
        {

            for (float d = 0; d < box_sizes[1] / 4.0; d += (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + d * (KDL::Vector(0, 0, 1));
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];

                // centers.push_back(center);
                // zaxes.push_back(normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // xaxes.push_back(yaxes[0]*normals[0]);

                centers.push_back(center);
                zaxes.push_back(normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                xaxes.push_back(yaxes[0] * normals[0]);
            }
            for (float d = -(box_sizes[1] / 4.0) / 20; d > -box_sizes[1] / 4.0; d -= (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + d * (KDL::Vector(0, 0, 1));
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];
                // centers.push_back(center);
                // zaxes.push_back(normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // xaxes.push_back(yaxes[0]*normals[0]);

                centers.push_back(center);
                zaxes.push_back(normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                xaxes.push_back(yaxes[0] * normals[0]);
            }

            for (float d = 0; d < box_sizes[1] / 4.0; d += (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + box_sizes[2] * normals[0] + d * KDL::Vector(0, 0, 1);
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];
                // centers.push_back(center);
                // zaxes.push_back(-normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // xaxes.push_back(yaxes[1]*zaxes[1]);

                centers.push_back(center);
                zaxes.push_back(-normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                xaxes.push_back(yaxes[yaxes.size() - 1] * zaxes[zaxes.size() - 1]);
            }
            for (float d = -(box_sizes[1] / 4.0) / 20; d > -box_sizes[1] / 4.0; d -= (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + box_sizes[2] * normals[0] + d * KDL::Vector(0, 0, 1);
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];
                // centers.push_back(center);
                // zaxes.push_back(-normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // xaxes.push_back(yaxes[1]*zaxes[1]);

                centers.push_back(center);
                zaxes.push_back(-normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                xaxes.push_back(yaxes[yaxes.size() - 1] * zaxes[zaxes.size() - 1]);
            }
        }

        if (box_sizes[2] <= m_grasp_width)
        {
            for (float d = 0; d < box_sizes[1] / 4.0; d += (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + box_sizes[2] / 2.0 * normals[0] + (normals[0] * KDL::Vector(0, 0, 1)) * box_sizes[0] / 2.0 + d * KDL::Vector(0, 0, 1);

                // xaxes.push_back(normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // zaxes.push_back(-(xaxes.back()*yaxes.back()));
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];
                // centers.push_back(center);

                xaxes.push_back(normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                zaxes.push_back((xaxes.back() * yaxes.back()));
                centers.push_back(center);
            }

            for (float d = -(box_sizes[1] / 4.0) / 20; d > -box_sizes[1] / 4.0; d -= (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + box_sizes[2] / 2.0 * normals[0] + (normals[0] * KDL::Vector(0, 0, 1)) * box_sizes[0] / 2.0 + d * KDL::Vector(0, 0, 1);

                // xaxes.push_back(normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // zaxes.push_back(-(xaxes.back()*yaxes.back()));
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];
                // centers.push_back(center);

                xaxes.push_back(normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                zaxes.push_back((xaxes.back() * yaxes.back()));
                centers.push_back(center);
            }

            for (float d = 0; d < box_sizes[1] / 4.0; d += (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + box_sizes[2] / 2.0 * normals[0] - (normals[0] * KDL::Vector(0, 0, 1)) * box_sizes[0] / 2.0 + d * KDL::Vector(0, 0, 1);
                // xaxes.push_back(normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // zaxes.push_back((xaxes.back()*yaxes.back()));
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];
                // centers.push_back(center);

                centers.push_back(center);
                xaxes.push_back(-normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                zaxes.push_back((xaxes.back() * yaxes.back()));
            }

            for (float d = -(box_sizes[1] / 4.0) / 20; d > -box_sizes[1] / 4.0; d -= (box_sizes[1] / 4.0) / 20)
            {
                KDL::Vector aux_center = KDL::Vector(centroids[0].x, centroids[0].y, centroids[0].z) + box_sizes[2] / 2.0 * normals[0] - (normals[0] * KDL::Vector(0, 0, 1)) * box_sizes[0] / 2.0 + d * KDL::Vector(0, 0, 1);
                // xaxes.push_back(normals[0]);
                // yaxes.push_back(KDL::Vector(0,0,1));
                // zaxes.push_back((xaxes.back()*yaxes.back()));
                pcl::PointXYZRGBA center;
                center.x = aux_center[0];
                center.y = aux_center[1];
                center.z = aux_center[2];
                // centers.push_back(center);

                centers.push_back(center);

                xaxes.push_back(-normals[0]);
                yaxes.push_back(-KDL::Vector(0, 0, 1));
                zaxes.push_back((xaxes.back() * yaxes.back()));
            }
            // aux_center = KDL::Vector (centroids[0].x,centroids[0].y, centroids[0].z) + box_sizes[1]/2.0*yaxes.back();
            // center.x = aux_center[0];
            // center.y = aux_center[1];
            // center.z = aux_center[2];
            // centers.push_back(center);
            // zaxes.push_back(-yaxes.back());
            // yaxes.push_back(xaxes.back());
            // xaxes.push_back(yaxes.back()*zaxes.back());
        }
        for (int i = 0; i < zaxes.size(); i++)
        {
            KDL::Rotation rot = KDL::Rotation(xaxes[i], yaxes[i], zaxes[i]);
            KDL::Frame frameTCP(rot);
            KDL::Frame frame_goal = frameTCP;
            frame_goal.p[0] = centers[i].x;
            frame_goal.p[1] = centers[i].y;
            frame_goal.p[2] = centers[i].z;
            std::vector<double> tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_goal);
            // printf("%f %f %f %f %f %f\n",tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);
            yarp::os::Bottle bPose;
            for (int j = 0; j < 6; j++)
            {
                bPose.addFloat64(tcpX[j]);
            }
            m_bObject.addList() = bPose;
        }
        // yInfo()<<m_bObject.toString();

        rosComputeGraspingPosesArrowAndSend("waist", centers, xaxes, yaxes, zaxes);
    }
}
void GetGraspingPoses::fillBoxPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filling_cloud,
                                         const pcl::PointXYZRGBA &centroid, const KDL::Vector &normal,
                                         float box_sizes[3], int rgb[3])
{

    yInfo() << "Filling box Point cloud";
    Eigen::Vector3f x(normal[0], normal[1], normal[2]);
    Eigen::Vector3f z(0, 0, 1);
    Eigen::Vector3f y = z.cross(x);
    // std::cout << "Cross product:\n" << z.cross(x) << std::endl;
    Eigen::Vector3f center_front_side(centroid.x, centroid.y, centroid.z);
    // yInfo()<<"Box size: "<<box_sizes[0]<<" "<<box_sizes[1]<<" "<<box_sizes[2];
    for (float distance_planes = 0; distance_planes <= box_sizes[2]; distance_planes += box_sizes[2])
    {
        center_front_side += distance_planes * x;
        for (float dx = -box_sizes[0] / 2.0; dx <= box_sizes[0] / 2.0; dx += m_resolution_filling_cloud)
        {
            for (float dz = -box_sizes[1] / 2.0; dz <= box_sizes[1] / 2.0; dz += m_resolution_filling_cloud)
            {
                Eigen::Vector3f aux_p = center_front_side + dx * y + dz * z;
                pcl::PointXYZRGBA point;
                point.r = rgb[0];
                point.g = rgb[1];
                point.b = rgb[2];
                point.a = 0;
                point.x = aux_p[0];
                point.y = aux_p[1];
                point.z = aux_p[2];
                filling_cloud->push_back(*(pcl::PointXYZRGBA *)(&point));
            }
        }
    }

    // Front side is thin enough to be grasped

    // yarp::os::Bottle bAllPoses;
    // if(box_sizes[0]<=m_grasp_width){
    //     center_front_side[0] = centroid.x;
    //     center_front_side[1] = centroid.y;
    //     center_front_side[2] = centroid.z;
    //     for(float dz = -box_sizes[1]/3.0; dz<= box_sizes[1]/3.0; dz += 0.002){
    //         Eigen::Vector3f aux = center_front_side+dz*z;
    //         yarp::os::Bottle bPose;
    //         bPose.addFloat64(aux.x);
    //         bPose.addFloat64(aux.y);
    //         bPose.addFloat64(aux.z);
    //         KDL::Rotation rot = KDL::Rotation(y,z,x);
    //         KDL::Frame frameTCP(rot);
    //         KDL::Frame frame_goal = frameTCP;
    //         frame_goal.p[0] = aux.x;
    //         frame_goal.p[1] = aux.y;
    //         frame_goal.p[2] = aux.z;
    //         std::vector<double> tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_goal);
    //         for(int j = 0; j < 6; j++){
    //             bPose.addFloat64(tcpX[j]);
    //         }
    //         bAllPoses.addList() = bPose;
    //     }
    // }

    Eigen::Vector3f center_top_side(centroid.x, centroid.y, centroid.z);

    for (float distance_planes = -box_sizes[1] / 2.0; distance_planes <= box_sizes[1] / 2.0; distance_planes += box_sizes[1])
    {
        Eigen::Vector3f aux_center = center_top_side + distance_planes * z;
        for (float dx = -box_sizes[0] / 2.0; dx <= box_sizes[0] / 2.0; dx += m_resolution_filling_cloud)
        {
            for (float dy = 0; dy <= box_sizes[2]; dy += m_resolution_filling_cloud)
            {
                Eigen::Vector3f aux_p = aux_center + dx * y + dy * x;
                pcl::PointXYZRGBA point;
                point.r = rgb[0];
                point.g = rgb[1];
                point.b = rgb[2];
                point.a = 0;
                point.x = aux_p[0];
                point.y = aux_p[1];
                point.z = aux_p[2];
                filling_cloud->push_back(*(pcl::PointXYZRGBA *)(&point));
            }
        }
    }

    // if(box_sizes[1]<=m_grasp_width){
    //     center_top_side[0] = centroid.x;
    //     center_top_side[1] = centroid.y;
    //     center_top_side[2] = centroid.z;
    //     center_top_side += box_sizes[1]/2.0*z + box_sizes[2]/2.0*x;
    //     for(float d = -box_sizes[2]/3.0; d<= box_sizes[2]/3.0; d += 0.002){
    //         Eigen::Vector3f aux = center_top_side+d*x;
    //         yarp::os::Bottle bPose;
    //         bPose.addFloat64(aux.x);
    //         bPose.addFloat64(aux.y);
    //         bPose.addFloat64(aux.z);
    //         KDL::Rotation rot = KDL::Rotation(y,x,z);
    //         KDL::Frame frameTCP(rot);
    //         KDL::Frame frame_goal = frameTCP;
    //         frame_goal.p[0] = aux.x;
    //         frame_goal.p[1] = aux.y;
    //         frame_goal.p[2] = aux.z;
    //         std::vector<double> tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_goal);
    //         for(int j = 0; j < 6; j++){
    //             bPose.addFloat64(tcpX[j]);
    //         }
    //         bAllPoses.addList() = bPose;
    //     }
    // }

    for (float distance_planes = -box_sizes[0] / 2.0; distance_planes <= box_sizes[0] / 2.0; distance_planes += box_sizes[0])
    {
        Eigen::Vector3f aux_center = center_top_side + distance_planes * y;
        for (float dz = -box_sizes[1] / 2.0; dz <= box_sizes[1] / 2.0; dz += m_resolution_filling_cloud)
        {
            for (float dy = 0; dy <= box_sizes[2]; dy += m_resolution_filling_cloud)
            {
                Eigen::Vector3f aux_p = aux_center + dz * z + dy * x;
                pcl::PointXYZRGBA point;
                point.r = rgb[0];
                point.g = rgb[1];
                point.b = rgb[2];
                point.a = 0;
                point.x = aux_p[0];
                point.y = aux_p[1];
                point.z = aux_p[2];
                filling_cloud->push_back(*(pcl::PointXYZRGBA *)(&point));
            }
        }
    }
    // // Eigen::Vector3f center_left_side = center_fr

    // if(box_sizes[2]<=m_grasp_width){
    //     center_top_side[0] = centroid.x;
    //     center_top_side[1] = centroid.y;
    //     center_top_side[2] = centroid.z;
    //     center_top_side += box_sizes[1]/2.0*z + box_sizes[2]/2.0*x;
    //     for(float d = -box_sizes[2]/3.0; d<= box_sizes[2]/3.0; d += 0.002){
    //         Eigen::Vector3f aux = center_top_side+d*x;
    //         yarp::os::Bottle bPose;
    //         bPose.addFloat64(aux.x);
    //         bPose.addFloat64(aux.y);
    //         bPose.addFloat64(aux.z);
    //         KDL::Rotation rot = KDL::Rotation(y,x,z);
    //         KDL::Frame frameTCP(rot);
    //         KDL::Frame frame_goal = frameTCP;
    //         frame_goal.p[0] = aux.x;
    //         frame_goal.p[1] = aux.y;
    //         frame_goal.p[2] = aux.z;
    //         std::vector<double> tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_goal);
    //         for(int j = 0; j < 6; j++){
    //             bPose.addFloat64(tcpX[j]);
    //         }
    //         bAllPoses.addList() = bPose;
    //     }
    // }
}
void GetGraspingPoses::getMinimumOrientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSegmented, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &bbox)
{
    pcl::PointXYZRGBA maxPoint;
    pcl::PointXYZRGBA minPoint;
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                   ///    the signs are different and the box doesn't get correctly oriented in some cases.
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    // pcl::PointXYZRGBA minPoint, maxPoint;
    // pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    pcl::getMinMax3D(*cloudSegmented, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr bbox_aux(new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::cout << "bboxTransform: " << bboxTransform << std::endl;
    pcl::PointXYZRGBA auxp;
    auxp.x = maxPoint.x;
    auxp.y = maxPoint.y;
    auxp.z = maxPoint.z;
    bbox_aux->points.push_back(auxp);
    auxp.x = minPoint.x;
    auxp.y = minPoint.y;
    auxp.z = minPoint.z;
    bbox_aux->points.push_back(auxp);
    // auxp.x = -(maxPoint.x - minPoint.x) / 2.0;
    // auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    // auxp.z = -(maxPoint.z - minPoint.z) / 2.0;
    // bbox_aux->points.push_back(auxp);
    // auxp.y = (maxPoint.y - minPoint.y) / 2.0;
    // bbox_aux->points.push_back(auxp);
    // auxp.z = (maxPoint.z - minPoint.z) / 2.0;
    // bbox_aux->points.push_back(auxp);
    // auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    // bbox_aux->points.push_back(auxp);

    // auxp.x = (maxPoint.x - minPoint.x) / 2.0;
    // auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    // auxp.z = -(maxPoint.z - minPoint.z) / 2.0;
    // bbox_aux->points.push_back(auxp);
    // auxp.y = (maxPoint.y - minPoint.y) / 2.0;
    // bbox_aux->points.push_back(auxp);
    // auxp.z = (maxPoint.z - minPoint.z) / 2.0;
    // bbox_aux->points.push_back(auxp);
    // auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    // bbox_aux->points.push_back(auxp);

    for (int i = 0; i < bbox_aux->points.size(); ++i)
    {
        std::cout << "point: " << bbox_aux->points[i].x << " " << bbox_aux->points[i].y << " " << bbox_aux->points[i].z << std::endl;
    }

    Eigen::Matrix3f mat3 = bboxQuaternion.toRotationMatrix();
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block(0, 0, 3, 3) = mat3;
    transform.col(3) << pcaCentroid[0], pcaCentroid[1], pcaCentroid[2], 1;

    pcl::transformPointCloud(*bbox_aux, *bbox, transform);

    // pcl::transformPointCloud(*bbox_aux, *bbox, projectionTransform);

    std::cout << "bbox->points.size(): " << bbox_aux->points.size() << std::endl;

    std::cout << "Centroid: " << pcaCentroid[0] << " " << pcaCentroid[1] << " " << pcaCentroid[2] << " " << pcaCentroid[3] << std::endl;
}
void GetGraspingPoses::getMinimumBoundingBoxPointCLoud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSegmented, pcl::PointXYZRGBA &maxPoint, pcl::PointXYZRGBA &minPoint)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                   ///    the signs are different and the box doesn't get correctly oriented in some cases.
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    // pcl::PointXYZRGBA minPoint, maxPoint;
    // pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    pcl::getMinMax3D(*cloudSegmented, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    std::cout << "bboxTransform: " << bboxTransform << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_aux(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointXYZRGBA auxp;
    auxp.x = -(maxPoint.x - minPoint.x) / 2.0;
    auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    auxp.z = -(maxPoint.z - minPoint.z) / 2.0;
    auxp.r = 255;
    cloud_aux->points.push_back(auxp);
    auxp.y = (maxPoint.y - minPoint.y) / 2.0;
    cloud_aux->points.push_back(auxp);
    auxp.z = (maxPoint.z - minPoint.z) / 2.0;
    cloud_aux->points.push_back(auxp);
    auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    cloud_aux->points.push_back(auxp);

    auxp.x = (maxPoint.x - minPoint.x) / 2.0;
    auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    auxp.z = -(maxPoint.z - minPoint.z) / 2.0;
    cloud_aux->points.push_back(auxp);
    auxp.y = (maxPoint.y - minPoint.y) / 2.0;
    cloud_aux->points.push_back(auxp);
    auxp.z = (maxPoint.z - minPoint.z) / 2.0;
    cloud_aux->points.push_back(auxp);
    auxp.y = -(maxPoint.y - minPoint.y) / 2.0;
    cloud_aux->points.push_back(auxp);

    std::cout << "cloud_aux->points.size(): " << cloud_aux->points.size() << std::endl;
    // yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloudWithoutHorizontalSurface;
    // yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(*cloud_aux, yarpCloudWithoutHorizontalSurface);

    // // yInfo()<<"Point cloud transformed";
    // rosComputeAndSendPc(yarpCloudWithoutHorizontalSurface, "waist", *pointCloudWithoutPlannarSurfaceTopic);

    // std::cout << "Centroid: " << pcaCentroid[0] << " " << pcaCentroid[1] << " " << pcaCentroid[2] << " " << pcaCentroid[3] << std::endl;
}

void GetGraspingPoses::getMinimumBoundingBoxPointCLoud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudSegmented, pcl::PointXYZRGBA &maxPoint, pcl::PointXYZRGBA &minPoint, KDL::Vector n)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
    Eigen::Vector3f x(n[0], n[1], n[2]);
    Eigen::Vector3f z(0, 0, 1);
    Eigen::Vector3f y = z.cross(x);
    std::cout << "Cross product:\n"
              << z.cross(x) << std::endl;

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    Eigen::Matrix3f rotationTransform;
    rotationTransform << x, y, z;

    projectionTransform.block<3, 3>(0, 0) = rotationTransform.transpose();
    projectionTransform.block<3, 1>(0, 3) = 1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());

    // Eigen::Matrix3f covariance;
    // computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    // Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    // eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    //                                                                                 ///    the signs are different and the box doesn't get correctly oriented in some cases.
    // // Transform the original cloud to the origin where the principal components correspond to the axes.
    // Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    // projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    // projectionTransform.block<3,1>(0,3) = 1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    // pcl::PointXYZRGBA minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    // const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // // Final transform
    // const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    // const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
}

void GetGraspingPoses::rosSendGraspingPoses(const std::string &frame_id, const std::vector<std::vector<double>> &graspingPoses)
{
    yarp::rosmsg::visualization_msgs::Marker marker;
    yarp::rosmsg::visualization_msgs::MarkerArray markerArray;

    marker.header.frame_id = frame_id;
    static int counter = 0;
    marker.header.stamp.nsec = 0;
    marker.header.stamp.sec = 0;

    marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);

    if (graspingPoses_outTopic)
    {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }
    for (int idx = 0; idx < graspingPoses.size(); idx++)
    {
        KDL::Frame frame = roboticslab::KdlVectorConverter::vectorToFrame(graspingPoses[idx]);
        KDL::Vector unitx = frame.M.UnitX();
        KDL::Vector unity = frame.M.UnitY();
        KDL::Vector unitz = frame.M.UnitZ();
        // std::cout << frame << std::endl;

        // std::cout << "unitx: " << unitx[0] << " " << unitx[1] << " " << unitx[2] << std::endl;

        // yarp::rosmsg::visualization_msgs::Marker marker;
        // yarp::rosmsg::visualization_msgs::MarkerArray markerArray;

        marker.header.frame_id = frame_id;
        static int counter = 0;
        marker.header.stamp.nsec = 0;
        marker.header.stamp.sec = 0;

        // marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
        // markerArray.markers.push_back(marker);

        // if (graspingPoses_outTopic)
        // {
        //     yInfo("Publish...\n");
        //     graspingPoses_outTopic->write(markerArray);
        // }

        float lengthArrow = 0.1;
        KDL::Vector auxV1 = unitz * lengthArrow;
        marker.header.seq = counter++;
        marker.id = idx;
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        yarp::rosmsg::geometry_msgs::Point pointRos;
        pointRos.x = frame.p[0] - auxV1[0];
        pointRos.y = frame.p[1] - auxV1[1];
        pointRos.z = frame.p[2] - auxV1[2];
        marker.points.clear();
        marker.points.push_back(pointRos);
        pointRos.x = frame.p[0];
        pointRos.y = frame.p[1];
        pointRos.z = frame.p[2];
        marker.pose.orientation.w = 1.0;
        marker.points.push_back(pointRos);
        marker.scale.x = 0.005;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        markerArray.markers.push_back(marker);

        auxV1 = unitx * lengthArrow;
        marker.header.seq = counter++;
        marker.id = idx + graspingPoses.size();
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        pointRos.x = frame.p[0] - auxV1[0];
        pointRos.y = frame.p[1] - auxV1[1];
        pointRos.z = frame.p[2] - auxV1[2];
        marker.points.clear();
        marker.points.push_back(pointRos);
        pointRos.x = frame.p[0];
        pointRos.y = frame.p[1];
        pointRos.z = frame.p[2];
        marker.pose.orientation.w = 1.0;
        marker.points.push_back(pointRos);
        marker.scale.x = 0.005;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);

        auxV1 = unity * lengthArrow;
        marker.header.seq = counter++;
        marker.id = idx + graspingPoses.size() * 2;
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        pointRos.x = frame.p[0] - auxV1[0];
        pointRos.y = frame.p[1] - auxV1[1];
        pointRos.z = frame.p[2] - auxV1[2];
        marker.points.clear();
        marker.points.push_back(pointRos);
        pointRos.x = frame.p[0];
        pointRos.y = frame.p[1];
        pointRos.z = frame.p[2];
        marker.pose.orientation.w = 1.0;
        marker.points.push_back(pointRos);
        marker.scale.x = 0.005;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
    }
    if (graspingPoses_outTopic)
    {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }
}

void GetGraspingPoses::rosComputeGraspingPosesArrowAndSend(const std::string &frame_id, std::vector<pcl::PointXYZRGBA> &centroids, std::vector<KDL::Vector> &xaxis, std::vector<KDL::Vector> &yaxis, std::vector<KDL::Vector> &normals)
{

    yarp::rosmsg::visualization_msgs::Marker marker;
    yarp::rosmsg::visualization_msgs::MarkerArray markerArray;

    marker.header.frame_id = frame_id;
    static int counter = 0;
    marker.header.stamp.nsec = 0;
    marker.header.stamp.sec = 0;

    marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);

    if (graspingPoses_outTopic)
    {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }

    for (int i = 0; i < centroids.size(); i++)
    {
        // printf("Normals %d: %f %f %f\n", i, normals[0], normals[1], normals[2]);
        float lengthArrow = 0.1;
        KDL::Vector auxV1 = normals[i] * lengthArrow;
        marker.header.seq = counter++;
        marker.id = i;
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        yarp::rosmsg::geometry_msgs::Point pointRos;
        pointRos.x = centroids[i].x - auxV1[0];
        pointRos.y = centroids[i].y - auxV1[1];
        pointRos.z = centroids[i].z - auxV1[2];
        marker.points.clear();
        marker.points.push_back(pointRos);
        pointRos.x = centroids[i].x;
        pointRos.y = centroids[i].y;
        pointRos.z = centroids[i].z;
        marker.pose.orientation.w = 1.0;
        marker.points.push_back(pointRos);
        marker.scale.x = 0.005;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        markerArray.markers.push_back(marker);
    }

    for (int i = 0; i < centroids.size(); i++)
    {
        // printf("Normals %d: %f %f %f\n", i, normals[0], normals[1], normals[2]);
        float lengthArrow = 0.1;
        KDL::Vector auxV1 = xaxis[i] * lengthArrow;
        marker.header.seq = counter++;
        marker.id = i + centroids.size();
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        yarp::rosmsg::geometry_msgs::Point pointRos;
        pointRos.x = centroids[i].x - auxV1[0];
        pointRos.y = centroids[i].y - auxV1[1];
        pointRos.z = centroids[i].z - auxV1[2];
        marker.points.clear();
        marker.points.push_back(pointRos);
        pointRos.x = centroids[i].x;
        pointRos.y = centroids[i].y;
        pointRos.z = centroids[i].z;
        marker.pose.orientation.w = 1.0;
        marker.points.push_back(pointRos);
        marker.scale.x = 0.005;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
    }

    for (int i = 0; i < centroids.size(); i++)
    {
        // printf("Normals %d: %f %f %f\n", i, normals[0], normals[1], normals[2]);
        float lengthArrow = 0.1;
        KDL::Vector auxV1 = yaxis[i] * lengthArrow;
        marker.header.seq = counter++;
        marker.id = i + centroids.size() * 2;
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        yarp::rosmsg::geometry_msgs::Point pointRos;
        pointRos.x = centroids[i].x - auxV1[0];
        pointRos.y = centroids[i].y - auxV1[1];
        pointRos.z = centroids[i].z - auxV1[2];
        marker.points.clear();
        marker.points.push_back(pointRos);
        pointRos.x = centroids[i].x;
        pointRos.y = centroids[i].y;
        pointRos.z = centroids[i].z;
        marker.pose.orientation.w = 1.0;
        marker.points.push_back(pointRos);
        marker.scale.x = 0.005;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
    }

    if (graspingPoses_outTopic)
    {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }
}

void GetGraspingPoses::rosComputeGraspingPosesArrowAndSend(const std::string &frame_id, std::vector<pcl::PointXYZRGBA> &centroids, std::vector<KDL::Vector> &normals)
{
    yarp::rosmsg::visualization_msgs::Marker marker;
    yarp::rosmsg::visualization_msgs::MarkerArray markerArray;

    marker.header.frame_id = frame_id;
    static int counter = 0;
    marker.header.stamp.nsec = 0;
    marker.header.stamp.sec = 0;

    marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);

    if (graspingPoses_outTopic)
    {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }

    for (int i = 0; i < centroids.size(); i++)
    {
        // printf("Normals %d: %f %f %f\n", i, normals[0], normals[1], normals[2]);
        float lengthArrow = 0.1;
        KDL::Vector auxV1 = normals[i] * lengthArrow;
        marker.header.seq = counter++;
        marker.id = i;
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        yarp::rosmsg::geometry_msgs::Point pointRos;
        pointRos.x = centroids[i].x - auxV1[0];
        pointRos.y = centroids[i].y - auxV1[1];
        pointRos.z = centroids[i].z - auxV1[2];
        marker.points.clear();
        marker.points.push_back(pointRos);
        pointRos.x = centroids[i].x;
        pointRos.y = centroids[i].y;
        pointRos.z = centroids[i].z;
        marker.pose.orientation.w = 1.0;
        marker.points.push_back(pointRos);
        marker.scale.x = 0.005;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }

    if (graspingPoses_outTopic)
    {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }
}