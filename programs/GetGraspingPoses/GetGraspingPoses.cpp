// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GetGraspingPoses.hpp"

#include <cstdio>

#include <ColorDebug.h>

using namespace sharon;

/************************************************************************/

bool GetGraspingPoses::configure(yarp::os::ResourceFinder & rf)
{
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("GetGraspingPoses options:\n");
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    printf("\t--RGBDDevice (device we create, default: \"%s\")\n",strRGBDDevice.c_str());
    printf("\t--RGBDLocal (if accesing remote, local port name, default: \"%s\")\n",strRGBDLocal.c_str());
    printf("\t--RGBDRemote (if accesing remote, remote port name, default: \"%s\")\n",strRGBDRemote.c_str());
    printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);


    if(rf.check("RGBDDevice")) strRGBDDevice = rf.find("RGBDDevice").asString();
    if(rf.check("RGBDLocal")) strRGBDLocal = rf.find("RGBDLocal").asString();
    if(rf.check("RGBDRemote")) strRGBDRemote = rf.find("RGBDRemote").asString();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asFloat64();

    printf("RgbdDetection using RGBDDevice: %s, RGBDLocal: %s, RGBDRemote: %s.\n",
           strRGBDDevice.c_str(), strRGBDLocal.c_str(), strRGBDRemote.c_str());
    printf("RgbdDetection using watchdog: %f.\n",watchdog);

    yarp::os::Property options;
    options.fromString( rf.toString() );  //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
    options.put("device",strRGBDDevice);  //-- Important to override in case there is a "device" in the future
    options.put("localImagePort",strRGBDLocal+"/rgbImage:i");
    options.put("localDepthPort",strRGBDLocal+"/depthImage:i");
    options.put("localRpcPort",strRGBDLocal+"/rpc:o");
    options.put("remoteImagePort",strRGBDRemote+"/rgbImage:o");
    options.put("remoteDepthPort",strRGBDRemote+"/depthImage:o");
    options.put("remoteRpcPort",strRGBDRemote+"/rpc:i");
    //if(rf.check("noMirror")) options.put("noMirror",1);  //-- Replaced by options.fromString( rf.toString() );

    if(!dd.open(options))
    {
        yError("Bad RGBDDevice \"%s\"...\n",strRGBDDevice.c_str());
        return false;
    }
    yInfo("RGBDDevice available.\n");

    if (! dd.view(iRGBDSensor) )
    {
        yError("RGBDDevice bad view.\n");
        return false;
    }
    yInfo("RGBDDevice ok view.\n");

    //-----------------OPEN LOCAL PORTS------------//
    std::string portPrefix(DEFAULT_PREFIX);
    portPrefix += strRGBDRemote;
    if(!outRgbImgPort.open(portPrefix + "/croppedImg:o"))
    {
        yError("Bad outRgbImgPort.open\n");
        return false;
    }
    if(!outDepthImgPort.open(portPrefix + "/croppedDepthImg:o"))
    {
        yError("Bad outDepthImgPort.open\n");
        return false;
    }

    if(!outPointCloudPort.open(portPrefix + "/pointCloud:o"))
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
        qrMin.addDouble(min);
        qrMax.addDouble(max);
        yInfo("Joint %d limits: [%f,%f]", joint, min, max);
    }

    if (robot == "/teoSim"){
        for (unsigned int joint = 0; joint < 2; joint++)
        {
            double min, max;
            headIControlLimits->getLimits(joint, &min, &max);
            qrMin.addDouble(min);
            qrMax.addDouble(max);
            yInfo("Joint %d limits: [%f,%f]", joint, min, max);
        }
    }
    else{
        for (unsigned int joint = 0; joint < 2; joint++)
        {
            qrMin.addDouble(-15);
            qrMax.addDouble(15.0);
        }
    }
    yarp::os::Property trunkAndHeadSolverOptions;
    rf.setDefaultContext("kinematics"); // context to find kinematic config files
    std::string trunkHeadKinPath = rf.findFileByName("teo-trunk-head.ini");
    trunkAndHeadSolverOptions.fromConfigFile(trunkHeadKinPath);
    trunkAndHeadSolverOptions.put("device","KdlSolver");
    trunkAndHeadSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    trunkAndHeadSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    trunkAndHeadSolverOptions.put("ik", "st"); // to use screw theory IK
    trunkAndHeadSolverDevice.open(trunkAndHeadSolverOptions);

    if( ! trunkAndHeadSolverDevice.isValid() )
    {
        yError() << "KDLSolver solver device for trunk and head is not valid";
        return false;
    }

    if( ! trunkAndHeadSolverDevice.view(trunkAndHeadSolverDeviceICartesianSolver) )
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    std::string prefix(DEFAULT_PREFIX);


    if (!rpcServer.open(portPrefix+"/rpc:s"))
    {
        yError() << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }


    yarp::os::Property depthIntrinsicParams;

    if(!iRGBDSensor->getDepthIntrinsicParam(depthIntrinsicParams))
    {
        yError("Cannot get depth params\n");
        return false;
    }


    intrinsics.fromProperty(depthIntrinsicParams);



    rosNode = new yarp::os::Node(DEFAULT_PREFIX);


    pointCloud_outTopic=new yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>;


    if (pointCloud_outTopic->topic("/depthToPointCloud")==false)
    {
        yError("Error opening depthToPointCloud topic.\n");
    }
    else{
        yInfo("Opening depthToPointCloud succesfully.\n");
    }

    
    graspingPoses_outTopic = new yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>;

    if (graspingPoses_outTopic->topic("/graspingPoses")==false)
    {
        yError("Error opening graspingPoses topic.\n");
    }
    else{
        yInfo("Opening graspingPoses succesfully.\n");
    }

    rpcServer.setReader(*this);


    yInfo("--- end: configure\n");
    return true;
}


/************************************************************************/

double GetGraspingPoses::getPeriod()
{
    return watchdog;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool GetGraspingPoses::updateModule()
{
    printf("GetGraspingPoses alive...\n");
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

    bool depth_ok = iRGBDSensor->getDepthImage(depthFrame);
    if (depth_ok == false)
    {
        yError()<< "getDepthImage failed";
        return false;
    }

    int min_x = 0;
    int min_y = 0;
    int max_x = depthFrame.width();
    int max_y = depthFrame.height();

    // mutexCloud.lock();
    yarp::sig::PointCloud<yarp::sig::DataXYZ> pcFiltered;
    pcFiltered.resize(depthFrame.width(),depthFrame.height());

    for (size_t i = 0, u = min_x; u < max_x; i++, u ++){
        for (size_t j = 0, v = min_y; v < max_y; j++, v ++) {
            if( depthFrame.pixel(u, v)<1.3){ // TODO: PARAMETER MAX DISTANCE FILTER
                pcFiltered(i, j).x = depthFrame.pixel(u, v); 
                pcFiltered(i, j).y = -(u - intrinsics.principalPointX) / intrinsics.focalLengthX * depthFrame.pixel(u, v);
                pcFiltered(i, j).z = -(v - intrinsics.principalPointY) / intrinsics.focalLengthY * depthFrame.pixel(u, v);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

    if(transformPointCloud(pcFiltered, transformed_cloud)){
        yInfo()<<"PointCloud transformed succesfully";
        yarp::sig::PointCloud<yarp::sig::DataXYZ> yarpCloud;
        yarp::pcl::fromPCL<pcl::PointXYZ, yarp::sig::DataXYZ>(*transformed_cloud, yarpCloud);

        // yInfo()<<"Point cloud transformed";
        rosComputeAndSendPc(yarpCloud, "map");
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


    dd.close();
    outRgbImgPort.close();
    outDepthImgPort.close();
    inMarchingObjectDataPort.close();

    return true;
}

/************************************************************************/

bool  GetGraspingPoses::openHeadDevice(){
    yarp::os::Property headOptions;
    headOptions.put("device","remote_controlboard");
    headOptions.put("remote",robot+"/head");
    headOptions.put("local",DEFAULT_PREFIX+robot+"/head");
    headDevice.open(headOptions);

    if( ! headDevice.isValid() )
    {
        printf("head remote_controlboard instantiation not worked.\n");
        return false;
    }


    if( ! headDevice.view(iHeadEncoders) )
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
bool GetGraspingPoses::openTrunkDevice(){
    yarp::os::Property trunkOptions;
    trunkOptions.put("device","remote_controlboard");
    trunkOptions.put("remote",robot+"/trunk");
    trunkOptions.put("local",DEFAULT_PREFIX+robot+"/trunk");
    trunkDevice.open(trunkOptions);

    if( ! trunkDevice.isValid() )
    {
        printf("trunk remote_controlboard instantiation not worked.\n");
        return false;
    }


    if( ! trunkDevice.view(iTrunkEncoders) )
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

/************************************************************************/
bool GetGraspingPoses::transformPointCloud(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>::Ptr & transformed_cloud){

    // if (!yarp::os::NetworkBase::exists(robot+"/trunk"+"/rpc:i"))
    // {
    //     yError()<<"Check if the device trunk is open!";
    // }
    // else{
    //     if(robot == "/teoSim"){
    //         if(!yarp::os::NetworkBase::isConnected(DEFAULT_PREFIX+robot+"/trunk"+"/rpc:o", "/"+robot+"/trunk"+"/rpc:i")){
    //             yWarning()<<robot+"/trunk"+" Device not connected";
    //             trunkDevice.close();
    //             openTrunkDevice();
    //         }
    //         if(!yarp::os::NetworkBase::isConnected(DEFAULT_PREFIX+robot+"/head"+"/rpc:o", "/"+robot+"/head"+"/rpc:i")){
    //             yWarning()<<robot+"/head"+" Device not connected";
    //             headDevice.close();
    //             openHeadDevice();
    //         }
    //     }
    //     else{
    //         if(!yarp::os::NetworkBase::isConnected(DEFAULT_PREFIX+robot+"/trunk"+"/rpc:o", "/"+robot+"/trunk"+"/rpc:i"))
    //         {
    //             yWarning()<<robot+"/trunk"+" Device not connected";
    //             trunkDevice.close();
    //             openTrunkDevice();
    //         }

    //     }
    //     yarp::os::Time::delay(2.0);
    //     yInfo()<<"Wait a 2 seconds...";
    // }


    yInfo()<<"Lets transform the point cloud";
    // yInfo()<<"Lets check if the head encoders are read";
    int numHeadJoints = 2;
    if(robot == "/teoSim")
        iHeadEncoders->getAxes(&numHeadJoints);

    std::vector<double> currentHeadQ(numHeadJoints);

    if(robot == "/teoSim"){
        if (!iHeadEncoders->getEncoders(currentHeadQ.data()))
        {
            yError()<<"getEncoders() failed";
            return false;
        }
    }
    else{
        currentHeadQ[0] = 0.0;
        currentHeadQ[1] = -14.0;
    }

    int numTrunkJoints;
    if(!iTrunkEncoders->getAxes(&numTrunkJoints)){
        yError()<<"getAxes() failed";
        return false;
    }
    

    std::vector<double> currentTrunkQ(numTrunkJoints);

    if (!iTrunkEncoders->getEncoders(currentTrunkQ.data()))
    {
        yError()<<"getEncoders() failed";
        return false;
    }
    yInfo()<<"CurrentTrunkQ: "<<currentTrunkQ[0]<<" "<<currentTrunkQ[1];

    /** --------------------------------------------------------------------------- **/

    std::vector<double> currentQ(numTrunkJoints+numHeadJoints);
    for(int i=0; i<numTrunkJoints; i++){
        currentQ[i] = currentTrunkQ[i];
    }
    for(int i=0; i<numHeadJoints; i++){
        currentQ[i+2] =- currentHeadQ[i];
    }
    
    std::vector<double> currentX;
    if(!trunkAndHeadSolverDeviceICartesianSolver->fwdKin(currentQ, currentX)){
        yError() << "fwdKin failed";
        return false;
    }

    // printf("Current head pose wrt trunk:");
    // for (int i = 0; i < 6; i++)
    // {
    //     printf(" %f", currentX[i]);
    // }
    // printf("\n");


    KDL::Frame frame_head_trunk = roboticslab::KdlVectorConverter::vectorToFrame(currentX);
    KDL::Frame frame;

    if(DEFAULT_ROBOT == "/teo"){
        KDL::Frame frame_camera_head;
        frame_camera_head  = frame_camera_head*KDL::Frame(KDL::Vector(0, 0, 0.059742));
        frame_camera_head = frame_camera_head*KDL::Frame(KDL::Vector(0.10, 0, 0.0));
        // frame_camera_head = frame_camera_head*KDL::Frame(KDL::Rotation::RotZ(-M_PI_2));
        // frame_camera_head = frame_camera_head*KDL::Frame(KDL::Rotation::RotX(-M_PI_2));
        frame_camera_head  = frame_camera_head*KDL::Frame(KDL::Vector(0.0, +0.018, 0.0));
        frame = frame_head_trunk*frame_camera_head;

    }
    else if(DEFAULT_ROBOT == "/teoSim"){
        frame = frame_head_trunk;
    }
    

    Eigen::Matrix4f transform = KDLToEigenMatrix(frame);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(yarp::pcl::toPCL<yarp::sig::DataXYZ, pcl::PointXYZ>(pc, *cloud)){

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter(*cloud);

    
        pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
        return true;

    }else{
        yError()<<"Could not transform pointcloud";
        return false;
    }

}
/************************************************************************/

Eigen::Matrix4f GetGraspingPoses::KDLToEigenMatrix(const KDL::Frame &p)
 {
   Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
   for(int i=0; i < 3; i++)
   {
     for(int j=0; j<3; j++)
     {
       b(i,j) = p.M(i,j);
     }
     b(i,3) = p.p(i);
   }
//    std::cout << "Here is the matrix m:\n" << b << std::endl;
   return b;
}
/************************************************************************/
void GetGraspingPoses::rosComputeAndSendPc(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, std::string frame_id)
{
    //yCDebug(LASER_FROM_POINTCLOUD) << "sizeof:" << sizeof(yarp::sig::DataXYZ);

    yarp::rosmsg::sensor_msgs::PointCloud2  rosPC_data;
    static int counter=0;
    rosPC_data.header.stamp.nsec=0;
    rosPC_data.header.stamp.sec=0;
    rosPC_data.header.seq=counter++;
    rosPC_data.header.frame_id = frame_id;

    rosPC_data.fields.resize(3);
    rosPC_data.fields[0].name       = "x";
    rosPC_data.fields[0].offset     = 0;    // offset in bytes from start of each point
    rosPC_data.fields[0].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[0].count      = 1;    // how many FLOAT32 used for 'x'

    rosPC_data.fields[1].name       = "y";
    rosPC_data.fields[1].offset     = 4;    // offset in bytes from start of each point
    rosPC_data.fields[1].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[1].count      = 1;    // how many FLOAT32 used for 'y'

    rosPC_data.fields[2].name       = "z";
    rosPC_data.fields[2].offset     = 8;    // offset in bytes from start of each point
    rosPC_data.fields[2].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[2].count      = 1;    // how many FLOAT32 used for 'z'

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
    rosPC_data.height=pc.height();
    rosPC_data.width=pc.width();
#endif

    rosPC_data.point_step = 3*4; //x, y, z
    rosPC_data.row_step   = rosPC_data.point_step*rosPC_data.width; //12 *number of points bytes
    rosPC_data.is_dense = true;   // what this field actually means?? When is it false??
    rosPC_data.data.resize(rosPC_data.row_step*rosPC_data.height);

    const char* ypointer = pc.getRawData()+12;
    unsigned char* rpointer = rosPC_data.data.data();

    // yInfo()<<pc.size() << pc.size()*4*4 << pc.dataSizeBytes() << rosPC_data.data.size();
    size_t elem =0;
    size_t yelem=0;
    for (; elem<pc.size()*3*4; elem++)
    {
        *rpointer=*ypointer;
        rpointer++;
        ypointer++; yelem++;
        if (elem%12==0) { ypointer+=4; yelem+=4;}
        // yCDebug(LASER_FROM_POINTCLOUD << "%d" <<ypointer;
    }
   //yCDebug(LASER_FROM_POINTCLOUD)<<elem <<yelem;

    if (pointCloud_outTopic) {
        // yInfo("Publish...\n");
        pointCloud_outTopic->write(rosPC_data);
    }
}


bool GetGraspingPoses::read(yarp::os::ConnectionReader &connection)
{
    // viewer =pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("view"));

    yarp::os::Bottle reply;
    auto *writer = connection.getWriter();


    yarp::os::Bottle command;
    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }
    yInfo() << "command:" << command.toString();

    int brx = command.get(0).find("brx").asInt32();
    int bry = command.get(0).find("bry").asInt32();
    int tlx = command.get(0).find("tlx").asInt32();
    int tly = command.get(0).find("tly").asInt32();
    std::string category = command.get(0).find("category").asString();
    printf("category: % s brx: %d bry: %d tlx: %d tly: %d\n", category.c_str(), brx, bry, tlx, tly);
  
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

    bool depth_ok = iRGBDSensor->getDepthImage(depthFrame);
    if (depth_ok == false)
    {
        yError()<< "getDepthImage failed";
        return false;
    }

    if(tlx<0){
        tlx = 0;
    }
    if(tly<0){
        tlx = 0;
    }
    if(brx>=depthFrame.width()){
        brx = depthFrame.width();
    }
    if(bry>=depthFrame.height()){
        bry = depthFrame.height();
    }
    int min_x = tlx;
    int min_y = tly;
    int max_x = brx;
    int max_y = bry;

    yarp::sig::PointCloud<yarp::sig::DataXYZ> pcFiltered;
    pcFiltered.resize(brx-tlx,bry-tly);

    for (size_t i = 0, u = min_x; u < max_x; i++, u ++){
        for (size_t j = 0, v = min_y; v < max_y; j++, v ++) {
            if( depthFrame.pixel(u, v)<1.3){ // TODO: PARAMETER MAX DISTANCE FILTER
                pcFiltered(i, j).x = depthFrame.pixel(u, v); 
                pcFiltered(i, j).y = -(u - intrinsics.principalPointX) / intrinsics.focalLengthX * depthFrame.pixel(u, v);
                pcFiltered(i, j).z = -(v - intrinsics.principalPointY) / intrinsics.focalLengthY * depthFrame.pixel(u, v);
            }
        }
    }



    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
   
    if( !transformPointCloud(pcFiltered, crop_transformed_cloud)){
        yError()<<"Could NOT transform the pointcloud";
        reply.addString("Could not transform the pointcloud");
    }
    else{
    

    pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
  



    reply.addString(category);
    yarp::os::Bottle bGraspingPoses;


    std::vector<std::vector<double>>graspingPoses;

    // yarp::os::Bottle& graspingPoses = pOutGraspingPosesPort->prepare();   // Get the object
    if (category == "water" || category == "nesquick"){
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.005);
        seg.setAxis(Eigen::Vector3f::UnitX());
        seg.setEpsAngle(0.005);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        int i = 0, nr_points = (int) crop_transformed_cloud->size ();

        pcl::PCDWriter writer;
        // while (transformed_cloud->size () > 0.05 * nr_points)
        // {
            printf("Plane %d\n", i);
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (crop_transformed_cloud);
            seg.segment (*inliers, *coefficients);

            printf("Model coefficients: %f %f %f %f\n",coefficients->values[0],coefficients->values[1],coefficients->values[2], coefficients->values[3]);
            double lengthNormal = sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1]*coefficients->values[1]
            +coefficients->values[2]*coefficients->values[2]);
            KDL::Vector n(coefficients->values[0]/lengthNormal, coefficients->values[1]/lengthNormal,coefficients->values[2]/lengthNormal);
            printf("Normal vector to plane: %f %f %f\n", n[0], n[1],n[2]);
            

            // Extract the inliers
            extract.setInputCloud (crop_transformed_cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            printf("PointCloud representing the planar component: %d data points\n",cloud_p->width * cloud_p->height);

            // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler_obj (transformed_cloud_copy, 20, 230, 20); // Red
            // viewer->addPointCloud (crop_transformed_cloud, transformed_cloud_color_handler_obj, "transformed_cloud1");



            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            (crop_transformed_cloud).swap(cloud_f);

            pcl::CentroidPoint<pcl::PointXYZ> centroid;

            for (auto& point: *crop_transformed_cloud)
            {
                centroid.add(point);
            }

            pcl::PointXYZ c1;
            centroid.get(c1);

            std::stringstream ss;
            ss << "water_scene_" << i << ".pcd";
            printf("water_scene_%d\n",i);
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);


            std::vector<pcl::PointXYZ>points;
            std::vector<KDL::Vector>normals;
            if (category == "water"){
                computeGraspingPosesWaterNesquick(c1, normals, points, graspingPoses, waterShape);
            }
            if (category == "nesquick"){

                computeGraspingPosesWaterNesquick(c1, normals, points, graspingPoses, nesquickShape);
            }

            rosComputeGraspingPosesArrowAndSend("map", points, normals);
    }
    else{ // milk and cereals

        std::vector<pcl::PointXYZ>maxPoints;
        std::vector<pcl::PointXYZ>minPoints;
        std::vector<KDL::Vector>normals;
        std::vector<pcl::PointXYZ>centroids;

        pcl::SACSegmentation<pcl::PointXYZ> seg;

        
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (2000);
        seg.setDistanceThreshold (0.005);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        int i = 0, nr_points = (int) crop_transformed_cloud->size ();
        pcl::PCDWriter writer;
        while (crop_transformed_cloud->size () > 0.05 * nr_points)
        {
            printf("Plane %d\n", i);
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (crop_transformed_cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
            printf("Could not estimate a planar model for the given dataset.\n");
            break;
            }

            printf("Model coefficients: %f %f %f %f\n",coefficients->values[0],coefficients->values[1],coefficients->values[2], coefficients->values[3]);
            double lengthNormal = sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1]*coefficients->values[1]
            +coefficients->values[2]*coefficients->values[2]);
            KDL::Vector n(coefficients->values[0]/lengthNormal, coefficients->values[1]/lengthNormal,coefficients->values[2]/lengthNormal);
            printf("Normal vector to plane: %f %f %f\n", n[0], n[1],n[2]);
            

            // Extract the inliers
            extract.setInputCloud (crop_transformed_cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            printf("PointCloud representing the planar component: %d data points\n",cloud_p->width * cloud_p->height);

            pcl::CentroidPoint<pcl::PointXYZ> centroid;

            for (auto& point: *cloud_p)
            {
                centroid.add(point);
            }

            pcl::PointXYZ c1;
            centroid.get (c1);
            printf("Centroid %f %f %f\n", c1.x, c1.y, c1.z);
            std::stringstream ss;
            ss << "milk_scene_" << i << ".pcd";
            printf("milk_scene_%d\n",i);

            writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

            if (fabs(n[2])<0.4){ // RIGHT NOW GRASPING FROM THE TOP IS NOT ALLOWEDDDD!!!

                printf("Valid plane i: %d\n",i);
                pcl::PointXYZ maxPoint;
                pcl::PointXYZ minPoint;
                getMinimumBoundingBoxPointCLoud(cloud_p, maxPoint, minPoint);
                printf("Normal: %f %f %f\n", n[0], n[1], n[2]);
                printf("Max point: %f %f %f, Min Point: %f %f %f\n", maxPoint.x,maxPoint.y, maxPoint.z, minPoint.x, minPoint.y, minPoint.z);
                
                normals.push_back(n);
                maxPoints.push_back(maxPoint);
                minPoints.push_back(minPoint);
                centroids.push_back(c1);
            }
            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            (crop_transformed_cloud).swap (cloud_f);
            i++;
        }
        if(normals.size() == 0){
            printf("No planes detected!\n");
        }
        else{
            printf("One or more Planes are detected!\n");
            if (category == "milk1" || category == "milk2")
            {
                std::vector<KDL::Vector> y_vectors;
                std::vector<KDL::Vector> x_vectors;
                computeGraspingPosesMilk(normals, centroids, maxPoints, minPoints, graspingPoses, y_vectors, x_vectors);
                rosComputeGraspingPosesArrowAndSend("map", centroids, normals);
                // addGraspingPointsAndNormalsToViewer(centroids, normals, viewer);
              
            }
            if(category =="cereals1" || category == "cereals2" || category == "cereals3"){
                computeGraspingPosesCereal(normals, centroids, maxPoints, minPoints, graspingPoses);
                rosComputeGraspingPosesArrowAndSend("map", centroids, normals);
            }
        }

    }


     



    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");


    // while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    //     viewer->spinOnce ();
    // }

    for(int i=0; i<graspingPoses.size(); i++){
        printf("%f %f %f %f %f %f \n", graspingPoses[i][0], graspingPoses[i][1], graspingPoses[i][2], graspingPoses[i][3], graspingPoses[i][4], graspingPoses[i][5]);
        yarp::os::Bottle bPose;
        for(int j = 0; j < 6; j++){
            bPose.addDouble(graspingPoses[i][j]);
        }
        bGraspingPoses.addList() = bPose;
    }
    reply.addList() = bGraspingPoses;
    }

    return reply.write(*writer);
}


void GetGraspingPoses::computeGraspingPosesMilk(std::vector<KDL::Vector> & normals,     std::vector<pcl::PointXYZ> &centroids, 
                                                std::vector<pcl::PointXYZ> & maxPoints, std::vector<pcl::PointXYZ> &minPoints, 
                                                std::vector<std::vector<double>> & graspingPoses,  std::vector<KDL::Vector> & y_vectors, std::vector<KDL::Vector> &x_vectors)
{

    if(normals.size() == 3){
        printf("Three planes detected lest get the others\n");
        if(fabs(normals[0][0])>fabs(normals[1][0])){
            normals.pop_back();
            centroids.pop_back();
        }
        else{
            normals.erase(normals.begin());
            centroids.erase(centroids.begin());
        }
    }

    if(normals.size() == 2){
        printf("Two planes detected lest get the others\n");
        if(fabs(normals[0][0])>fabs(normals[1][0])){
            normals.pop_back();
            centroids.pop_back();
        }
        else{
            normals.erase(normals.begin());
            centroids.erase(centroids.begin());
        }
    }



    if(normals.size() == 1){
        printf("Just one plane detected lest get the others\n");
        KDL::Vector y_vector;

        y_vector[2] = -1.0;
        y_vectors.push_back(y_vector);

        y_vector[2] = 1.0;
        y_vectors.push_back(y_vector);

        normals.push_back(normals[0]);

        x_vectors.push_back(y_vectors[0]*normals[0]);
        x_vectors.push_back(y_vectors[1]*normals[1]);

        centroids.push_back(centroids[0]);



        // Opposite plane
        KDL::Vector auxV1 = normals[0]*milkBoxShape[1];
        pcl::PointXYZ auxCentroid = centroids[0];
        auxCentroid.x += auxV1[0];
        auxCentroid.y += auxV1[1];
        auxCentroid.z += auxV1[2];

        centroids.push_back(auxCentroid);
        centroids.push_back(auxCentroid);
        
        normals.push_back(-normals[0]);
        normals.push_back(-normals[0]);

        y_vector[2] = -1.0;
        y_vectors.push_back(y_vector);

        y_vector[2] = 1.0;
        y_vectors.push_back(y_vector);

        x_vectors.push_back(y_vectors[2]*normals[2]);
        x_vectors.push_back(y_vectors[3]*normals[3]);

        // First |_ plane
        auxCentroid = centroids[0];
        auxV1 = -x_vectors[0]*milkBoxShape[1]/2.0+normals[0]*milkBoxShape[1]/2.0;
        auxCentroid.x += auxV1[0];
        auxCentroid.y += auxV1[1];
        auxCentroid.z += auxV1[2];

        centroids.push_back(auxCentroid);
        centroids.push_back(auxCentroid);

        normals.push_back(x_vectors[0]);
        normals.push_back(x_vectors[0]);

        y_vector[2] = -1.0;
        y_vectors.push_back(y_vector);

        y_vector[2] = 1.0;
        y_vectors.push_back(y_vector);


        x_vectors.push_back(y_vectors[4]*normals[4]);
        x_vectors.push_back(y_vectors[5]*normals[5]);

        //Second |_ plane
        
        auxCentroid = centroids[0];
        auxV1 = x_vectors[0]*milkBoxShape[1]/2.0+normals[0]*milkBoxShape[1]/2.0;
        auxCentroid.x += auxV1[0];
        auxCentroid.y += auxV1[1];
        auxCentroid.z += auxV1[2];

        centroids.push_back(auxCentroid);
        centroids.push_back(auxCentroid);

        normals.push_back(-x_vectors[0]);
        normals.push_back(-x_vectors[0]);

        y_vector[2] = -1.0;
        y_vectors.push_back(y_vector);

        y_vector[2] = 1.0;
        y_vectors.push_back(y_vector);

        x_vectors.push_back(y_vectors[6]*normals[6]);
        x_vectors.push_back(y_vectors[7]*normals[7]);

    }

    int previousLength = normals.size();
    for(int i=0; i<previousLength;i++){
        for(int j=-10; j< 11; j++){
            pcl::PointXYZ p = centroids[i];
            p.z+=float(j)/10*milkBoxShape[2]/4.0;
            centroids.push_back(p);
            normals.push_back(normals[i]);
            y_vectors.push_back(y_vectors[i]);
            x_vectors.push_back(x_vectors[i]);
        }
    }
    for(int i=0; i<normals.size(); i++){
        KDL::Rotation rot = KDL::Rotation(x_vectors[i], y_vectors[i],normals[i]);         
        KDL::Frame frameTCP(rot);
        KDL::Frame frame_goal = frameTCP;

        frame_goal.p[0] = centroids[i].x;
        frame_goal.p[1] = centroids[i].y;
        frame_goal.p[2] = centroids[i].z;
        std::vector<double> tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_goal);
        printf("%f %f %f %f %f %f\n",tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);

        graspingPoses.push_back(tcpX);
    }
    
}       



void GetGraspingPoses::computeGraspingPosesWaterNesquick(pcl::PointXYZ & centroid, std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &points, 
                                                std::vector<std::vector<double>> & graspingPoses, double (&cylinderShape)[2])
{
    std::vector<KDL::Vector> x_vectors;
    std::vector<KDL::Vector> y_vectors;

    // centroid.x += cylinderShape[0]/2.0; 

    int nNormals = 16;
    for(int i = 0; i<nNormals; i++){
        float angle = (i*(-1.5*M_PI)/(nNormals))+2*M_PI/3;
        KDL::Vector normal;
        normal[0] = cos(angle);
        normal[1] = sin(angle);
        normals.push_back(normal);
        normals.push_back(normal);

        KDL::Vector y_vector;
        y_vector[2] = 1.0;
        y_vectors.push_back(y_vector);
        x_vectors.push_back(y_vector*normal);

        y_vector[2] = -1.0;
        y_vectors.push_back(y_vector);
        x_vectors.push_back(y_vector*normal);

        pcl::PointXYZ point;
        KDL::Vector auxV = normal*cylinderShape[0];
        point.x = centroid.x - auxV[0];
        point.y = centroid.y - auxV[1];
        point.z = centroid.z - auxV[2];
        points.push_back(point);
        points.push_back(point);

    }
    nNormals = normals.size();

    for(int i=0; i<nNormals; i++){
        for(int j=-5; j< 6; j++){
            pcl::PointXYZ p = points[i];
            p.z+=float(j)/10*cylinderShape[1]/3.0;
            points.push_back(p);
            normals.push_back(normals[i]);
            y_vectors.push_back(y_vectors[i]);
            x_vectors.push_back(x_vectors[i]);
        }
    }

    for(int i=0; i<normals.size(); i++){
        KDL::Rotation rot = KDL::Rotation(x_vectors[i], y_vectors[i],normals[i]);         
        KDL::Frame frameTCP(rot);
        KDL::Frame frame_goal = frameTCP;

        frame_goal.p[0] = points[i].x;
        frame_goal.p[1] = points[i].y;
        frame_goal.p[2] = points[i].z;
        std::vector<double> tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_goal);
        printf("%f %f %f %f %f %f\n",tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);

        graspingPoses.push_back(tcpX);
    }




}

void GetGraspingPoses::computeGraspingPosesCereal(std::vector<KDL::Vector> & normals, std::vector<pcl::PointXYZ> &centroids, 
                                                std::vector<pcl::PointXYZ> & maxPoints, std::vector<pcl::PointXYZ> &minPoints, 
                                                std::vector<std::vector<double>> & graspingPoses){
    
    std::vector<KDL::Vector> y_vectors;
    std::vector<KDL::Vector> x_vectors;

    if(normals.size() == 3){
        printf("Three planes detected lest get the others\n");
        if(fabs(normals[0][0])>fabs(normals[1][0])){
            normals.pop_back();
            centroids.pop_back();
            maxPoints.pop_back();
            minPoints.pop_back();
        }
        else{
            normals.erase(normals.begin());
            centroids.erase(centroids.begin());
            maxPoints.erase(maxPoints.begin());
            minPoints.erase(minPoints.begin());
        }
    }

    if(normals.size() == 2){
        printf("Two planes detected lest get the others\n");
        if(fabs(normals[0][0])>fabs(normals[1][0])){
            normals.pop_back();
            centroids.pop_back();
            maxPoints.pop_back();
            minPoints.pop_back();            
        }
        else{
            normals.erase(normals.begin());
            centroids.erase(centroids.begin());
            maxPoints.erase(maxPoints.begin());
            minPoints.erase(minPoints.begin());
        }
    }
    if(normals.size() == 1){
        yInfo()<<"Max points: "<< maxPoints[0].x << maxPoints[0].y<<  maxPoints[0].z;
        yInfo()<<"Min points: "<< minPoints[0].x << minPoints[0].y<<  minPoints[0].z;
        float cerealSizes[3] = {maxPoints[0].x-minPoints[0].x, maxPoints[0].y-minPoints[0].y, maxPoints[0].z-minPoints[0].z};
        yInfo()<<"Cereal size: "<<cerealSizes[0] <<cerealSizes[1]<<cerealSizes[2];

        float marginShape = 0.07;
        KDL::Vector normal;


        KDL::Vector y_vector;
        y_vector[2] = -1.0;
        if( cerealSizes[1]>= (cerealBoxShape[0]+marginShape)){
            
            yInfo()<<"Front-wide position";
            KDL::Vector x_vector = y_vector*normals[0];

            KDL::Vector auxV1 = normals[0]*cerealBoxShape[0]/2;
            yInfo()<<centroids[0].x<<centroids[0].y<<centroids[0].z;
            centroids[0].x += auxV1[0];
            centroids[0].y += auxV1[1];
            centroids[0].z += auxV1[2];


            normals[0] = x_vector;
            auxV1 = x_vector*cerealBoxShape[1]/2;
            centroids[0].x -= auxV1[0];
            centroids[0].y -= auxV1[1];
            centroids[0].z -= auxV1[2];
            yInfo()<<centroids[0].x<<centroids[0].y<<centroids[0].z;
            y_vectors.push_back(y_vector);
            x_vectors.push_back(y_vector*normals[0]);

        }
        else if((cerealSizes[1]>= (cerealBoxShape[0]-marginShape)) &&  (cerealSizes[1]<= (cerealBoxShape[0]+marginShape))){
            yInfo()<<"Front narrow position";
            y_vectors.push_back(y_vector);
            x_vectors.push_back(y_vector*normals[0]);
            // int nGraspingPoints = 20;
            // KDL::Vector y_vector;
            // normal = normals[0];
            // p = centroids[0];
            // centroids.pop_back();
            // normals.pop_back();
            // y_vector[2] = 1.0;

        }
        int nGraspingPoints = 20;

        for(int i=-nGraspingPoints/2; i<nGraspingPoints/2; i++){
            pcl::PointXYZ p = centroids[0]; 
            p.z+=float(i)/nGraspingPoints*cerealBoxShape[2]/2.5;
            centroids.push_back(p);
            normals.push_back(normals[0]);
            y_vectors.push_back(y_vector);
            x_vectors.push_back(y_vector*normals[0]);


            KDL::Vector auxV1 = normals[0]*cerealBoxShape[1];
                
            p.x += auxV1[0];
            p.y += auxV1[1];
            p.z += auxV1[2];

            centroids.push_back(p);
            normals.push_back(-normals[0]);

            y_vectors.push_back(y_vector);
            x_vectors.push_back(y_vector*-normals[0]);

        }
        int nNormals = normals.size();
        y_vector[2] = 1.0;
        for(int i=0; i<nNormals; i++){
            centroids.push_back(centroids[i]);
            normals.push_back(normals[i]);
            y_vectors.push_back(y_vector);
            x_vectors.push_back(y_vector*normals[i]);
        }

        for(int i=0; i<normals.size(); i++){
            KDL::Rotation rot = KDL::Rotation(x_vectors[i], y_vectors[i],normals[i]);         
            KDL::Frame frameTCP(rot);
            KDL::Frame frame_goal = frameTCP;

            frame_goal.p[0] = centroids[i].x;
            frame_goal.p[1] = centroids[i].y;
            frame_goal.p[2] = centroids[i].z;
            std::vector<double> tcpX = roboticslab::KdlVectorConverter::frameToVector(frame_goal);
            printf("%f %f %f %f %f %f\n",tcpX[0], tcpX[1], tcpX[2], tcpX[3], tcpX[4], tcpX[5]);

            graspingPoses.push_back(tcpX);
        }

    }

                                                

}


void GetGraspingPoses::getMinimumBoundingBoxPointCLoud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, pcl::PointXYZ & maxPoint, pcl::PointXYZ & minPoint){
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    // pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();



}


void GetGraspingPoses::rosComputeGraspingPosesArrowAndSend(const std::string &frame_id, std::vector<pcl::PointXYZ> &centroids, std::vector<KDL::Vector> & normals){
    yarp::rosmsg::visualization_msgs::Marker marker;
    yarp::rosmsg::visualization_msgs::MarkerArray markerArray;


    marker.header.frame_id = frame_id;
    static int counter=0;
    marker.header.stamp.nsec=0;
    marker.header.stamp.sec=0;

    marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);

     if (graspingPoses_outTopic) {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }



    for (int i=0; i<centroids.size(); i++)
    {
        // printf("Normals %d: %f %f %f\n", i, normals[0], normals[1], normals[2]);
        float lengthArrow = 0.1;
        KDL::Vector auxV1 = normals[i]*lengthArrow;
        marker.header.seq=counter++;
        marker.id = i;
        marker.type = yarp::rosmsg::visualization_msgs::Marker::ARROW;
        marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        yarp::rosmsg::geometry_msgs::Point pointRos;
        pointRos.x = centroids[i].x-auxV1[0];
        pointRos.y = centroids[i].y-auxV1[1];
        pointRos.z = centroids[i].z-auxV1[2];
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

    if (graspingPoses_outTopic) {
        yInfo("Publish...\n");
        graspingPoses_outTopic->write(markerArray);
    }

}