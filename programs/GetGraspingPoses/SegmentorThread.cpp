// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"
#include <opencv2/saliency.hpp>

using namespace saliency;
using namespace std;
bool less_by_x(const cv::Point& lhs, const cv::Point& rhs)
{
  return lhs.x < rhs.x;
}

bool less_by_y(const cv::Point& lhs, const cv::Point& rhs)
{
  return lhs.y < rhs.y;
}

// C++ program to find equation of a plane
// passing through given 3 points.
#include <bits/stdc++.h>
#include<math.h>
#include <iostream>
#include <iomanip>
 
using namespace std;
 

namespace sharon{


/************************************************************************/

void SegmentorThread::setIRGBDSensor(yarp::dev::IRGBDSensor *_iRGBDSensor)
{
    iRGBDSensor = _iRGBDSensor;
}

/************************************************************************/

void SegmentorThread::setOutRgbImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutRgbImgPort)
{
    pOutRgbImgPort = _pOutRgbImgPort;
}

/************************************************************************/

void SegmentorThread::setOutDepthImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > * _pOutDepthImgPort)
{
    pOutDepthImgPort = _pOutDepthImgPort;
}

/************************************************************************/

void SegmentorThread::setInMarchingObjectDataPort(yarp::os::BufferedPort<yarp::os::Bottle>* _pInMarchingObjectDataPort)
{
    pInMarchingObjectDataPort = _pInMarchingObjectDataPort;
}
/************************************************************************/


void SegmentorThread::setOutPointCloudPort(yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZ>> * _pOutPointCloudPort){
    pOutPointCloudPort = _pOutPointCloudPort;
}
/************************************************************************/

void SegmentorThread::setHeadIEncoders(yarp::dev::IEncoders *_iEncoders){
    iHeadEncoders = _iEncoders;
}
/************************************************************************/

void SegmentorThread::setTrunkIEncoders(yarp::dev::IEncoders *_iEncoders){
    iTrunkEncoders = _iEncoders;
}

/************************************************************************/
void SegmentorThread::setICartesianSolver( ICartesianSolver * _iCartesianSolver){
    trunkAndHeadICartesianSolver = _iCartesianSolver;
}

Eigen::Matrix4f SegmentorThread::KDLToEigenMatrix(const KDL::Frame &p)
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
   std::cout << "Here is the matrix m:\n" << b << std::endl;
   return b;
}

bool SegmentorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;

    yarp::os::Property depthIntrinsicParams;

    if(!iRGBDSensor->getDepthIntrinsicParam(depthIntrinsicParams))
    {
        yError("\n");
        return false;
    }

    // fx_d = depthIntrinsicParams.find("focalLengthX").asFloat64();
    // fy_d = depthIntrinsicParams.find("focalLengthY").asFloat64();
    // cx_d = depthIntrinsicParams.find("principalPointX").asFloat64();
    // cy_d = depthIntrinsicParams.find("principalPointY").asFloat64();

    intrinsics.fromProperty(depthIntrinsicParams);

    // yInfo("SegmentorThread options:\n");
    // yInfo("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    // yInfo("\t--rateMs (default: \"%d\")\n",rateMs);
    // yInfo("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n", fx_d,fy_d,cx_d,cy_d);

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt32();
    }


     if(!setPeriod(rateMs * 0.001))
     {
         yError("\n");
         return false;
     }


     if(!start())
     {
         yError("\n");
         return false;
     }

     return true;
}




/************************************************************************/
void SegmentorThread::run()
{
    yInfo()<<"run";

    yarp::os::Bottle* objectData = pInMarchingObjectDataPort->read();
    printf("pInMarchingObjectDataPort: %s\n",objectData->toString().c_str());
    int brx = objectData->get(0).find("brx").asInt32();
    int bry = objectData->get(0).find("bry").asInt32();
    int tlx = objectData->get(0).find("tlx").asInt32();
    int tly = objectData->get(0).find("tly").asInt32();
    printf("brx: %d bry: %d tlx: %d tly: %d\n", brx, bry, tlx, tly);

  
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

    bool depth_ok = iRGBDSensor->getDepthImage(depthFrame);
    if (depth_ok == false)
    {
        yError()<< "getDepthImage failed";
        return;
    }

    int min_x = tlx;
    int min_y = tly;
    int max_x = brx;
    int max_y = bry;

    // yarp::sig::PointCloud<yarp::sig::DataXYZ> pcFiltered  = yarp::sig::utils::depthToPC(depthFrame, intrinsics);
    yarp::sig::PointCloud<yarp::sig::DataXYZ> pcFiltered;
    pcFiltered.resize(brx-tlx,bry-tly);

    for (size_t i = 0, u = min_x; u < max_x; i++, u ++){
        for (size_t j = 0, v = min_y; v < max_y; j++, v ++) {
            if( depthFrame.pixel(u, v)<1.4){ // TODO: PARAMETER MAX DISTANCE FILTER
                pcFiltered(i, j).x = depthFrame.pixel(u, v); 
                pcFiltered(i, j).y = -(u - intrinsics.principalPointX) / intrinsics.focalLengthX * depthFrame.pixel(u, v);
                pcFiltered(i, j).z = -(v - intrinsics.principalPointY) / intrinsics.focalLengthY * depthFrame.pixel(u, v);
            }
        }
    }

    printf("Lets check if the head encoders are read");
    int numHeadJoints;
    iHeadEncoders->getAxes(&numHeadJoints);
    printf("head joints: %d\n",numHeadJoints);

    std::vector<double> currentHeadQ(numHeadJoints);

    if (!iHeadEncoders->getEncoders(currentHeadQ.data()))
    {
        printf("getEncoders() failed)\n");
        return;
    }
    printf("Current head encs: %f %f\n", currentHeadQ[0],currentHeadQ[1]);


    printf("Lets check if the trunk encoders are read");
    int numTrunkJoints;
    iTrunkEncoders->getAxes(&numTrunkJoints);
    printf("Trunk joints: %d\n",numTrunkJoints);

    std::vector<double> currentTrunkQ(numTrunkJoints);

    if (!iTrunkEncoders->getEncoders(currentTrunkQ.data()))
    {
        printf("getEncoders() failed)\n");
        return;
    }
    printf("Current Trunk encs: %f %f\n", currentTrunkQ[0],currentTrunkQ[1]);

    std::vector<double> currentQ(numTrunkJoints+numHeadJoints);
    for(int i=0; i<numTrunkJoints; i++){
        currentQ[i] = currentTrunkQ[i];
    }
    for(int i=0; i<numHeadJoints; i++){
        currentQ[i+2] = currentHeadQ[i];
    }
    
    std::vector<double> currentX;
    if(!trunkAndHeadICartesianSolver->fwdKin(currentQ, currentX)){
        yError() << "fwdKin failed";
        return;
    }

    printf("Current head pose wrt trunk:");
    for (int i = 0; i < 6; i++)
    {
        printf(" %f", currentX[i]);
    }
    printf("\n");



    
    KDL::Frame frame = vectorToFrame(currentX);

    Eigen::Matrix4f transform = KDLToEigenMatrix(frame);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    yarp::pcl::toPCL<yarp::sig::DataXYZ, pcl::PointXYZ>(pcFiltered, *cloud);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);


    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    // Visualization
    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
     viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }

    yarp::sig::PointCloud<yarp::sig::DataXYZ> yarpCloud;
    yarp::pcl::fromPCL<pcl::PointXYZ, yarp::sig::DataXYZ>(*transformed_cloud, yarpCloud);




    pOutPointCloudPort->prepare() = yarpCloud;


    pOutPointCloudPort->write();

    const string filename("pcl.pcd");

    int result = yarp::pcl::savePCD< yarp::sig::DataXYZ, pcl::PointXYZ>(filename, yarpCloud);

    const string filename1("pclNotTransformed.pcd");

    result = yarp::pcl::savePCD< yarp::sig::DataXYZ, pcl::PointXYZ>(filename1, pcFiltered);

    pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.005);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) transformed_cloud->size ();
    // While 30% of the original cloud is still there
    pcl::PCDWriter writer;
    while (transformed_cloud->size () > 0.05 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (transformed_cloud);
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
        extract.setInputCloud (transformed_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        printf("PointCloud representing the planar component: %d data points\n",cloud_p->width * cloud_p->height);

        pcl::CentroidPoint<pcl::PointXYZ> centroid;

        for (auto& point: *transformed_cloud)
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

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        (transformed_cloud).swap (cloud_f);
        i++;

        if (n[2]<0.8){
            printf("Valid plane i: %d\n",i);
            KDL::Vector up_vector(1.0, 0.0, 0.0);
            KDL::Vector right_vector = n*up_vector;
            KDL::Rotation rot = KDL::Rotation::Quaternion(right_vector[0], right_vector[1], right_vector[2],-1.0*acos(dot(n,up_vector)));
            KDL::Vector center(c1.x, c1.y, c1.z);
            // frame_goal.M.DoRotY(M_PI/2.0);
            // frame_goal.M.DoRotZ(-M_PI/2.0);
            
            KDL::Frame frameTCP(rot);
            KDL::Frame frame_goal = frameTCP;
            frame_goal.M.DoRotY(M_PI/2.0);
            frame_goal.M.DoRotZ(-M_PI/2.0);
            
            frame_goal.p[0] = c1.x;
            frame_goal.p[1] = c1.y;
            frame_goal.p[2] = c1.z;
            std::vector<double> tcpX = frameToVector(frame_goal);

            printf("TCP pose:");
            for (int i = 0; i < 6; i++)
            {
                printf(" %f", tcpX[i]);
            }
            printf("\n");
          
        }
    }


  


    // yarp::os::Bottle * bFileName;
    // bFileName = pInFileNamePort->read();
    // yInfo()<<bFileName->toString();


    Mat depthImage = yarp::cv::toCvMat<yarp::sig::PixelFloat>(depthFrame);

  
    // imshow("test",inCvMat);
    // waitKey(0);

    
    Mat cloneCroppedDepthImage = depthImage.clone();

    yarp::sig::ImageOf<yarp::sig::PixelFloat> outCroppedDepthImage = yarp::cv::fromCvMat<yarp::sig::PixelFloat>(cloneCroppedDepthImage);




    // Lets prepare the output cropped image
    yInfo()<<"Sending the cropped Rgb and depth Images through the yarp ports";

    pOutDepthImgPort->prepare() = outCroppedDepthImage;
    pOutDepthImgPort->write();

}


}

