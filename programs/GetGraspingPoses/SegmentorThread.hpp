// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SEGMENTOR_THREAD_HPP__
#define __SEGMENTOR_THREAD_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>

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
#include <pcl/visualization/pcl_visualizer.h>


#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"
#include <KdlVectorConverter.hpp>

#include <fstream>
#include <random>
#include <math.h> 
#define DEFAULT_RATE_MS 20
#define BACKGROUND_CROP false
using namespace std;
using namespace cv;
using namespace roboticslab::KinRepresentation;
using namespace roboticslab::KdlVectorConverter;
using namespace roboticslab;


namespace sharon {



class SegmentorThread : public yarp::os::PeriodicThread
{
public:
    SegmentorThread() : PeriodicThread(DEFAULT_RATE_MS*0.001) {}

    void setIRGBDSensor(yarp::dev::IRGBDSensor * _iRGBDSensor);
    void setOutRgbImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutRgbImgPort);
    void setOutDepthImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > * _pOutDepthImgPort);
    void setInMarchingObjectDataPort(yarp::os::BufferedPort<yarp::os::Bottle>* _pInMarchingObjectDataPort);
    void setOutPointCloudPort(yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZ>> * _pOutPointCloudPort);
    bool init(yarp::os::ResourceFinder &rf);
    void setHeadIEncoders(yarp::dev::IEncoders *_iEncoders);
    void setTrunkIEncoders(yarp::dev::IEncoders *_iEncoders);
    void setICartesianSolver( ICartesianSolver * _iCartesianSolver);
    Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);

    

private:
    void run() override; // The periodical function

    yarp::dev::PolyDriver detectorDevice;
    yarp::sig::IntrinsicParams intrinsics;

    yarp::dev::IEncoders *iHeadEncoders;
    yarp::dev::IEncoders *iTrunkEncoders;

    ICartesianSolver *trunkAndHeadICartesianSolver;

    yarp::dev::IRGBDSensor *iRGBDSensor;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutRgbImgPort;  // Port for sending the cropped image of the rgb frames
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > *pOutDepthImgPort;  // Port for sending the cropped image of the depth frames
    yarp::os::BufferedPort<yarp::os::Bottle> *pInMarchingObjectDataPort;
    yarp::os::Port *pOutPort;
    yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZ>> *pOutPointCloudPort;


};
}
#endif //__SEGMENTOR_THREAD_HPP__
