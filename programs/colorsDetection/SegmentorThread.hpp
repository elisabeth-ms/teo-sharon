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

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/text.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <sstream>

#include "TravisLib.hpp"

#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#define DEFAULT_RATE_MS 20

using namespace std;
using namespace cv;

namespace sharon {



class SegmentorThread : public yarp::os::PeriodicThread
{
public:
    SegmentorThread() : PeriodicThread(DEFAULT_RATE_MS * 0.001) {}

    void setIRGBDSensor(yarp::dev::IRGBDSensor * _iRGBDSensor);
    void setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg);
    void setOutPort(yarp::os::Port *_pOutPort);
    bool init(yarp::os::ResourceFinder &rf);

    void setCropSelector(int cropSelector) { this->cropSelector = cropSelector; }
    void setOutCropSelectorImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg) { this->outCropSelectorImg = outCropSelectorImg; }
    void setInCropSelectorPort(yarp::os::Port* inCropSelectorPort) { this->inCropSelectorPort = inCropSelectorPort; }
    void depthFilter(Mat &image, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth,float maxDistance);
    bool findCup(const Mat& origImage,const Mat& image, vector<Point>& points);
    bool findMilkBottle(const Mat& origImage,const Mat& image, vector<Point>& points);
    int  countWindowPixels(Mat & image, int roiX, int roiY, int roiWidth, int roiHeight, float * xAvg, float * yAvg);

private:
    void run() override; // The periodical function

    yarp::dev::PolyDriver detectorDevice;
    //IDetector* iDetector;

    yarp::dev::IRGBDSensor *iRGBDSensor;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutImg;  // for testing
    yarp::os::Port *pOutPort;
    //
    double fx_d,fy_d,cx_d,cy_d;
    //
    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg;
    yarp::os::Port* inCropSelectorPort;
    //DataProcessor processor;

    std::string algorithm;
    int maxNumBlobs;
    double morphClosing;
    double morphOpening;
    int outFeaturesFormat;
    int outImage;
    int seeBounding;
    int threshold;

    static const std::string DEFAULT_ALGORITHM;
    static const double DEFAULT_MORPH_CLOSING;
    static const double DEFAULT_THRESHOLD;
    static const double DEFAULT_MAX_NUM_BLOBS;
};
}
#endif //__SEGMENTOR_THREAD_HPP__
