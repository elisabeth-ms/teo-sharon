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
#include <yarp/cv/Cv.h>

#include <fstream>
#include <random>

#define DEFAULT_RATE_MS 20
#define BACKGROUND_CROP false
using namespace std;
using namespace cv;

namespace sharon {



class SegmentorThread : public yarp::os::PeriodicThread
{
public:
    SegmentorThread() : PeriodicThread(DEFAULT_RATE_MS*0.001) {}

    void setIRGBDSensor(yarp::dev::IRGBDSensor * _iRGBDSensor);
    void setOutRgbImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutRgbImgPort);
    void setOutDepthImgPort(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > * _pOutDepthImgPort);
    void setFileNamePort(yarp::os::BufferedPort<yarp::os::Bottle>* _pInFileNamePort);

    void setOutPort(yarp::os::Port *_pOutPort);
    bool init(yarp::os::ResourceFinder &rf);
    Point getCentroid(InputArray Points);

    void setCropSelector(int cropSelector) { this->cropSelector = cropSelector; }
    void setOutCropSelectorImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* outCropSelectorImg) { this->outCropSelectorImg = outCropSelectorImg; }
    void setInCropSelectorPort(yarp::os::Port* inCropSelectorPort) { this->inCropSelectorPort = inCropSelectorPort; }
    void depthFilter(Mat &image, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth,float maxDistance);
    vector<float> findCup(const Mat& origImage,const Mat& image, vector<Point>& points);
    vector<float> cropSalientObject(Mat& origImage,const Mat& image, const Mat &depthImage, Mat & croppedImage, Mat & croppedDepthImage, int &bx, int &by);
    int  countWindowPixels(Mat & image, int roiX, int roiY, int roiWidth, int roiHeight, float * xAvg, float * yAvg, vector<float>& yvector);
    bool storeRgbDepthImages(yarp::sig::ImageOf<yarp::sig::PixelRgb> rgb,yarp::sig::ImageOf<yarp::sig::PixelRgb> cropRgb, yarp::sig::ImageOf<yarp::sig::PixelFloat> depth,yarp::sig::ImageOf<yarp::sig::PixelFloat> cropDepth, string strRgbFileName);
    vector<float> getBoundingBox(int origImageWidth, int origImageHeitgh, int bxRoi, int byRoi, const Mat& maskDepth);
    bool createLabelsFile(string strFileName, vector<float> boundingBox, int label, yarp::sig::ImageOf<yarp::sig::PixelRgb> rgb, yarp::sig::ImageOf<yarp::sig::PixelFloat> depth);
    vector<float> findGlass(const Mat& origImage,const Mat& image, const Mat &depthImage);
    vector<float> findCereal(const Mat& origImage,const Mat& image, const Mat &depthImage);

private:
    void run() override; // The periodical function

    yarp::dev::PolyDriver detectorDevice;
    //IDetector* iDetector;

    yarp::dev::IRGBDSensor *iRGBDSensor;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *pOutRgbImgPort;  // Port for sending the cropped image of the rgb frames
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > *pOutDepthImgPort;  // Port for sending the cropped image of the depth frames
    yarp::os::BufferedPort<yarp::os::Bottle> *pInFileNamePort;
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
