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
namespace sharon{

const std::string SegmentorThread::DEFAULT_ALGORITHM = "blueMinusRed";
const double SegmentorThread::DEFAULT_MORPH_CLOSING = 2;
const double SegmentorThread::DEFAULT_THRESHOLD = 55;
const double SegmentorThread::DEFAULT_MAX_NUM_BLOBS = 1;
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

void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/

void SegmentorThread::setFileNamePort(yarp::os::BufferedPort<yarp::os::Bottle>* _pInFileNamePort)
{
    pInFileNamePort = _pInFileNamePort;
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

    fx_d = depthIntrinsicParams.find("focalLengthX").asFloat64();
    fy_d = depthIntrinsicParams.find("focalLengthY").asFloat64();
    cx_d = depthIntrinsicParams.find("principalPointX").asFloat64();
    cy_d = depthIntrinsicParams.find("principalPointY").asFloat64();

    yInfo("SegmentorThread options:\n");
    yInfo("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    yInfo("\t--rateMs (default: \"%d\")\n",rateMs);
    yInfo("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n", fx_d,fy_d,cx_d,cy_d);

    if (rf.check("rateMs"))
    {
        rateMs = rf.find("rateMs").asInt32();
    }

     algorithm = DEFAULT_ALGORITHM;

     if(rf.check("algorithm"))
     {
         yInfo("\"algorithm\" parameter found\n");
         algorithm = rf.find("algorithm").asString();
     }
     yInfo("Using \"algorithm\": %s.\n", algorithm.c_str());

     morphClosing = DEFAULT_MORPH_CLOSING;
     if(rf.check("morphClosing"))
     {
         yInfo("\"morphClosing\" parameter found\n");
         morphClosing = rf.find("morphClosing").asFloat64();
     }
     yInfo("Using \"morphClosing\": %f.\n", morphClosing);

     threshold = DEFAULT_THRESHOLD;
     if(rf.check("threshold"))
     {
         yInfo("\"threshold\" parameter found\n");
         threshold = rf.find("threshold").asInt32();
     }
     yInfo("Using \"threshold\": %d.\n", threshold);

     maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
     if(rf.check("maxNumBlobs"))
     {
         yInfo("\"maxNumBlobs\" parameter found\n");
         maxNumBlobs = rf.find("maxNumBlobs").asInt32();
     }
     yInfo("Using \"maxNumBlobs\": %d.\n", maxNumBlobs);

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

bool SegmentorThread::createLabelsFile(string strFileName, vector<float> boundingBox, int label, yarp::sig::ImageOf<yarp::sig::PixelRgb> rgb, yarp::sig::ImageOf<yarp::sig::PixelFloat> depth){
    yInfo()<<strFileName;
    std::string labelsDirectory =  "/home/elisabeth/data/testCrop/labels/";
    string strFileNoExt = strFileName.substr(1,strFileName.length()-6);
    std::string strLabelFile;
    strLabelFile.insert(0,labelsDirectory);
    strLabelFile.append(strFileNoExt);
    strLabelFile.append(".txt");

    std::ofstream labelFile (strLabelFile);
    labelFile<<label<<" "<<boundingBox[0]<<" "<<boundingBox[1]<<" "<<boundingBox[2]<<" "<<boundingBox[3]<<std::endl;

    std::string rgbDirectory =  "/home/elisabeth/data/testCrop/images/";
    std::string strFileRgb;
    strFileRgb.insert(0,rgbDirectory);
    strFileRgb.append(strFileNoExt);
    strFileRgb.append(".ppm");
    yInfo()<<strFileRgb;

    // Check if the file already exists
    ifstream ifile;
    ifile.open(strFileRgb);
    if(ifile) {
       yInfo()<<"File already exists";
       return false;
    }


    yarp::sig::file::write(rgb,strFileRgb,yarp::sig::file::FORMAT_PPM);

    std::string strFloatFileName;
    std::string depthDirectory = "/home/elisabeth/data/testCrop/depth/";
    strFloatFileName.insert(0,depthDirectory);
    strFloatFileName.append(strFileNoExt);
    strFloatFileName.append(".float");
    yarp::sig::file::write(depth,strFloatFileName,yarp::sig::file::FORMAT_NUMERIC);
}


/************************************************************************/
void SegmentorThread::run()
{
    yInfo()<<"run";
    yarp::sig::FlexImage colorFrame;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

    if(!iRGBDSensor->getImages(colorFrame, depthFrame))
        return;

    yarp::os::Bottle * bFileName;
    bFileName = pInFileNamePort->read();
    yInfo()<<bFileName->toString();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;
    inYarpImg.copy(colorFrame);

//    if (depth.height()<10) {
//        yDebug()<<"No depth image yet";
//        return;
//    };
    //printf("width: %d, height: %d\n", colorFrame.width(), colorFrame.height());
    IplImage *inIplImage = cvCreateImage(cvSize(colorFrame.width(), colorFrame.height()),
                                            IPL_DEPTH_8U, 3 );

    cvCvtColor((IplImage*)colorFrame.getIplImage(), inIplImage, cv::COLOR_RGB2BGR);
    //Mat inCvMat(inIplImage);
    Mat inCvMat = cv::cvarrToMat(inIplImage);
    Mat origCvMat = inCvMat.clone();

    Mat copyOrigCvMat = origCvMat.clone();


    yarp::sig::ImageOf<yarp::sig::PixelRgb> rgbImage = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(copyOrigCvMat);

    float maxDistance = 1.1;
    depthFilter(inCvMat,depthFrame, maxDistance);


    Mat depthImage = yarp::cv::toCvMat<yarp::sig::PixelFloat>(depthFrame);
    float maxZ=0;
    float minZ = maxDistance;
    Mat cloneDepthImage = depthImage.clone();
    for(int x = 0;x<cloneDepthImage.rows;x++){
        for(int y=0;y<cloneDepthImage.cols;y++){
            float pix =  cloneDepthImage.at<float>(x, y);
            if(pix>maxZ){
                maxZ = pix;
            }
            if(pix<minZ){
                minZ = pix;
            }
        }
    }
    yInfo()<<"Mat Max: "<<maxZ<<" Min: "<<minZ;
    imshow("test",inCvMat);
    waitKey(0);

    vector<float> boundingBox;

    Mat croppedImage, croppedDepthImage;
    bool correct;
    vector<Point> points;
    //boundingBox = findCup(origCvMat,inCvMat, points);
    yInfo()<<"Crop salient object";

    //cout<<boundingBox<<endl;
    //cv::Rect roiBoundingBox(boundingBox[0]*origCvMat.cols-boundingBox[2]*origCvMat.cols/2,boundingBox[1]*origCvMat.rows - boundingBox[3]*origCvMat.rows/2,boundingBox[2]*origCvMat.cols,boundingBox[3]*origCvMat.rows);

    //rectangle(origCvMat, roiBoundingBox, 0, 2);
    //imshow("bounding box", origCvMat);
//    waitKey(0);

    int bx=0, by=0;
    //boundingBox = cropSalientObject(origCvMat,inCvMat,depthImage,croppedImage, croppedDepthImage, bx, by);

    // boundingBox = findGlass(origCvMat,inCvMat,depthImage);
    boundingBox = findCereal(origCvMat,inCvMat,depthImage);

    Mat cloneMatCroppedImage;
    Mat test;
    cvtColor(origCvMat, cloneMatCroppedImage, CV_BGR2RGB);
    Mat cloneCroppedDepthImage = croppedDepthImage.clone();

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outCroppedRgbImage = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cloneMatCroppedImage);
    yarp::sig::ImageOf<yarp::sig::PixelFloat> outCroppedDepthImage = yarp::cv::fromCvMat<yarp::sig::PixelFloat>(cloneCroppedDepthImage);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> storeCroppedRgb = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cloneMatCroppedImage);

    storeRgbDepthImages(rgbImage,storeCroppedRgb,depthFrame,outCroppedDepthImage,bFileName->toString());
    //outCroppedDepthImage.setPixelSize(outCroppedDepthImage.getPixelSize());

    createLabelsFile(bFileName->toString(), boundingBox, 3, rgbImage, depthFrame);



    // Lets prepare the output cropped image
    yInfo()<<"Sending the cropped Rgb and depth Images through the yarp ports";
    pOutRgbImgPort->prepare() = outCroppedRgbImage;
    pOutRgbImgPort->write();

    pOutDepthImgPort->prepare() = outCroppedDepthImage;
    pOutDepthImgPort->write();

}

bool SegmentorThread::storeRgbDepthImages(yarp::sig::ImageOf<yarp::sig::PixelRgb> rgb,yarp::sig::ImageOf<yarp::sig::PixelRgb> cropRgb,yarp::sig::ImageOf<yarp::sig::PixelFloat> depth,yarp::sig::ImageOf<yarp::sig::PixelFloat> cropDepth, string strRgbFileName){
    //TODO! yarp write
    yInfo()<<strRgbFileName;
    std::string rgbDirectory =  "/home/elisabeth/data/testCrop/rgb/";
    string strFileRgbNoExt = strRgbFileName.substr(1,strRgbFileName.length()-6);

    std::string strFileRgb;
    strFileRgb.insert(0,rgbDirectory);
    strFileRgb.append(strFileRgbNoExt);
    strFileRgb.append(".ppm");
    yInfo()<<strFileRgb;

    // Check if the file already exists
    ifstream ifile;
    ifile.open(strFileRgb);
    if(ifile) {
       yInfo()<<"File already exists";
       return false;
    }

    yarp::sig::file::write(rgb,strFileRgb,yarp::sig::file::FORMAT_PPM);

    std::string cropRgbDirectory = "/home/elisabeth/data/testCrop/crop-rgb/";
    std::string strCropRgbFile;
    strCropRgbFile.insert(0,cropRgbDirectory);
    strCropRgbFile.append(strFileRgbNoExt);
    strCropRgbFile.append(".ppm");
    yarp::sig::file::write(cropRgb,strCropRgbFile,yarp::sig::file::FORMAT_PPM);

    std::string strFloatFileName;
    std::string depthDirectory = "/home/elisabeth/data/testCrop/depth/";
    strFloatFileName.insert(0,depthDirectory);
    strFloatFileName.append(strFileRgbNoExt);
    strFloatFileName.append(".float");
    yarp::sig::file::write(depth,strFloatFileName,yarp::sig::file::FORMAT_NUMERIC);


    std::string strCropFloatFileName;
    std::string cropDepthDirectory = "/home/elisabeth/data/testCrop/crop-depth/";
    strCropFloatFileName.insert(0,cropDepthDirectory);
    strCropFloatFileName.append(strFileRgbNoExt);
    strCropFloatFileName.append(".float");
    yarp::sig::file::write(cropDepth,strCropFloatFileName,yarp::sig::file::FORMAT_NUMERIC);

    return true;

}


vector<float> SegmentorThread::getBoundingBox(int origImageWidth, int origImageHeitgh, int bxRoi, int byRoi, const Mat& maskDepth){
    cout<<"Lets get the bounding box"<<endl;

    vector<Point> locations;   // output, locations of non-zero pixels
    cv::findNonZero(maskDepth, locations);

    auto mmx = std::minmax_element(locations.begin(), locations.end(), less_by_x);
    cout<<"Minx pixel: "<<*mmx.first<<endl;
    cout<<"Maxx pixel: "<<*mmx.second<<endl;
    auto mmy = std::minmax_element(locations.begin(), locations.end(), less_by_y);
    cout<<"Miny pixel: "<<*mmy.first<<endl;
    cout<<"Maxy pixel: "<<*mmy.second<<endl;

    float widthBox = (*mmx.second).x-(*mmx.first).x;
    float heightBox = (*mmy.second).y-(*mmy.first).y;

    cv::Rect roiBoundingBox((*mmx.first).x,(*mmy.first).y,widthBox,heightBox);

    rectangle(maskDepth, roiBoundingBox, 50, 2);
    imshow("maskDepth", maskDepth);


    vector<float> boundingBox;

    boundingBox.push_back(((*mmx.first).x+bxRoi+widthBox/2)/origImageWidth);
    boundingBox.push_back(((*mmy.first).y+byRoi+heightBox/2)/origImageHeitgh);
    boundingBox.push_back(widthBox/origImageWidth);
    boundingBox.push_back(heightBox/origImageHeitgh);
    return boundingBox;



}

vector<float> SegmentorThread::cropSalientObject(Mat& origImage,const Mat& image, const Mat &depthImage, Mat & croppedImage, Mat & croppedDepthImage, int & bx, int &by)
{
    Mat bgr[3];   //destination array
    split(image,bgr);
    Mat dstImage, mask;
    cv::subtract(bgr[2],bgr[1],dstImage,mask);

    // For the cup

//    dstImage = bgr[2].clone();
    imshow("Channel 3 - channel 2", dstImage);
    //waitKey(0);




//    cv::subtract(dstImage,bgr[0],dstImage,mask);
//    imshow("Result-channel 1", dstImage);
//     waitKey(2);

    Mat thresholdImage;
    cv::threshold(bgr[2], thresholdImage,20, 255, THRESH_BINARY);
    imshow("thresholdImage", dstImage);
    waitKey(0);



//    Size kernalSize (3,3); //kernal size may change for differnt image
//    Mat element = getStructuringElement (MORPH_RECT, kernalSize, Point(1,1));
//    morphologyEx( thresholdImage, thresholdImage, MORPH_OPEN, element );
    imshow("ex", thresholdImage);
    //waitKey(2);


//    imshow("depth", depthImage);
//    waitKey(2);

    Mat cloneDepthImage = depthImage.clone();
    float maxZ=1.0;
    cout<<"Maxz: "<<maxZ<<endl;

    Mat gray(cloneDepthImage.rows,cloneDepthImage.cols,CV_8U);

    for(int x = 0;x<gray.rows;x++){
        for(int y=0;y<gray.cols;y++){
            float pix =  cloneDepthImage.at<float>(x, y);
            if (pix >= maxZ){
                pix = maxZ;
            }
            gray.at<uchar>(x,y) = (int)(pix*255)/maxZ;
        }
    }
    imshow("gray", gray);
    //waitKey(0);

    vector<Point> locations;   // output, locations of non-zero pixels
    cv::findNonZero(thresholdImage, locations);
    cout<<locations.size()<<endl;
    auto mmx = std::minmax_element(locations.begin(), locations.end(), less_by_x);
    cout<<"Minx pixel: "<<*mmx.first<<endl;
    cout<<"Maxx pixel: "<<*mmx.second<<endl;
    auto mmy = std::minmax_element(locations.begin(), locations.end(), less_by_y);
    cout<<"Miny pixel: "<<*mmy.first<<endl;
    cout<<"Maxy pixel: "<<*mmy.second<<endl;

//    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
//    erode(thresholdImage, thresholdImage, kernel);
//    imshow("erode", thresholdImage);
//    waitKey(0);

    int widthROI = 220;
    int heightROI = 220;
    int bestTotalIntensity = 0;
    int totalIntensityROI = 0;
    int bestX=0, bestY=0;
    float prevAvgLimitsIntensity[2] = {0.0,0.0};
    float avgX=0;
    float avgY=0;
    vector<float> yvector;

    auto minx = *mmx.first;

    int x = minx.x-widthROI;
    if (x<0){
        x=0;
    }
    int xmax = (*mmx.second).x;

    if(xmax>thresholdImage.cols-widthROI)
    {
        xmax = thresholdImage.cols-widthROI;
    }
    int y = (*mmy.first).y-heightROI;
    if (y<0){
        y=0;
    }
    int startY = y;

    int ymax = (*mmy.second).y;

    if(ymax>thresholdImage.rows-heightROI)
    {
        ymax = thresholdImage.rows-heightROI;
    }



    for(x;x<xmax; x=x+1)
    {
        for(y=startY;y<ymax;y=y+1)
        {
            avgX=0;
            avgY=0;
            totalIntensityROI = countWindowPixels(thresholdImage, x,y,widthROI,heightROI,&avgX,&avgY,yvector);
            if(totalIntensityROI>=bestTotalIntensity)
            {
                        bestTotalIntensity = totalIntensityROI;
                        bestX = x;
                        bestY = y;
            }
        }
    }
    yvector.clear();
    cout<<"best crop"<<endl;
    totalIntensityROI = countWindowPixels(thresholdImage, bestX,bestY,widthROI,heightROI,&avgX,&avgY,yvector);
//    cout<<yvector<<endl;
    cout<<"avg: "<<avgX<<", "<<avgY<<endl;
//    auto n = yvector.size();
//    float average = 0;
//    for(int i=0;i<n;i++){
//         average =average+ yvector[i];
//    }

//    cout<<"avg Y:"<<avgY<<endl;
//    avgY = average/n;
//    cout<<"average: "<<avgY<<endl;
    bx = bestX+avgX-widthROI/2;
    if ((bx+widthROI)>origImage.cols)
    {
        bx = origImage.cols-widthROI-1;
    }
    cout<<origImage.cols<<" "<<origImage.rows<<endl;
    cout<<bx<<" "<<origImage.cols-widthROI<<endl;
    by = bestY+avgY-heightROI/2;
    if ((by+heightROI)>origImage.rows)
    {
        by = origImage.rows-heightROI-1;
    }
    cout<<by<<" "<<origImage.rows-heightROI<<endl;

    cv::Rect roi(bx,by,widthROI,heightROI);
    // check the box within the image plane
    if (0 <= roi.x
        && 0 <= roi.width
        && roi.x + roi.width <= origImage.cols
        && 0 <= roi.y
        && 0 <= roi.height
        && roi.y + roi.height <= origImage.rows){

    }
    else{
        yError()<<"outside bounds";
        // box out of image plane, do something...
    }
#if BACKGROUND_CROP
    // Now we have the positive images, and we need the negative ones
    int minimumNumberBxBackground = 0;
    int maximumNumberBxBackground = origImage.cols - widthROI;

    int minimumNumberByBackground = 0;
    int maximumNumberByBackground = origImage.rows - heightROI;
    bool inX = true;
    bool inY = true;
    int bxBackground = 0;
    int byBackground = 0;
    while((inX) && (inY)){
        yInfo()<<inX<<" "<<inY;
        bxBackground = (rand() % (maximumNumberBxBackground + 1 - minimumNumberBxBackground)) + minimumNumberBxBackground;
        byBackground = (rand() % (maximumNumberByBackground + 1 - minimumNumberByBackground)) + minimumNumberByBackground;
        // Check background's roi does not overlap the object's roi
        if(bxBackground<bx && (bxBackground+widthROI)>bx){
            inX = true;
        }
        else if(bxBackground>bx && bxBackground<(bx+widthROI)){
            inX = true;
        }
        else{
            inX = false;
        }

        if(byBackground<by && (byBackground+heightROI)>by){
            inY = true;
        }
        else if(byBackground>by && byBackground<(by+heightROI)){
            inY = true;
        }
        else{
            inY = false;
        }

    }

    cv::Rect roiBackground(bxBackground,byBackground,widthROI,heightROI);
    croppedImage = origImage(roiBackground);
    croppedDepthImage = depthImage(roiBackground);
#else
    croppedImage = image(roi);
    croppedDepthImage = depthImage(roi);

    x = 2;
    Vec3b bgrPixelTable;
    for(int y=0 ;y<croppedImage.rows;y++){
        bgrPixelTable = croppedImage.at<Vec3b>(y, x);
        if(bgrPixelTable.val[0] != 0 || bgrPixelTable.val[1] != 0  || bgrPixelTable.val[2] != 0 )
        {
            break;
        }
    }

    for(int x = 0;x<croppedImage.cols;x++){
        for(int y = 0;y<croppedImage.rows;y++){
            Vec3b bgrPixel = croppedImage.at<Vec3b>(y, x);
            if(bgrPixel.val[0] == 0 && bgrPixel.val[1] == 0  && bgrPixel.val[2] == 0 )
            {
                croppedImage.at<Vec3b>(y, x) = bgrPixelTable;
            }
        }
    }


    imshow("cropped after",croppedImage);


#endif


    //circle(croppedImage, Point(avgX, avgY), 4, Scalar(0, 255, 0),-1);
    //imshow("cropped", croppedImage);
    //waitKey(2);
    //imshow("depth image", depthImage);
    //imshow("cropped depth", croppedDepthImage);
    //waitKey(2);


    //printf("avg: %f, %f\n", avgX, avgY);
    //printf("%d\n", totalIntensityROI);
    //printf("distance to center y: %f\n", abs((heightROI/2)-avgY));
    //printf("distance to center x: %f\n", abs((widthROI/2)-avgX));
    //printf("width roi: %d, height roi: %d", widthROI, heightROI);

    //cv::Mat croppedThresholdImage = thresholdImage(roi);
    //imshow("cropped Threshold", croppedThresholdImage);
    //waitKey(0);

    Mat croppedGray = gray(roi);

    imshow("Depth cropped gray", croppedGray);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9));
    dilate(croppedGray, croppedGray, kernel);

//    dilate(croppedGray, croppedGray, kernel);

    imshow("dilate",croppedGray);
//    waitKey(1);



    x = 2;
    int pix = 0;
    for(int y=croppedGray.rows-1;y>=0;y--){
        if(croppedGray.at<uchar>(y,x) != 0){
            pix = croppedGray.at<uchar>(y,x);
            break;
        }
    }
    cout<<pix<<endl;
    for(int x = 0;x<croppedGray.cols;x++)
    for(int y = 0;y<croppedGray.rows;y++){
        if(croppedGray.at<uchar>(y,x) == 0){
           croppedGray.at<uchar>(y,x) = pix;
        }
    }
    imshow("delete contour of depth image",croppedGray);
//    waitKey(0);

//    Mat thresholdCroppedGray;
//    cv::threshold(croppedGray, thresholdCroppedGray, 0, 255, CV_THRESH_BINARY | THRESH_OTSU);

//    imshow("THRESHOLD cropped depth gray",thresholdCroppedGray);
//    waitKey(1);



    Mat dist = croppedDepthImage.clone();


    normalize(dist, dist, 0.0, 1.1, NORM_MINMAX);
    imshow("Distance Transform Image", dist);

    Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(dist, dist, kernel1);
    imshow("Peaks", dist);
//    waitKey(0);




    // Threshold to obtain the peaks
    // This will be the markers for the foreground objects
    //cv::threshold(dist, dist, 0.3, 1.0, THRESH_BINARY);
    // Dilate a bit the dist image
    //imshow("Distance Transform thr", dist);


    Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    Mat grad, grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    blur(croppedGray, croppedGray, Size( 5, 5 ), Point(-1,-1) );
    imshow("blur", croppedGray);


//    waitKey(0);
    Sobel(croppedGray, grad_x, CV_16S, 1, 0, 3, 5, 0, BORDER_DEFAULT);
    Sobel(croppedGray, grad_y, CV_16S, 0, 1, 3, 5, 0, BORDER_DEFAULT);
    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    imshow("Sobel", grad);

//    waitKey(0);

//    y = grad.rows-1;
//    for(int x = 0;x<grad.cols;x++){
//           grad.at<uchar>(y,x) = 0;
//    }
    cv::Rect roi_grad(0,0,grad.cols,grad.rows);
    rectangle(grad, roi_grad, 0, 4);

    imshow("Sobel2", grad);
//    waitKey(0);

//    add(grad,grad,grad);
//    imshow("Sobel3", grad);
//    waitKey(0);
    // Visualize the final image

//    cv::threshold (grad, grad, 20, 255, CV_THRESH_BINARY);
//    imshow("Sobel", grad);
//    waitKey(0);

//    cv::add(dstImage,bgr[0],dstImage,mask);
//    imshow("Channel 3 - channel 2 +channel 1", dstImage);

    Mat croppedProject = dstImage(roi);
    imshow("dst", croppedProject);
//    waitKey(0);

    Mat cropChannel = bgr[2](roi);
    imshow("dst0", cropChannel);
//    waitKey(0);

    for(int x = 0;x<cropChannel.cols;x++)
    for(int y = 0;y<cropChannel.rows;y++){
        if(cropChannel.at<uchar>(y,x) == 0){
           cropChannel.at<uchar>(y,x) = 255;
        }
    }

    Mat cannyChannel;
    Canny(cropChannel, cannyChannel, 200, 255);
    imshow("Canny channel", cannyChannel);

    add(croppedProject,cannyChannel,croppedProject);
    imshow("dst1", croppedProject);
//    waitKey(0);

    Mat addition;
    add(croppedProject,dist_8u, addition);
    imshow("addition dist", addition);
    //waitKey(0);
    //add(addition,dist_8u, addition);


    add(addition,grad, addition);



    imshow("addition sobel",addition);
    //waitKey(0);

//    Mat kernelErode = getStructuringElement(MORPH_RECT, Size(3, 3));
//    erode(addition, addition, kernelErode);
//    imshow("Erode", addition);
//    waitKey(0);


    Mat cannyAdd;
    Canny(addition, cannyAdd, 120, 255);
    imshow("Canny add", cannyAdd);
//    waitKey(0);


    Mat kernel2 = getStructuringElement(MORPH_RECT, Size(17, 17));
    dilate(cannyAdd, cannyAdd, kernel);


    imshow("Canny add dilate", cannyAdd);
//    waitKey(0);


    vector< vector <Point> > contours; // Vector for storing contour
    vector< Vec4i > hierarchy;
    findContours(cannyAdd, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

    int largest_contour_index=0;
    int largest_area=0;

    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a=contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
        }
    }

    Mat maskDepth(Size(croppedGray.cols, croppedGray.rows),CV_8UC1);
    drawContours(maskDepth,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
    Mat kernel4 = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(maskDepth,maskDepth,kernel4);
    Mat kernel5 = getStructuringElement(MORPH_RECT, Size(7, 7));
    dilate(maskDepth,maskDepth,kernel5);
    imshow("Mask depth",maskDepth);
    waitKey(0);



    // Apply mask to rgb crop
    cv::threshold (maskDepth, maskDepth, 70, 255, CV_THRESH_BINARY);
    Mat maskInColor;
    croppedImage.copyTo(maskInColor, maskDepth);

    //croppedImage = maskInColor;
    vector<float> boundingBox;

    boundingBox = getBoundingBox(origImage.cols,origImage.rows, bx,by,maskDepth);
    cout<<boundingBox<<endl;
    cv::Rect roiBoundingBox(boundingBox[0]*origImage.cols-boundingBox[2]*origImage.cols/2,boundingBox[1]*origImage.rows - boundingBox[3]*origImage.rows/2,boundingBox[2]*origImage.cols,boundingBox[3]*origImage.rows);

    rectangle(origImage, roiBoundingBox, 0, 2);
    imshow("bounding box", origImage);
    waitKey(0);

//    //imshow("Canny after dilate", canny);
//    cvtColor(canny, lineas, CV_GRAY2BGR);
//    inRange(lineas, Scalar(100,100,100), Scalar(255,255,255), mask);
//    lineas.setTo(Scalar(0,0,255),mask);
//    imshow("Detected cup", lineas);
//    waitKey(0);

//    cv::add(origImage,lineas,dstImage);
//    imshow("Detected cup", dstImage);
//    waitKey(0);



//    normalize(croppedGray, croppedGray, 0, 255, cv::NORM_MINMAX);

//    erode(croppedGray, croppedGray, kernel);
//    imshow("normalize",croppedGray);

//    waitKey(1);



    //Try kmeans

//    Mat ocv;
//    cvtColor(croppedGray,ocv,CV_GRAY2BGR);
//    Mat data;
//    ocv.convertTo(data,CV_32F);
//    data = data.reshape(1,data.total());

//    // do kmeans
//    Mat labels, centers;
//    kmeans(data, 2, labels, TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 10,
//           KMEANS_PP_CENTERS, centers);

//    // reshape both to a single row of Vec3f pixels:
//    centers = centers.reshape(3,centers.rows);
//    data = data.reshape(3,data.rows);

//    // replace pixel values with their center value:
//    Vec3f *p = data.ptr<Vec3f>();
//    for (size_t i=0; i<data.rows; i++) {
//       int center_id = labels.at<int>(i);
//       p[i] = centers.at<Vec3f>(center_id);
//    }

//    // back to 2d, and uchar:
//    ocv = data.reshape(3, ocv.rows);
//    ocv.convertTo(ocv, CV_8U);

//    imshow("kmeans", ocv);
//    waitKey(0);
//    destroyAllWindows();
//    vector< vector <Point> > contours; // Vector for storing contour
//    vector< Vec4i > hierarchy;
//    findContours(grad, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

//    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
//    {
//        drawContours( croppedImage,contours, i, Scalar(255,255,255),CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.

//    }
//    imshow("contours", croppedImage);
//    waitKey(0);


    return boundingBox;

}

Point SegmentorThread::getCentroid( InputArray Points )
{
    Point Coord;
    Moments mm = moments( Points, true );
    double moment10 = mm.m10;
    double moment01 = mm.m01;
    double moment00 = mm.m00;
    Coord.x = (int)moment10 / moment00;
    Coord.y = (int)moment01 / moment00;
    return Coord;
}


int SegmentorThread::countWindowPixels(Mat& image, int roiX, int roiY, int roiWidth, int roiHeight, float * xAvg, float * yAvg, vector<float>& yvector){
    cv::Rect roi(roiX,roiY,roiWidth,roiHeight);
    cv::Mat croppedImage = image(roi);
    int totalIntensity = 0;
    float countPixels = 0;

    cv::Scalar sum= cv::sum(croppedImage);
    totalIntensity = sum[0];
    vector<Point> locations;   // output, locations of non-zero pixels
    cv::findNonZero(croppedImage, locations);

    if (!locations.empty()){
        auto mmx = std::minmax_element(locations.begin(), locations.end(), less_by_x);
        auto mmy = std::minmax_element(locations.begin(), locations.end(), less_by_y);
        *xAvg = ((*mmx.first).x + (*mmx.second).x)/2.0;
        *yAvg = ((*mmy.first).y + (*mmy.second).y)/2.0;
    }


    //cout<<"avg x:"<<*xAvg<<endl;
    //cout<<"avg y:"<<*yAvg<<endl;

//    float xmean=0, ymean=0;

//    int ymax=0;
//    for(int x = 0; x < croppedImage.cols; x++)
//    {
//        for(int y=0; y < croppedImage.rows; y++)
//        {
//            uint pixelIntensity = (int)croppedImage.at<uchar>(x,y);
//            totalIntensity += pixelIntensity;
//            if(pixelIntensity>=240){
//                xmean += x;
//                ymean += y;
//                countPixels+=1;
//                if(y>=ymax){
//                    ymax=y;
//                }
//                yvector.push_back(y);
//            }
//        }
//    }

//    if(countPixels!=0){
//        xmean = xmean/countPixels;
//        ymean = ymean/countPixels;
//    }

//    *xAvg = xmean;
//    *yAvg = ymean;

    return totalIntensity;


}
void SegmentorThread::depthFilter(Mat &image, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth, float maxDistance){

    //imshow("Imagen", image);
    //waitKey(1);

    Point center = Point(image.size().width/2, image.size().height/2);
    float maxZ = 0, minZ = maxDistance;
    for(int x = 0; x < depth.width(); x++){
        for(int y = 0; y < depth.height(); y++){

            double mmZ_tmp = depth.pixel(int(x), int(y));
            if(mmZ_tmp > maxDistance){
                circle(image, Point(x,y), 4, Scalar(0, 0, 0), -1, 8);
            }
            if(mmZ_tmp>maxZ){
                maxZ = mmZ_tmp;
            }
            if(mmZ_tmp<minZ){
                minZ = mmZ_tmp;
            }

            if (y<180){
                circle(image, Point(x,y), 8, Scalar(0, 0, 0), -1, 8);
            }

            if(x<30){
                circle(image, Point(x,y), 8, Scalar(0, 0, 0), -1, 8);

            }

            if(x>600 && y<280){
                circle(image, Point(x,y), 8, Scalar(0, 0, 0), -1, 8);

            }


        }
    }

    yInfo()<<"Max: "<<maxZ<<"Min: "<<minZ;
//    imshow("Imagen2", image);
//    waitKey(0);

}

vector<float> SegmentorThread::findGlass(const Mat& origImage,const Mat& image, const Mat &depthImage){
    Mat cloneDepthImage = depthImage.clone();
    float maxZ=1.0;
    cout<<"Maxz: "<<maxZ<<endl;

    Mat gray(cloneDepthImage.rows,cloneDepthImage.cols,CV_8U);

    for(int x = 0;x<gray.rows;x++){
        for(int y=0;y<gray.cols;y++){
            float pix =  cloneDepthImage.at<float>(x, y);
            if (pix >= maxZ){
                pix = maxZ;
            }
            gray.at<uchar>(x,y) = (int)(pix*255)/maxZ;
            if(x<250)
            {
                gray.at<uchar>(x,y) = 255;

            }

            if(x>460)
            {
                gray.at<uchar>(x,y) = 255;

            }

            if(y>590)
            {
                gray.at<uchar>(x,y) = 255;

            }

            if(y<60)
            {
                gray.at<uchar>(x,y) = 255;

            }
        }
    }
//    imshow("gray", gray);
//    waitKey(0);

    cv::threshold(gray, gray,120, 255, THRESH_BINARY_INV);

//    imshow("threshold", gray);
//    waitKey(0);

//    imshow("rectangle", gray);
//    waitKey(0);

    Mat thresholdImage = gray.clone();

    vector<Point> locations;   // output, locations of non-zero pixels
    cv::findNonZero(thresholdImage, locations);
    cout<<locations.size()<<endl;
    auto mmx = std::minmax_element(locations.begin(), locations.end(), less_by_x);
    cout<<"Minx pixel: "<<*mmx.first<<endl;
    cout<<"Maxx pixel: "<<*mmx.second<<endl;
    auto mmy = std::minmax_element(locations.begin(), locations.end(), less_by_y);
    cout<<"Miny pixel: "<<*mmy.first<<endl;
    cout<<"Maxy pixel: "<<*mmy.second<<endl;

//    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
//    erode(thresholdImage, thresholdImage, kernel);
//    imshow("erode", thresholdImage);
//    waitKey(0);

    int widthROI = 100;
    int heightROI = 100;
    int bestTotalIntensity = 0;
    int totalIntensityROI = 0;
    int bestX=0, bestY=0;
    float prevAvgLimitsIntensity[2] = {0.0,0.0};
    float avgX=0;
    float avgY=0;
    int bx=0, by=0;

    vector<float> yvector;

    auto minx = *mmx.first;

    int x = minx.x-widthROI;
    if (x<0){
        x=0;
    }
    int xmax = (*mmx.second).x;

    if(xmax>thresholdImage.cols-widthROI)
    {
        xmax = thresholdImage.cols-widthROI;
    }
    int y = (*mmy.first).y-heightROI;
    if (y<0){
        y=0;
    }
    int startY = y;

    int ymax = (*mmy.second).y;

    if(ymax>thresholdImage.rows-heightROI)
    {
        ymax = thresholdImage.rows-heightROI;
    }



    for(x;x<xmax; x=x+1)
    {
        for(y=startY;y<ymax;y=y+1)
        {
            avgX=0;
            avgY=0;
            totalIntensityROI = countWindowPixels(thresholdImage, x,y,widthROI,heightROI,&avgX,&avgY,yvector);
            if(totalIntensityROI>=bestTotalIntensity)
            {
                        bestTotalIntensity = totalIntensityROI;
                        bestX = x;
                        bestY = y;
            }
        }
    }
    yvector.clear();
    cout<<"best crop"<<endl;
    totalIntensityROI = countWindowPixels(thresholdImage, bestX,bestY,widthROI,heightROI,&avgX,&avgY,yvector);
//    cout<<yvector<<endl;
    cout<<"avg: "<<avgX<<", "<<avgY<<endl;
//    auto n = yvector.size();
//    float average = 0;
//    for(int i=0;i<n;i++){
//         average =average+ yvector[i];
//    }

//    cout<<"avg Y:"<<avgY<<endl;
//    avgY = average/n;
//    cout<<"average: "<<avgY<<endl;
    bx = bestX+avgX-widthROI/2;
    if ((bx+widthROI)>origImage.cols)
    {
        bx = origImage.cols-widthROI-1;
    }
    cout<<origImage.cols<<" "<<origImage.rows<<endl;
    cout<<bx<<" "<<origImage.cols-widthROI<<endl;
    by = bestY+avgY-heightROI/2;
    if ((by+heightROI)>origImage.rows)
    {
        by = origImage.rows-heightROI-1;
    }
    cout<<by<<" "<<origImage.rows-heightROI<<endl;

    cv::Rect roi(bx,by,widthROI,heightROI);
    // check the box within the image plane
    if (0 <= roi.x
        && 0 <= roi.width
        && roi.x + roi.width <= origImage.cols
        && 0 <= roi.y
        && 0 <= roi.height
        && roi.y + roi.height <= origImage.rows){

    }
    else{
        yError()<<"outside bounds";
        // box out of image plane, do something...
    }


    Mat croppedImage = thresholdImage(roi);
    imshow("cropped", croppedImage);
//   waitKey(0);


    vector< vector <Point> > contours; // Vector for storing contour
    vector< Vec4i > hierarchy;
    findContours(croppedImage, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

    int largest_contour_index=0;
    int largest_area=0;

    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a=contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
        }
    }
    drawContours(croppedImage,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
    imshow("maskDepth",croppedImage);
//    waitKey(0);
//    Mat kernel4 = getStructuringElement(MORPH_RECT, Size(5, 5));
//    erode(croppedImage,croppedImage,kernel4);

    Mat kernel5 = getStructuringElement(MORPH_RECT, Size(9, 9));
    dilate(croppedImage,croppedImage,kernel5);

    vector<float> boundingBox;

    boundingBox = getBoundingBox(origImage.cols,origImage.rows, bx,by,croppedImage);

    cv::Rect roiBoundingBox(boundingBox[0]*origImage.cols-boundingBox[2]*origImage.cols/2,boundingBox[1]*origImage.rows - boundingBox[3]*origImage.rows/2,boundingBox[2]*origImage.cols,boundingBox[3]*origImage.rows);

    rectangle(origImage, roiBoundingBox, 0, 2);
    imshow("bounding box", origImage);
//    waitKey(0);

    return boundingBox;





}

vector<float> SegmentorThread::findCereal(const Mat& origImage,const Mat& image, const Mat &depthImage){
    Mat bgr[3];   //destination array
    split(image,bgr);
    Mat dstImage, mask;
    cv::subtract(bgr[2],bgr[1],dstImage,mask);

    imshow("Channel 1", bgr[0]);
    imshow("Depth", depthImage);
    Mat prueba;

    cv::threshold(bgr[0], prueba,120, 255, THRESH_BINARY_INV);
    imshow("Threshold", prueba);

    dstImage = prueba.clone();

    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(dstImage, dstImage, kernel);
    dilate(dstImage,dstImage,kernel);
    imshow("final", dstImage);
    waitKey(0);

    //instantiates the specific Saliency
    Ptr<Saliency> saliencyAlgorithm = StaticSaliencySpectralResidual::create();
    Mat binaryMap;
    Mat saliencyMap;
    if( saliencyAlgorithm->computeSaliency( bgr[0], saliencyMap ) )
    {
      StaticSaliencySpectralResidual spec;
      spec.computeBinaryMap( saliencyMap, binaryMap );

      imshow( "Saliency Map", saliencyMap );
      imshow( "Original Image", image );
      imshow( "Binary Map", binaryMap );
      waitKey( 0 );
    }


    vector<float> boundingBox;


    return boundingBox;





}


vector<float> SegmentorThread::findCup(const Mat& origImage,const Mat& image, vector<Point>& points){

    Mat gray0(image.size(), CV_8U), gray;
    Mat kernel, canny, lineas;
    Mat prueba;
    Mat thresholdImage;
    Mat bgr[3];   //destination array
    split(image,bgr);

    //imshow("blue", bgr[0]);
    //imshow("green", bgr[1]);
    imshow("red", bgr[2]);
    cv::threshold(bgr[2], prueba,120, 255, THRESH_BINARY_INV);
    imshow("Threshold", prueba);
//    waitKey(0);

//    vector< vector <Point> > contours; // Vector for storing contour
//    vector< Vec4i > hierarchy;
//    findContours(prueba, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
//    int largest_contour_index=0;
//    int largest_area=0;

//    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
//    {
//        double a=contourArea( contours[i],false);  //  Find the area of contour
//        if(a>largest_area){
//            largest_area=a;
//            largest_contour_index=i;                //Store the index of largest contour
//        }
//    }

//    drawContours( prueba,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.

//    imshow("contour",prueba);

//    Size kernalSize (7,100); //kernal size may change for differnt image
//    Mat element = getStructuringElement (MORPH_RECT, kernalSize, Point(1,1)  );
//    morphologyEx( prueba, prueba, MORPH_OPEN, element );


//    kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
//    dilate(prueba, prueba, kernel);
//    imshow("dilate", prueba);

//    kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
//    erode(prueba, prueba, kernel);
//    imshow("erode", prueba);
//    Mat invertedErode;
//    cv::bitwise_not(prueba,invertedErode);
//    imshow("inverted erode", invertedErode);

//    Mat dstImage,mask;

//    cv::subtract(thresholdImage,invertedErode,dstImage,mask);
//    imshow("mask inverted", dstImage);
//    waitKey(0);

//    cv::bitwise_not(dstImage,dstImage);
//    imshow("mask inverted", dstImage);
//    waitKey(0);

//    cv::subtract(dstImage,invertedErode,dstImage,mask);
//    imshow("mask inverted", dstImage);
//    waitKey(0);

//    cv::threshold(dstImage, dstImage,120, 200, THRESH_BINARY);
//    imshow("final", dstImage);
//    waitKey(0);
    Mat dstImage = prueba.clone();

    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(dstImage, dstImage, kernel);
    dilate(dstImage,dstImage,kernel);
    imshow("final", dstImage);
//    waitKey(0);

    thresholdImage = dstImage.clone();

    vector<Point> locations;   // output, locations of non-zero pixels
    cv::findNonZero(thresholdImage, locations);
    cout<<locations.size()<<endl;
    auto mmx = std::minmax_element(locations.begin(), locations.end(), less_by_x);
    cout<<"Minx pixel: "<<*mmx.first<<endl;
    cout<<"Maxx pixel: "<<*mmx.second<<endl;
    auto mmy = std::minmax_element(locations.begin(), locations.end(), less_by_y);
    cout<<"Miny pixel: "<<*mmy.first<<endl;
    cout<<"Maxy pixel: "<<*mmy.second<<endl;

//    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
//    erode(thresholdImage, thresholdImage, kernel);
//    imshow("erode", thresholdImage);
//    waitKey(0);

    int widthROI = 100;
    int heightROI = 100;
    int bestTotalIntensity = 0;
    int totalIntensityROI = 0;
    int bestX=0, bestY=0;
    float prevAvgLimitsIntensity[2] = {0.0,0.0};
    float avgX=0;
    float avgY=0;
    int bx=0, by=0;

    vector<float> yvector;

    auto minx = *mmx.first;

    int x = minx.x-widthROI;
    if (x<0){
        x=0;
    }
    int xmax = (*mmx.second).x;

    if(xmax>thresholdImage.cols-widthROI)
    {
        xmax = thresholdImage.cols-widthROI;
    }
    int y = (*mmy.first).y-heightROI;
    if (y<0){
        y=0;
    }
    int startY = y;

    int ymax = (*mmy.second).y;

    if(ymax>thresholdImage.rows-heightROI)
    {
        ymax = thresholdImage.rows-heightROI;
    }



    for(x;x<xmax; x=x+1)
    {
        for(y=startY;y<ymax;y=y+1)
        {
            avgX=0;
            avgY=0;
            totalIntensityROI = countWindowPixels(thresholdImage, x,y,widthROI,heightROI,&avgX,&avgY,yvector);
            if(totalIntensityROI>=bestTotalIntensity)
            {
                        bestTotalIntensity = totalIntensityROI;
                        bestX = x;
                        bestY = y;
            }
        }
    }
    yvector.clear();
    cout<<"best crop"<<endl;
    totalIntensityROI = countWindowPixels(thresholdImage, bestX,bestY,widthROI,heightROI,&avgX,&avgY,yvector);
//    cout<<yvector<<endl;
    cout<<"avg: "<<avgX<<", "<<avgY<<endl;
//    auto n = yvector.size();
//    float average = 0;
//    for(int i=0;i<n;i++){
//         average =average+ yvector[i];
//    }

//    cout<<"avg Y:"<<avgY<<endl;
//    avgY = average/n;
//    cout<<"average: "<<avgY<<endl;
    bx = bestX+avgX-widthROI/2;
    if ((bx+widthROI)>origImage.cols)
    {
        bx = origImage.cols-widthROI-1;
    }
    cout<<origImage.cols<<" "<<origImage.rows<<endl;
    cout<<bx<<" "<<origImage.cols-widthROI<<endl;
    by = bestY+avgY-heightROI/2;
    if ((by+heightROI)>origImage.rows)
    {
        by = origImage.rows-heightROI-1;
    }
    cout<<by<<" "<<origImage.rows-heightROI<<endl;

    cv::Rect roi(bx,by,widthROI,heightROI);
    // check the box within the image plane
    if (0 <= roi.x
        && 0 <= roi.width
        && roi.x + roi.width <= origImage.cols
        && 0 <= roi.y
        && 0 <= roi.height
        && roi.y + roi.height <= origImage.rows){

    }
    else{
        yError()<<"outside bounds";
        // box out of image plane, do something...
    }


    Mat croppedImage = thresholdImage(roi);
    imshow("cropped", croppedImage);
//   waitKey(0);


    vector< vector <Point> > contours; // Vector for storing contour
    vector< Vec4i > hierarchy;
    findContours(croppedImage, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

    int largest_contour_index=0;
    int largest_area=0;

    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a=contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
        }
    }
    Mat maskDepth(Size(croppedImage.cols, croppedImage.rows),CV_8UC1);
    drawContours(croppedImage,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
    imshow("maskDepth",croppedImage);
//    waitKey(0);
//    Mat kernel4 = getStructuringElement(MORPH_RECT, Size(5, 5));
//    erode(croppedImage,croppedImage,kernel4);

    Mat kernel5 = getStructuringElement(MORPH_RECT, Size(9, 9));
    dilate(croppedImage,croppedImage,kernel5);



    imshow("maskDepth",croppedImage);
//    waitKey(0);

    vector<float> boundingBox;

    boundingBox = getBoundingBox(origImage.cols,origImage.rows, bx,by,croppedImage);

    cv::Rect roiBoundingBox(boundingBox[0]*origImage.cols-boundingBox[2]*origImage.cols/2,boundingBox[1]*origImage.rows - boundingBox[3]*origImage.rows/2,boundingBox[2]*origImage.cols,boundingBox[3]*origImage.rows);

    rectangle(origImage, roiBoundingBox, 0, 2);
    imshow("bounding box", origImage);
//    waitKey(0);

    return boundingBox;




//    Canny(dstImage, canny, 100, 150);

//    imshow("Canny", canny);
//    waitKey(0);




//    //imshow("Canny after dilate", canny);
//    cvtColor(canny, lineas, CV_GRAY2BGR);
//    inRange(lineas, Scalar(100,100,100), Scalar(255,255,255), mask);
//    lineas.setTo(Scalar(0,0,255),mask);
//    imshow("Detected cup", lineas);
//    waitKey(0);

//    cv::add(origImage,lineas,dstImage);
//    imshow("Detected cup", dstImage);
//    waitKey(0);


//#if 0 //cambiar a 1 para usar el método estándar
//    vector<Vec2f> lines;
//    HoughLines(canny, lines, 1, CV_PI/180, 50, 0, 0);

//    //Almacenamos todos los puntos que formas las lineas en el vector
//    for (size_t i = 0; i < lines.size(); i++){
//        float rho = lines[i][0], theta = lines[i][1];
//        Point pt1, pt2;
//        double a = cos(theta), b = sin(theta);
//        double x0 = a*rho0, y0 = b*rho;
//        pt1.x = cvRound(x0 + 1000 * (-b));
//        pt1.y = cvRound(y0 + 1000 * (a));
//        pt2.x = cvRound(x0 + 1000 * (-b));
//        pt2.y = cvRound(y0 + 1000 * (a));
//        line(lineas, pt1, pt2, Scalar(0,0,255), 3, CV_AA);    }
//#else
//    vector<Vec4i> lines;
//    HoughLinesP(canny, lines, 1.0, CV_PI/180, 30, 30, 30);
//    for (size_t i = 0; i < lines.size(); i++){
//        Vec4i l = lines[i];
//        line(lineas, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
//        points.push_back(Point(l[0], l[1]));
//        points.push_back(Point(l[2], l[3]));
//    }
//#endif
//    //imshow("lineas sin reducir", lineas);
//    /*
//    kernel = getStructuringElement(MORPH_RECT, Size(6, 6));
//    erode(lineas, gray, kernel);
//*/
//    //
//    gray = lineas.clone();
//    //

//    imshow("Lineas", gray);
//    cvtColor(gray, gray, CV_BGR2GRAY);
////            }
////            else{
////                gray = gray0 >= (l+1) * 255/N;
////            }


//    cout << "Cuantos puntos habra??? " << points.size() << endl;
//    bool correct;
//    if (points.size() >= 10 && points.size() < 14){
//        correct = true;
//        cout << "Numero correcto de lineas detectadas" << endl;
//    }
//    else{
//        cout << "Muchas lineas detectadas" << endl;
//        correct = false;
//    }
//    return correct;

//}

}
}
