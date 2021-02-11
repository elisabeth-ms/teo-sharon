// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

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

void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/

void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
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

/************************************************************************/
void SegmentorThread::run()
{
    yInfo()<<"run";
    yarp::sig::FlexImage colorFrame;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;

    if(!iRGBDSensor->getImages(colorFrame, depthFrame))
        return;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;
    inYarpImg.copy(colorFrame);


//    if (depth.height()<10) {
//        yDebug()<<"No depth image yet";
//        return;
//    };
    printf("width: %d, height: %d\n", colorFrame.width(), colorFrame.height());
    IplImage *inIplImage = cvCreateImage(cvSize(colorFrame.width(), colorFrame.height()),
                                            IPL_DEPTH_8U, 3 );

    cvCvtColor((IplImage*)colorFrame.getIplImage(), inIplImage, cv::COLOR_RGB2BGR);
    //Mat inCvMat(inIplImage);
    Mat inCvMat = cv::cvarrToMat(inIplImage);
    Mat origCvMat;
    inCvMat.copyTo(origCvMat);

    float maxDistance = 1.3;
    depthFilter(inCvMat,depthFrame, maxDistance);

    vector<Point> points;
    bool correct;
    //correct = findCup(origCvMat,inCvMat, points);
    printf("find milk bottle\n");
    correct = findMilkBottle(origCvMat,inCvMat,points);


}

bool SegmentorThread::findMilkBottle(const Mat& origImage,const Mat& image, vector<Point>& points)
{
    Mat bgr[3];   //destination array
    split(image,bgr);
    Mat dstImage, mask;
    cv::subtract(bgr[2],bgr[1],dstImage,mask);
    imshow("redMinusGreen", dstImage);
    waitKey(0);

    Mat thresholdImage;
    cv::threshold(dstImage, thresholdImage,60, 255, THRESH_BINARY);
    imshow("threshold", thresholdImage);
    waitKey(0);




//    vector< vector <Point> > contours; // Vector for storing contour
//    vector< Vec4i > hierarchy;
//    findContours(dstImage, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
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

//    drawContours( thresholdImage,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.

//    imshow("contour",thresholdImage);
//    waitKey(0);
//    Size kernalSize (3,3); //kernal size may change for differnt image
//    Mat element = getStructuringElement (MORPH_RECT, kernalSize, Point(1,1));
//    morphologyEx( thresholdImage, thresholdImage, MORPH_OPEN, element );
//    imshow("ex", thresholdImage);
//    waitKey(0);

//    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
//    erode(thresholdImage, thresholdImage, kernel);
//    imshow("erode", thresholdImage);
//    waitKey(0);

    int widthROI = 220;
    int heightROI = 220;
    int bestTotalIntensity = 255;
    int totalIntensityROI = 0;
    int bestX, bestY;
    float prevAvgLimitsIntensity[2] = {0.0,0.0};
    float avgX=0;
    float avgY=0;
    for(int x=0;x<(thresholdImage.cols-widthROI);x++)
    {
        for(int y=0;y<(thresholdImage.rows-heightROI);y++)
        {
            avgX=0;
            avgY=0;
            totalIntensityROI = countWindowPixels(thresholdImage, x,y,widthROI,heightROI,&avgX,&avgY);
            if(totalIntensityROI>=bestTotalIntensity)
            {
                if(fabs(heightROI/2 - avgY)<= fabs(heightROI/2-prevAvgLimitsIntensity[1]+0.0000001)){

                    if(fabs(widthROI/2 - avgX)<= fabs(widthROI/2 - prevAvgLimitsIntensity[0]+0.0000001))
                    {
                        printf("avgy: %f\n", avgY);
                        printf("more centered y\n");
                        printf("best: %d, now: %d\n", bestTotalIntensity, totalIntensityROI);
                        bestTotalIntensity = totalIntensityROI;
                        bestX = x;
                        bestY = y;
                        printf("avg: %f, %f\n", avgX, avgY);
                        prevAvgLimitsIntensity[0] = avgX;
                        prevAvgLimitsIntensity[1] = avgY;
                        printf("%d\n", totalIntensityROI);
                        printf("distance to center y: %f\n", abs((heightROI/2)-avgY));
                        printf("distance to center x: %f\n", abs((widthROI/2)-avgX));
                        printf("width roi: %d, height roi: %d", widthROI, heightROI);
                    }
                }

            }
        }
    }
    cv::Rect roi(bestX,bestY,widthROI,heightROI);
    cv::Mat croppedImage = origImage(roi);
    imshow("cropped", croppedImage);
    waitKey(0);

    totalIntensityROI = countWindowPixels(thresholdImage, bestX,bestY,widthROI,heightROI,&avgX,&avgY);
    printf("avg: %f, %f\n", avgX, avgY);
    printf("%d\n", totalIntensityROI);
    printf("distance to center y: %f\n", abs((heightROI/2)-avgY));
    printf("distance to center x: %f\n", abs((widthROI/2)-avgX));
    printf("width roi: %d, height roi: %d", widthROI, heightROI);

    cv::Mat croppedThresholdImage = thresholdImage(roi);
    imshow("cropped Threshold", croppedThresholdImage);
    waitKey(0);
    return true;

}

int SegmentorThread::countWindowPixels(Mat& image, int roiX, int roiY, int roiWidth, int roiHeight, float * xAvg, float * yAvg){
    cv::Rect roi(roiX,roiY,roiWidth,roiHeight);
    cv::Mat croppedImage = image(roi);
    int totalIntensity = 0;
    float countPixels = 0;

    for(int x = 0; x < croppedImage.cols; x++)
    {
        for(int y=0; y < croppedImage.rows; y++)
        {
            uint pixelIntensity = (int)croppedImage.at<uchar>(x,y);
            totalIntensity += pixelIntensity;
            if(pixelIntensity>=250){
                *xAvg += x;
                *yAvg += y;
                countPixels+=1;
            }
        }
    }

    if (countPixels > 10){
        *xAvg = *xAvg/countPixels;
        *yAvg = *yAvg/countPixels;
    }
    else{
        totalIntensity=0;
    }


    return totalIntensity;


}
void SegmentorThread::depthFilter(Mat &image, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth, float maxDistance){

    imshow("Imagen", image);
    waitKey(1);

    Point center = Point(image.size().width/2, image.size().height/2);

    for(int x = 0; x < depth.width(); x++){
        for(int y = 0; y < depth.height(); y++){

            double mmZ_tmp = depth.pixel(int(x), int(y));
            if(mmZ_tmp > maxDistance){
                circle(image, Point(x,y), 4, Scalar(0, 0, 0), -1, 8);
            }

        }
    }
    imshow("Imagen2", image);
    waitKey(1);

}


bool SegmentorThread::findCup(const Mat& origImage,const Mat& image, vector<Point>& points){

    Mat gray0(image.size(), CV_8U), gray;
    Mat kernel, canny, lineas;
    Mat prueba;
    Mat thresholdImage;
    Mat bgr[3];   //destination array
    split(image,bgr);

    //imshow("blue", bgr[0]);
    //imshow("green", bgr[1]);
    imshow("red", bgr[2]);
    cv::threshold(bgr[2], prueba,120, 200, THRESH_BINARY);
    imshow("Threshold", prueba);
    prueba.copyTo(thresholdImage);
    waitKey(0);

    vector< vector <Point> > contours; // Vector for storing contour
    vector< Vec4i > hierarchy;
    findContours(prueba, contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
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

    drawContours( prueba,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.

    imshow("contour",prueba);

    Size kernalSize (7,100); //kernal size may change for differnt image
    Mat element = getStructuringElement (MORPH_RECT, kernalSize, Point(1,1)  );
    morphologyEx( prueba, prueba, MORPH_OPEN, element );


    kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    dilate(prueba, prueba, kernel);
    imshow("dilate", prueba);

    kernel = getStructuringElement(MORPH_RECT, Size(19, 19));
    erode(prueba, prueba, kernel);
    imshow("erode", prueba);
    Mat invertedErode;
    cv::bitwise_not(prueba,invertedErode);
    imshow("inverted erode", invertedErode);

    Mat dstImage,mask;

    cv::subtract(thresholdImage,invertedErode,dstImage,mask);
    imshow("mask inverted", dstImage);
    waitKey(0);

    cv::bitwise_not(dstImage,dstImage);
    imshow("mask inverted", dstImage);
    waitKey(0);

    cv::subtract(dstImage,invertedErode,dstImage,mask);
    imshow("mask inverted", dstImage);
    waitKey(0);

    cv::threshold(dstImage, dstImage,120, 200, THRESH_BINARY);
    imshow("final", dstImage);
    waitKey(0);

    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(dstImage, dstImage, kernel);
    dilate(dstImage,dstImage,kernel);
    imshow("final", dstImage);
    waitKey(0);

    Canny(dstImage, canny, 100, 150);

    imshow("Canny", canny);
    waitKey(0);




    //imshow("Canny after dilate", canny);
    cvtColor(canny, lineas, CV_GRAY2BGR);
    inRange(lineas, Scalar(100,100,100), Scalar(255,255,255), mask);
    lineas.setTo(Scalar(0,0,255),mask);
    imshow("Detected cup", lineas);
    waitKey(0);

    cv::add(origImage,lineas,dstImage);
    imshow("Detected cup", dstImage);
    waitKey(0);


#if 0 //cambiar a 1 para usar el método estándar
    vector<Vec2f> lines;
    HoughLines(canny, lines, 1, CV_PI/180, 50, 0, 0);

    //Almacenamos todos los puntos que formas las lineas en el vector
    for (size_t i = 0; i < lines.size(); i++){
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho0, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 + 1000 * (-b));
        pt2.y = cvRound(y0 + 1000 * (a));
        line(lineas, pt1, pt2, Scalar(0,0,255), 3, CV_AA);    }
#else
    vector<Vec4i> lines;
    HoughLinesP(canny, lines, 1.0, CV_PI/180, 30, 30, 30);
    for (size_t i = 0; i < lines.size(); i++){
        Vec4i l = lines[i];
        line(lineas, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
        points.push_back(Point(l[0], l[1]));
        points.push_back(Point(l[2], l[3]));
    }
#endif
    //imshow("lineas sin reducir", lineas);
    /*
    kernel = getStructuringElement(MORPH_RECT, Size(6, 6));
    erode(lineas, gray, kernel);
*/
    //
    gray = lineas.clone();
    //

    imshow("Lineas", gray);
    cvtColor(gray, gray, CV_BGR2GRAY);
//            }
//            else{
//                gray = gray0 >= (l+1) * 255/N;
//            }


    cout << "Cuantos puntos habra??? " << points.size() << endl;
    bool correct;
    if (points.size() >= 10 && points.size() < 14){
        correct = true;
        cout << "Numero correcto de lineas detectadas" << endl;
    }
    else{
        cout << "Muchas lineas detectadas" << endl;
        correct = false;
    }
    return correct;

}


}
