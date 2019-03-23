//TODO: add include files

#include <opencv2/videoio.hpp>

//misc includes
#include <string>
#include <chrono>
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <stdio.h>
#include <algorithm>
#include <utility>
#include <zmq.hpp>
#include <zmq.h>

#include "opencv2/cudaimgproc.hpp"
#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef withGst
#else
//#include <opencv2/gpu/gpu.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/highgui.hpp>
#endif


static const int CAM_RES_WIDTH = 320;
static const int CAM_RES_HEIGHT = 240;
static const double CAM_ANGLE = 54.0; //degrees
static const double CAM_FOCAL_LEN = 1170.0; //TODO figure this out, is too largy
static const double TAPE_WIDTH = 2; //inches
static const double TAPE_HEIGHT = 5; //inches
static const double BETWEEN_TAPES_DIST = 9.25; //inches
static const double CAM_HEIGHT_OFFSET = 0; //TODO WRONG //47/12.0 approximately

static const double CAM_FOCAL_WIDTH = 11.0; //actual width of tape pair when at 15 inches away
static const double CAM_FOCAL_DIST = 15.0; //at this distance the tape pair is in full view
static const double DEGREES_TO_INCHES = CAM_FOCAL_WIDTH/CAM_RES_WIDTH; //APPROX 0.034375

static const int LOWER_GREEN_HUE = 50;
static const int LOWER_GREEN_SAT = 60;
static const int LOWER_GREEN_VAL = 60;
static const int UPPER_GREEN_HUE = 80;
static const int UPPER_GREEN_SAT = 255;
static const int UPPER_GREEN_VAL = 255;

// TODO: TUNE ALL OF THE BELOW
static const int MIN_AREA = 100;
static const double MIN_ASPECT_RATIO = 0.357 - 0.2;//0.15;
static const double MAX_ASPECT_RATIO = 0.357 + 0.2;
//const double MIN_LEFT_ASPECT_RATIO = 2.70 - 0.2;
//const double MAX_LEFT_ASPECT_RATIO = 2.70 + 0.2;

static const double LEFT_MIN_ANGLE = -15.0 - 10.0;//75.0 - 10.0;
static const double LEFT_MAX_ANGLE = -15.0 + 10.0;//75.0 + 10.0; 
static const double RIGHT_MIN_ANGLE = -75.0 - 10.0;//105.0 - 10.0;
static const double RIGHT_MAX_ANGLE = -75.0 + 10.0;//105.0 + 10.0;

cv::RNG rng(12345);

#define DEFAULT_CAMERA 1
#define PI 3.141592653589

cv::VideoCapture init(int camera_id){
    cv::VideoCapture cap(camera_id);
    //std::string command = std::string("v4l2-ctl -d /dev/video") + std::to_string(camera_id) + std::string(" -c exposure_auto=1 -c exposure_absolute=5 -c brightness=30"); //TODO outted for assigncamera
    //system(command.c_str()); //TODO to
//    std::string command = std::string("v4l2-ctl --set-fmt-video=width=") + std::to_string(CAM_RES_WIDTH) +
  //                                    ",height=" + std::to_string(CAM_RES_HEIGHT)+",pixelformat=YUYV";
    //std::string exposureCommand = std::string("v4l2-ctl -d /dev/video") + std::to_string(DEFAULT_CAMERA) + " -c exposure_auto=0 -c exposure_absolute=1 -c white_balance_temperature_auto=0";
   // std::cout << command << std::endl << exposureCommand << std::endl;
    //system(command.c_str());
    //system(exposureCommand.c_str());
    while(!cap.isOpened()){
        std::cout << "opening camera..." << std::endl;
        cap.open(camera_id);
    }
    return cap;
}

void readToGPUMat(cv::VideoCapture cap, cv::cuda::GpuMat &outMat){
    cv::Mat frame;
    cap.read(frame);
    //cv::resize(frame, frame, cvSize(320, 240), (1.0/3), (1.0/3), 3); //NOTE: values probably wrong (except 1-3 params)
    //cv::cuda::GpuMat gpumat;
    outMat.upload(frame); //about 1ms
}

void filterGreen(cv::cuda::GpuMat hsvMat, cv::cuda::GpuMat &outMat) {
    cv::cuda::GpuMat temp;

    cv::cuda::GpuMat splitHsv[3];
    cv::cuda::split(hsvMat, splitHsv);
    
    cv::cuda::GpuMat thresc[3];
    
    cv::cuda::threshold(splitHsv[0], thresc[0], LOWER_GREEN_HUE, UPPER_GREEN_HUE, cv::THRESH_BINARY);
    cv::cuda::threshold(splitHsv[1], thresc[1], LOWER_GREEN_SAT, UPPER_GREEN_SAT, cv::THRESH_BINARY);
    cv::cuda::threshold(splitHsv[2], thresc[2], LOWER_GREEN_VAL, UPPER_GREEN_VAL, cv::THRESH_BINARY);
    
    cv::cuda::bitwise_and(thresc[0], thresc[1], temp);
    cv::cuda::bitwise_and(temp, thresc[2], outMat);
}

//TODO
//draw rotated rects, match pairs, select pair, TEST, angle/distance
void findRectangles(cv::Mat greenMat, std::vector<std::vector<cv::Point>> &contours, std::vector<cv::Vec4i> &hierarchy, std::vector<cv::RotatedRect> &rotatedRects){
    //possible error: greenMat not black and white? check filterGreen()
    cv::findContours(greenMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0)); //TODO Check
    for(int i = 0; i<contours.size(); i++){
        //std::cout << "checking some contour " << std::endl;
	    cv::RotatedRect temp = cv::minAreaRect(cv::Mat(contours[i]));
	    cv::Point2f tempPts[4];
	    temp.points(tempPts);
	    double area = ((double) temp.size.width) * ((double) temp.size.height);
	    if (area < MIN_AREA) {
	        continue;
	    }
	    
	    //correctlySortPts(tempPts);

	    double realWidth = sqrt( abs(tempPts[2].x-tempPts[1].x)*abs(tempPts[2].x-tempPts[1].x) + abs(tempPts[2].y-tempPts[1].y)*abs(tempPts[2].y-tempPts[1].y) );
	    double realHeight = sqrt( abs(tempPts[0].x-tempPts[1].x)*abs(tempPts[0].x-tempPts[1].x) + abs(tempPts[0].y-tempPts[1].y)*abs(tempPts[0].y-tempPts[1].y) );
	    double aspectRatio = (double) realWidth / realHeight;
	    //std::cout << "Width: " << realWidth << "      Height: " << realHeight << std::endl;
	    //std::cout << "ratio: " << aspectRatio << "Angle: " << temp.angle << std::endl;

     	if ( (temp.angle > -45.0 && (aspectRatio < MIN_ASPECT_RATIO || aspectRatio > MAX_ASPECT_RATIO)) || 
     	    (temp.angle < -45.0 && (1.0/aspectRatio < MIN_ASPECT_RATIO || 1.0/aspectRatio > MAX_ASPECT_RATIO)) ){
     	    //std::cout << "ded ratio: " << aspectRatio << std::endl;
	        continue;
	    }
        rotatedRects.push_back(temp);
    }
}

void drawRotatedRects(cv::Mat &frame, std::vector<cv::RotatedRect> rotatedRects){
    for (int i = 0; i < rotatedRects.size(); i++) {
	    cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
	    cv::Point2f rectPoints[4];
	    rotatedRects[i].points(rectPoints);
	    //std::cout << "hi" << std::endl;
	    for (int j = 0; j < 4; j++) {
	       line(frame, rectPoints[j], rectPoints[(j+1)%4], color,1,8);
	    }
    }
}

bool compareRect(cv::RotatedRect r1, cv::RotatedRect r2) {	// returns true if r1 is lefter than r2
    cv::Point2f pts1[4];
    cv::Point2f pts2[4];
    r1.points(pts1);
    r2.points(pts2);
    return pts1[0].x+pts1[1].x+pts1[2].x+pts1[3].x < pts2[0].x+pts2[1].x+pts2[2].x+pts2[3].x;
}

bool findPairs(std::vector<cv::RotatedRect> rotatedRects, std::vector<std::pair<cv::RotatedRect, cv::RotatedRect> > &pairs) {
    if(rotatedRects.size() < 2) return false;
    for (int i = 0; i < rotatedRects.size() - 1; i++) {
	    cv::Point2f left[4];
	    rotatedRects[i].points(left);
	    cv::Point2f right[4];
	    rotatedRects[i+1].points(right);
	    double leftAngle = atan((left[0].y - left[1].y)/(left[1].x - left[0].x)) * 180 / PI;	// if this is actually left, result should be approx 75 deg
	    double rightAngle = atan((right[0].y - right[1].y)/(right[1].x - right[0].x)) * 180 / PI;	// if actuall right, result should be approx 105 deg
	    //if(rotatedRects[i].angle < -45.0) leftAngle = -leftAngle - 90;
	    //if(rotatedRects[i+1].angle < -45.0) rightAngle = -rightAngle - 90;
	    if (leftAngle >= LEFT_MIN_ANGLE && leftAngle <= LEFT_MAX_ANGLE && rightAngle >= RIGHT_MIN_ANGLE && rightAngle <= RIGHT_MAX_ANGLE) {
	    //std::cout << "left angle of a pair: " << leftAngle << "right angle of a pair: " << rightAngle << std::endl;
		//std::cout << "left min " << LEFT_MIN_ANGLE << " left max " << LEFT_MAX_ANGLE << " right min " << RIGHT_MIN_ANGLE << " right max " << RIGHT_MAX_ANGLE << std::endl;
	        std::pair <cv::RotatedRect, cv::RotatedRect> temp (rotatedRects[i], rotatedRects[i+1]);
	        pairs.push_back(temp);
	        i += 1; // not necessary but skips processsing right again
	    }
    }
    if (pairs.size() < 1) {
	return false;
    }
    return true;
}

void findThePair(std::vector<std::pair<cv::RotatedRect, cv::RotatedRect> > pairs, std::pair<cv::RotatedRect, cv::RotatedRect> &best){//pair) {
    double bestDist = CAM_RES_WIDTH;
    //std::pair<cv::RotatedRect, cv::RotatedRect> best;
    cv::Point2f leftPts[4];
	cv::Point2f rightPts[4];
    //for (std::pair<cv::RotatedRect, cv::RotatedRect> curr : pairs) {
    for(int i = 0; i<pairs.size(); i++){
        pairs[i].first.points(leftPts);
	    pairs[i].second.points(rightPts);

        double centerX = (leftPts[1].x + rightPts[2].x)/2.0;
	    double dist = abs(centerX - CAM_RES_WIDTH/2.0);
	    if (dist < bestDist) {
	        bestDist = dist;
	        best = pairs[i];
	    }
    }
    //std::cout << " best distance: " << bestDist << std::endl;
    //pair = best;
}

double calculateAngle(int x) {
    //std::cout << "x: " << x << "CAM_ANGLE: " << CAM_ANGLE << std::endl;
    return x*CAM_ANGLE/CAM_RES_WIDTH - CAM_ANGLE/2.0;
}

double calculateDistance(/*int tapeWidth, int tapeHeight,*/ int tapeBetweenDist) { //returns in feet
    //std::cout << "CAM_FOCAL_LEN: " << CAM_FOCAL_LEN << " tapeWidth: " << tapeWidth << std::endl;
    double rawDistance = CAM_FOCAL_LEN*BETWEEN_TAPES_DIST/tapeBetweenDist;//CAM_FOCAL_LEN*TAPE_WIDTH/tapeWidth;//CAM_FOCAL_LEN*TAPE_WIDTH/tapeHeight; //CAM_FOCAL_LEN*tapeHeight/TAPE_HEIGHT
    //std::cout << "raw dist " << rawDistance << std::endl;
    //std::cout << "blah:" << CAM_HEIGHT_OFFSET << std::endl;
    return sqrt(rawDistance*rawDistance - CAM_HEIGHT_OFFSET*CAM_HEIGHT_OFFSET);
}

double distanceToTapes(double deltax) {
    double scalefactor = deltax*DEGREES_TO_INCHES/CAM_FOCAL_WIDTH;
    return CAM_FOCAL_DIST/scalefactor;
}

int main(int argc, char **argv){
    std::string lifecam (argv[1]);
    bool show_desired = false;
    if (argc>2){
       show_desired = true;
    }
    int LIFECAM = lifecam[lifecam.length()-1]-'0';
    cv::VideoCapture cap = init(LIFECAM);
    
    cv::Mat cpuGreenMat; //frame = NULL
    cv::cuda::GpuMat gpuMat, gpuGreenMat;
    cv::cuda::GpuMat hsvGpuMat;
    cv::Mat cpuMat;
    cv::Mat hsvMat, greenMat;
    // above all had NULLs... was giving errors tho :(
    cap.set(CV_CAP_PROP_FRAME_WIDTH, CAM_RES_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_RES_HEIGHT);
    
	cv::Point2f pairLeftPts[4];
	cv::Point2f pairRightPts[4];
	double centerX;
	//double angle, distance;
	
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:1188"); //5808");
	int confl = 1;
	publisher.setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));	
    
    bool hasRects;
   
    while(true){
	    std::vector<std::vector<cv::Point>> contours;
	    std::vector<cv::Vec4i> hierarchy;
	    std::vector<cv::RotatedRect> rotatedRects;
	    std::vector<std::pair<cv::RotatedRect, cv::RotatedRect> > pairs;
	    std::pair<cv::RotatedRect, cv::RotatedRect> centerPair;
	    
	    /*readToGPUMat(cap, gpuMat);
            if(gpuMat.empty()) continue; // TODO: check lol*/
            
	    cap >> cpuMat;
        if (cpuMat.empty()) continue;
        //gpuMat.download()
        //cv::cuda::cvtColor(gpuMat, hsvGpuMat, cv::COLOR_BGR2HSV);
           
	    cv::cvtColor(cpuMat, hsvMat, cv::COLOR_BGR2HSV);
        cv::inRange(hsvMat, cv::Scalar(LOWER_GREEN_HUE, LOWER_GREEN_SAT, LOWER_GREEN_VAL), cv::Scalar(UPPER_GREEN_HUE,UPPER_GREEN_SAT,UPPER_GREEN_VAL), greenMat);
        //filterGreen(hsvGpuMat, gpuGreenMat);
    //  gpuGreenMat.download(cpuGreenMat);
    //  cpuGreenMat = gpuGreenMat; //gpu->cpu, find contours not faster in gpu and cuda does not have equiv
	
	    //cv::GaussianBlur(greenMat, greenMat, cv::Size(7,7), 2, 2);
        findRectangles(greenMat, contours, hierarchy, rotatedRects);
        //std::cout << "rotated rects size: " << rotatedRects.size() << std::endl;
       /* for (int i = 0; i < contours.size(); i++) {
            cv::Scalar colorlol = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
            //cv::drawContours(cpuMat, contours, i, colorlol, 2, cv::LINE_8, hierarchy, 0);
        }*/
        
	    drawRotatedRects(cpuMat, rotatedRects);
	    std::sort(rotatedRects.begin(), rotatedRects.end(), compareRect);
	    hasRects = findPairs(rotatedRects, pairs);
	    //std::cout << hasRects << std::endl;
	    if(hasRects){
	        findThePair(pairs, centerPair);
            centerPair.first.points(pairLeftPts);
            centerPair.second.points(pairRightPts);
            //std::cout << "greetings: " << pairLeftPts[0].x << std::endl;
            centerX = ((double) pairLeftPts[0].x + pairLeftPts[1].x+pairLeftPts[2].x+pairLeftPts[3].x+
                       pairRightPts[0].x+pairRightPts[1].x+pairRightPts[2].x+pairRightPts[3].x) /8.0;
                      
            std::cout << "centerX "<<centerX<< " Angle: " << calculateAngle(centerX) << std::endl;
            //" Distance: " <<  calculateDistance((pairRightPts[0].x+pairRightPts[1].x+pairRightPts[2].x+pairRightPts[3].x)/4.0 - (pairLeftPts[0].x + pairLeftPts[1].x+pairLeftPts[2].x+pairLeftPts[3].x)/4.0) << std::endl; 
            
            double deltax = pairRightPts[3].x - pairLeftPts[1].x;
            std::cout << "distance: " << distanceToTapes(deltax) << std::endl;
            std::string gap = " ";
            std::string message = std::to_string(calculateAngle(centerX)) + gap + std::to_string(distanceToTapes(deltax));
            std::cout << message << std::endl;
            zmq_send(publisher, &message, sizeof(message), 0);
            
            
            //TODO SKETCH, calculating distance based on right tape??
            //calculateDistance((pairRightPts[0].x+pairRightPts[1].x+pairRightPts[2].x+pairRightPts[3].x)/4.0 - (pairLeftPts[0].x + pairLeftPts[1].x+pairLeftPts[2].x+pairLeftPts[3].x)/4.0
            //calculateDistance(pairRightPts[2].x-pairRightPts[1].x, pairRightPts[0].y-pairRightPts[1].y) //width/height

	    }         
	    if (show_desired){    
           imshow("original image", cpuMat); //must be cpu mat to imshow
           imshow("green mask", greenMat);
 
           char c=(char)cv::waitKey(25);
           if (c==27) break;
        }
     
    }
    cap.release();
    //cv::destroyAllWindows();
}
