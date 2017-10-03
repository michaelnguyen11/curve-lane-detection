#ifndef PROCESSINGLANE_H
#define PROCESSINGLANE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <vector>
#include <iterator>
#include <opencv2/opencv.hpp>

#include "DetectLanes.h"
#include "Config.h"

using namespace std;
using namespace cv;

class ProcessingLane
{
public:
    ProcessingLane(Mat &laneMat);
    void InitlaneDetectorConf(const cv::Mat &laneMat, LaneDetectorConf &laneDetectorConf);

private:
    KalmanFilter laneKalmanFilter;
    Mat laneKalmanMeasureMat;
    int laneKalmanIdx;
    vector<cv::Vec2f> hfLanes;
    vector<cv::Vec2f> lastHfLanes;
    const double YAW_ANGLE = 0;
    const double PITCH_ANGLE = 0.1;
    std::vector<cv::Vec2f> postHfLanes;
    double lateralOffset;
    int isChangeLane;
    const double COEF = 0.75;

};

#endif // PROCESSINGLANE_H
