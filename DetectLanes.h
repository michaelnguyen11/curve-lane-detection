#ifndef DETECTLANES_H
#define DETECTLANES_H
#include <iostream>
#include "SharedImageBuffer.h"
#include "Config.h"

using namespace std;

void DetectLanes(const cv::Mat &laneMat,
                 const LaneDetectorConf &laneDetectorConf,
                 const int &offsetX,
                 const int &offsetY,
                 std::vector<cv::Vec2f> &hfLanes,
                 std::vector<cv::Vec2f> &pHfLanes,
                 const int &laneKalmanIdx,
                 const double &isChangeLane);
void FilterLanes(cv::Mat &laneMat);
void GetHfLanes(cv::Mat &laneMat,
                const LaneDetectorConf &laneDetectorConf,
                std::vector<cv::Vec2f> &hfLanes,
                const cv::Mat &lLaneMask,
                const cv::Mat &rLaneMask,
                const int &isROI, const int &isChangeLane);
void GetLateralOffset(const cv::Mat &laneMat,const double &leftPoint,const double &rightPoint,double &lateralOffset);
//void InitlaneDetectorConf(const cv::Mat &laneMat, LaneDetectorConf &laneDetectorConf, const int database);
void HfLanetoLane(const cv::Mat &laneMat, const std::vector<cv::Vec2f> &hfLanes, std::vector<Lane> &lanes);

void DrawPreROI(cv::Mat &laneMat,
                const int offsetX,
                const int offsetY,
                const std::vector<cv::Vec2f> &pHfLanes,
                const int &laneKalmanIdx,
                const int &isChangeLane,
                const LaneDetectorConf &laneDetectorConf);

void GetMarkerPoints(const cv::Mat &laneMat, const std::vector<cv::Vec2f> &hfLanes,
                    cv::Point2d &vp, cv::Point2d &corner_l, cv::Point2d &corner_r, const int offsetX, const int offsetY);
bool sort_smaller(const cv::Vec2f &lane1, const cv::Vec2f &lane2);
#endif // DETECTLANES_H
