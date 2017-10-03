#include "ProcessingLane.h"

ProcessingLane::ProcessingLane(Mat &laneMat)
{
    const int WIDTH = laneMat.cols;
    const int HEIGHT = laneMat.rows;

    resize(laneMat, laneMat, Size(cvRound(WIDTH), cvRound(HEIGHT)), INTER_AREA);

    Mat grayMat = Mat(cvRound(HEIGHT), cvRound(WIDTH), CV_8UC1);
    cvtColor(laneMat, grayMat, COLOR_BGR2GRAY);

    vector<Lane> lanes, postLanes;
    cv::Point2d vanishPt, leftCornerPt, rightCornerPt;
    const int offsetX = cvRound(laneMat.cols * YAW_ANGLE);
    const int offsetY = cvRound(laneMat.rows * PITCH_ANGLE);

    LaneDetectorConf laneDetectorConf;
    InitlaneDetectorConf(laneMat, laneDetectorConf);

    hfLanes.clear();

    DrawPreROI(laneMat, offsetX, offsetY, postHfLanes, laneKalmanIdx, isChangeLane, laneDetectorConf);

    DetectLanes(grayMat, laneDetectorConf, offsetX, offsetY, hfLanes, postHfLanes, laneKalmanIdx, isChangeLane);

    //! Draw the detected lanes
    if (!hfLanes.empty()) {
        HfLanetoLane(laneMat, hfLanes, lanes);

        for (std::vector<Lane>::const_iterator iter = lanes.begin(); iter != lanes.end(); ++iter) {
            cv::line(laneMat, cv::Point2d(iter->startPoint.x+offsetX, iter->startPoint.y+offsetY),
                     cv::Point2d(iter->endPoint.x+offsetX, iter->endPoint.y+offsetY), CV_RGB(255, 255, 0), 3);
        }

        if(hfLanes.size() == 2) {
            // std::cout << "Update LO in Detection" << std::endl;
            GetMarkerPoints(laneMat, hfLanes, vanishPt, leftCornerPt, rightCornerPt, offsetX, offsetY);
            GetLateralOffset(laneMat, leftCornerPt.x, rightCornerPt.x, lateralOffset);
            // std::cout << "Detection >> LO: "<< lateralOffset << std::endl;
            cv::circle(laneMat, vanishPt, 4, CV_RGB(100, 100 , 0), 2);
        }
    }
}
void ProcessingLane::InitlaneDetectorConf(const cv::Mat &laneMat, LaneDetectorConf &laneDetectorConf)
{
    /* Parameters of configuration of camera */
    laneDetectorConf.m = laneMat.rows * COEF;    //Rows (height of Image)
    laneDetectorConf.n = laneMat.cols * COEF;     //Columns (width of Image)
    laneDetectorConf.h = 1.15;              //Distance of camera above the ground (meters)
    laneDetectorConf.alphaTot = atan(3/12.5); //HALF viewing angle

        //! \param 6.7 for lane(data_130326)
        //! \param 5.5 for lane(data_121013)
        // laneDetectorConf.theta0 = CV_PI*(5.5/180);   //Camera tilted angle below the horizontal(positive)

        //! \params for lane (data_130710)
    laneDetectorConf.theta0 = CV_PI * (8.5/180.0); //the pitch angle

    laneDetectorConf.kernelWidth = 2;
    laneDetectorConf.kernelHeight = 2;

    laneDetectorConf.groupingType = 1;

    laneDetectorConf.rhoMin  = 30;
    laneDetectorConf.rhoStep = 1;

    laneDetectorConf.thetaStep = CV_PI/180;

    laneDetectorConf.thetaMin = CV_PI * 0.25;//45 degree
    laneDetectorConf.thetaMax = CV_PI * 0.36; //72 degree
    laneDetectorConf.top_range = 20;
    laneDetectorConf.bottom_range = 70;

    laneDetectorConf.vpTop = laneMat.rows * 0.2 * COEF;
    laneDetectorConf.vpBottom = laneMat.rows * 0.6 * COEF;
    laneDetectorConf.distCornerMin = laneMat.cols * 0.2 * COEF;
    laneDetectorConf.distCornerMax = laneMat.cols * 0.5 * COEF;

}//end InitlaneDetectorConf
