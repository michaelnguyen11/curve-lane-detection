extern const int    DEBUG_LANE = 0;
extern const int    DEBUG_HOUGH = 0;

#include "DetectLanes.h"

void DetectLanes(const cv::Mat &laneMat,
                 const LaneDetectorConf &laneDetectorConf,
                 const int &offsetX,
                 const int &offsetY,
                 std::vector<cv::Vec2f> &hfLanes,
                 std::vector<cv::Vec2f> &pHfLanes,
                 const int &laneKalmanIdx,
                 const double &isChangeLane)
{
    cv::Mat roiMat = laneMat(cv::Rect(offsetX, offsetY, laneMat.cols-2*offsetX, laneMat.rows-offsetY));

    /* Edge Detection */
    FilterLanes(roiMat);

    /*
     * Utilize the last tracked lane position to predict the current ones
     * If change lanes, the ROI region will not work;
     * It should redefine the ROI
     * First Left, Second Right
     */
    cv::Mat lLaneMask = cv::Mat(roiMat.rows, roiMat.cols, CV_8U, cv::Scalar::all(0));
    cv::Mat rLaneMask = cv::Mat(roiMat.rows, roiMat.cols, CV_8U, cv::Scalar::all(0));

    int isROI = 0;// Indicator whether Hough execute in the ROI
    if(laneKalmanIdx > 1 && isChangeLane == 0 && pHfLanes.size() == 2)
    {
        // std::cout << "Set Up ROI" << std::endl;

        /* Predict the current possible area from last tracked range */
        const int proStartXRange = laneDetectorConf.top_range; //!pixel
        const int proEndXRange = laneDetectorConf.bottom_range; //!pixel
        // std::cout << proStartXRange << std::endl;
        // std::cout << proEndXRange << std::endl;

        std::vector<Lane> lastLanes;
        HfLanetoLane(roiMat, pHfLanes, lastLanes);

        /* The Coordinate reversed to the normal xy coordinates */
        int lStartX = cvRound(lastLanes[0].startPoint.x);
        double lY = lastLanes[0].endPoint.y;
        int lEndX = 0;
        if(pHfLanes[0][1] > 0) {
            lEndX = cvRound((lY - roiMat.rows) * tan(pHfLanes[0][1]));
        } else {
            lEndX = roiMat.cols - cvRound((lY - roiMat.rows) * -tan(pHfLanes[0][1]));
        }

        /* Reversed to normal xy coordinates & Move to right edge of image */
        int rStartX = cvRound(lastLanes[1].startPoint.x);
        double rY = lastLanes[1].endPoint.y;
        int rEndX = roiMat.cols - cvRound((rY - roiMat.rows) * tan(CV_PI - pHfLanes[1][1]));
        if(pHfLanes[1][1] > 0) {
            rEndX = cvRound((rY - roiMat.rows) * tan(pHfLanes[1][1]));
        } else {
            rEndX = roiMat.cols - cvRound((rY - roiMat.rows) * -tan(pHfLanes[1][1]));
        }

        cv::Point lPoly[1][4], rPoly[1][4];
        int npoly[] = {4};

        /* Left Side */
        lPoly[0][0] = cv::Point(lStartX+proStartXRange, 0);
        lPoly[0][1] = cv::Point(lStartX-proStartXRange, 0);
        lPoly[0][2] = cv::Point(lEndX-proEndXRange, roiMat.rows);
        lPoly[0][3] = cv::Point(lEndX+proEndXRange, roiMat.rows);
        const cv::Point *plPoly[1] = {lPoly[0]};
        cv::fillPoly(lLaneMask, plPoly, npoly, 1, cv::Scalar(255));

        /* Right Side */
        rPoly[0][0] = cv::Point(rStartX+proStartXRange, 0);
        rPoly[0][1] = cv::Point(rStartX-proStartXRange, 0);
        rPoly[0][2] = cv::Point(rEndX-proEndXRange, roiMat.rows);
        rPoly[0][3] = cv::Point(rEndX+proEndXRange, roiMat.rows);
        const cv::Point *prPoly[1] = {rPoly[0]};
        cv::fillPoly(rLaneMask, prPoly, npoly, 1, cv::Scalar(255));

        isROI = 1;
    }
    /* Lane Extraction */
    GetHfLanes(roiMat, laneDetectorConf, hfLanes, lLaneMask, rLaneMask, isROI, isChangeLane);
}

void FilterLanes(Mat &laneMat)
{
    cv::Mat mat1, mat2, mat3, finalMat;
    cv::Sobel(laneMat, mat1, CV_8U, 1, 0, 3);
    cv::Sobel(mat1, mat2, CV_8U, 0, 1, 3);
    cv::medianBlur(mat2, mat3, 3);
    cv::threshold(mat3, finalMat, 100, 255, cv::THRESH_BINARY);
    //cvCanny demands the 8UC1(0~255)
    cv::Canny(laneMat, finalMat, 10, 100, 3);

    finalMat.copyTo(laneMat);
}

void GetHfLanes(cv::Mat &laneMat,
                const LaneDetectorConf &laneDetectorConf,
                std::vector<cv::Vec2f> &hfLanes,
                const cv::Mat &lLaneMask,
                const cv::Mat &rLaneMask,
                const int &isROI, const int &isChangeLane)
{
    std::vector<cv::Vec2f> hfLanesCandi;

    std::vector<cv::Vec2f> leftHfLanesCandi;
    std::vector<cv::Vec2f> leftHfLanes;
    std::vector<cv::Vec2f> rightHfLanesCandi;
    std::vector<cv::Vec2f> rightHfLanes;

    std::vector<cv::Point2d> lLanePts;
    std::vector<cv::Point2d> rLanePts;

    //std::cout << "isROI: " << isROI << std::endl;

    if(!isROI)
    {
       /*
        * Detect the lane in whole image
        * \param isChangeLane indicates the direction for ROI
        */
        cv::HoughLines(laneMat, hfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0);

        /* Adjust the parameters when theta > CV_PI/2 */
        for(size_t i = 0; i < hfLanesCandi.size(); i++) {
            hfLanesCandi[i][1] = hfLanesCandi[i][0] > 0 ? hfLanesCandi[i][1] : hfLanesCandi[i][1]-(float)CV_PI;
            hfLanesCandi[i][0] = std::abs(hfLanesCandi[i][0]);
        }


        for (size_t i = 0; i < hfLanesCandi.size(); i++)
        {
            double thetaCandi = hfLanesCandi[i][1];
            double rhoCandi = hfLanesCandi[i][0];

            //! Solution considering for changing lanes
            if(rhoCandi > laneDetectorConf.rhoMin) {
                if(isChangeLane == 0) {
                    //! car keep in the center
                    if( thetaCandi >= 0 && thetaCandi < laneDetectorConf.thetaMax)
                        leftHfLanes.push_back(hfLanesCandi.at(i)); // line in region from 0 to 90 degree
                    else if(thetaCandi > -laneDetectorConf.thetaMax && thetaCandi < 0)
                        rightHfLanes.push_back(hfLanesCandi.at(i));// line in region from -90 to 0 degree
                }
                else if(isChangeLane == 1) {
                    //!car towards right
                    if(thetaCandi <= laneDetectorConf.thetaMin && thetaCandi > -laneDetectorConf.thetaMin)
                        leftHfLanes.push_back(hfLanesCandi.at(i));
                    else if(thetaCandi < -laneDetectorConf.thetaMin && thetaCandi >  -laneDetectorConf.thetaMax)
                        rightHfLanes.push_back(hfLanesCandi.at(i));
                }
                else if(isChangeLane == -1) {
                    //!car towards left
                    if(thetaCandi >= laneDetectorConf.thetaMin && thetaCandi < laneDetectorConf.thetaMax)
                        leftHfLanes.push_back(hfLanesCandi.at(i));
                    else if(thetaCandi < laneDetectorConf.thetaMin && thetaCandi > -laneDetectorConf.thetaMin)
                        rightHfLanes.push_back(hfLanesCandi.at(i));
                }
            }
        }

        /*
         * Sort in sequence
         * Find correct line from list of HoughLine
         * Matched lines may be the middle of all candidate lines
         */
        int lanesNum;
        float rho, theta;
        if (!leftHfLanes.empty()) {
            sort(leftHfLanes.begin(), leftHfLanes.end(), sort_smaller);
            lanesNum = (int)leftHfLanes.size();
            rho = leftHfLanes.at(lanesNum/2)[0];
            theta = leftHfLanes.at(lanesNum/2)[1];
            hfLanes.push_back(cv::Vec2f(rho, theta));
        }
        if (!rightHfLanes.empty()) {
            sort(rightHfLanes.begin(), rightHfLanes.end(), sort_smaller);
            lanesNum = (int)rightHfLanes.size();
            rho = rightHfLanes.at(lanesNum/2)[0];
            theta = rightHfLanes.at(lanesNum/2)[1];
            hfLanes.push_back(cv::Vec2f(rho, theta));
        }


        /* Draw the lane candidates of Hough transform */
        if (DEBUG_HOUGH) {
            std::vector<Lane> drawLanes;
            cv::Mat laneMatRGB;
            cv::cvtColor(laneMat, laneMatRGB, cv::COLOR_GRAY2RGB);

            if (!leftHfLanes.empty()) {
                HfLanetoLane(laneMat, leftHfLanes, drawLanes);
                // cv::threshold(laneMatRGB, laneMatRGB, 0, 255, 1);//draw
                for(size_t i = 0; i < drawLanes.size(); i++) {
                    cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0,0,200), 1);
                }
            }

            if(!rightHfLanes.empty()) {
                drawLanes.clear();
                HfLanetoLane(laneMat, rightHfLanes, drawLanes);
                for(size_t i = 0; i < drawLanes.size(); i++)
                    cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(200,0,0), 1);
            }

             //imShowSub("Hough Lanes Candi", laneMatRGB,  WIN_COLS, WIN_ROWS, 5);
            //cv::moveWindow("Hough Lanes Candi", 790, 0);
        }
    }
    else
    {
        /* Detect the lane in the ROI, set by the tracked lane */
        //std::cout << "Detect Lanes in ROI" << std::endl;

        /* Threshold of distance value to fix the most fitted hfLane */
        const int TH_DIS = 10;

        /* Left ROI */
        cv::Mat lLaneMat = laneMat & lLaneMask;
        cv::HoughLines(lLaneMat, leftHfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0);



        for(size_t i = 0; i < leftHfLanesCandi.size(); i++) {
            leftHfLanesCandi[i][1] = leftHfLanesCandi[i][0] > 0 ? leftHfLanesCandi[i][1] : leftHfLanesCandi[i][1]-(float)CV_PI;
            leftHfLanesCandi[i][0] = std::abs(leftHfLanesCandi[i][0]);

            if(leftHfLanesCandi[i][0] > laneDetectorConf.rhoMin &&
                leftHfLanesCandi[i][1] < laneDetectorConf.thetaMax &&
                leftHfLanesCandi[i][1] > -laneDetectorConf.thetaMin)
                leftHfLanes.push_back(leftHfLanesCandi[i]);
        }

        //ExtractPointSet(lLaneMat, lLanePts);

        if(!leftHfLanes.empty()) {
            std::vector<Lane> leftLanes;
            HfLanetoLane(laneMat, leftHfLanes, leftLanes);

            int pos = 0;
            int maxFitNum = 0;
            //! Lane Candidates
            for(size_t i = 0; i < leftLanes.size(); i++)
            {
                double xIntercept = leftLanes[i].startPoint.x;
                double yIntercept = leftLanes[i].endPoint.y;
                int fitNum = 0;

                //! Points
                for(size_t j = 0; j < lLanePts.size(); j++) {
                    double dis = std::abs(xIntercept*lLanePts[j].y + yIntercept*lLanePts[j].x - xIntercept*yIntercept) / (xIntercept * xIntercept + yIntercept * yIntercept);

                    if(dis  < TH_DIS)
                        fitNum ++;
                }

                if (maxFitNum < fitNum) {
                    maxFitNum = fitNum;
                    pos = i;
                }
            }

            hfLanes.push_back(leftHfLanes[pos]);
        }


        //! Right ROI
        cv::Mat rLaneMat = laneMat & rLaneMask;
        cv::HoughLines(rLaneMat, rightHfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0);
        for(size_t i = 0; i < rightHfLanesCandi.size(); i++) {
            rightHfLanesCandi[i][1] = rightHfLanesCandi[i][0] > 0 ? rightHfLanesCandi[i][1] : rightHfLanesCandi[i][1]-(float)CV_PI;
            rightHfLanesCandi[i][0] = std::abs(rightHfLanesCandi[i][0]);

            if(rightHfLanesCandi[i][0] > laneDetectorConf.rhoMin &&
               rightHfLanesCandi[i][1] > -laneDetectorConf.thetaMax &&
               rightHfLanesCandi[i][1] < laneDetectorConf.thetaMin)
                rightHfLanes.push_back(rightHfLanesCandi[i]);
        }

        //ExtractPointSet(rLaneMat, rLanePts);

        if(!rightHfLanes.empty())
        {
            std::vector<Lane> rightLanes;
            HfLanetoLane(laneMat, rightHfLanes, rightLanes);

            int pos = 0;
            int maxFitNum = 0;
            //! Lane Candidates
            for(int i = 0; i < (int)rightLanes.size(); i++)
            {
                double xIntercept = rightLanes[i].startPoint.x;
                double yIntercept = rightLanes[i].endPoint.y;
                int fitNum = 0;
                //! Points
                for(int j = 0; j < (int)rLanePts.size(); j++)
                {
                    double dis = std::abs(xIntercept*rLanePts[j].y + yIntercept*rLanePts[j].x - xIntercept*yIntercept) / (xIntercept * xIntercept + yIntercept * yIntercept);

                    if(dis  < TH_DIS)
                        fitNum ++;
                }

                if (maxFitNum < fitNum)
                {
                    maxFitNum = fitNum;
                    pos = i;
                }
            }
            hfLanes.push_back(rightHfLanes[pos]);
        }

        //! Draw the lane candidates of Hough transform
        if (DEBUG_HOUGH) {
            std::vector<Lane> drawLanes;
            cv::Mat laneMatRGB;
            cv::cvtColor(laneMat, laneMatRGB, cv::COLOR_GRAY2RGB);

            if (!leftHfLanes.empty()) {
                HfLanetoLane(laneMat, leftHfLanes, drawLanes);
                // cv::threshold(laneMatRGB, laneMatRGB, 0, 255, 1);//1
                for(size_t i = 0; i < drawLanes.size(); i++)
                    cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0,0,200), 1);
            }
            if(!rightHfLanes.empty()) {
                drawLanes.clear();
                HfLanetoLane(laneMat, rightHfLanes, drawLanes);
                for(size_t i = 0; i < drawLanes.size(); i++)
                    cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(200,0,0), 1);
            }
             //imShowSub("Hough Lanes Candi", laneMatRGB,  WIN_COLS, WIN_ROWS, 5);
        }

        /* Debug: draw the ROI */
        if(DEBUG_HOUGH) {
            cv::Mat roiMat = lLaneMat + rLaneMat;
            //cv::imshow("roiMat", roiMat);
        }
    }
}

void HfLanetoLane(const cv::Mat &laneMat, const std::vector<cv::Vec2f> &hfLanes, std::vector<Lane> &lanes)
{
    lanes.clear();

    for( size_t i = 0; i < hfLanes.size(); i++)
    {
        cv::Point2d Pt1, Pt2;
        double rho = hfLanes.at(i)[0];
        double theta = hfLanes.at(i)[1];

        if( theta > 0 && theta < CV_PI/2 )
        {
            //! Pt1
            Pt1.y = 0;
            Pt1.x = rho/cos(theta);
            // if(Pt1.x > laneMat.cols)
            // {
            //    Pt1.y = (Pt1.x - laneMat.cols) / tan(theta);
            //    Pt1.x = laneMat.cols;

            // }
            //! Pt2
            Pt2.x = 0;
            Pt2.y = rho/sin(theta);
            // if(Pt2.y > laneMat.rows)
            // {
            //    Pt2.x = (Pt2.y - laneMat.rows) * tan(theta);
            //    Pt2.y = laneMat.rows;
            // }
        }
        else if(theta == CV_PI/2)
        {
            //! Pt1
            Pt1.x = laneMat.cols;
            Pt1.y = rho;
            //! Pt2
            Pt2.x = 0;
            Pt2.y = rho;

        }
        else if(theta == 0)
        {
            //!Pt1
            Pt1.y = 0;
            Pt1.x = std::abs(rho);
            //!Pt2
            Pt2.y = laneMat.rows;
            Pt2.x = std::abs(rho);
        }
        else if(theta > -CV_PI/2 && theta < 0)
        {
            //!Pt1
            Pt1.y = 0;
            Pt1.x = rho / cos(theta);
//                if(Pt1.x < 0)
//                {
//                    Pt1.y = std::abs(Pt1.x) * tan(theta - CV_PI/2);
//                    Pt1.x = 0;
//                }

            //!Pt2
            Pt2.x = laneMat.cols;
            Pt2.y = (laneMat.cols - Pt1.x) / -tan(theta);
//                if(Pt2.y > laneMat.rows )
//                {
//                    Pt2.x = laneMat.rows - (Pt2.y - laneMat.rows) * tan(CV_PI - theta);
//                    Pt2.y = laneMat.rows;
//                }
        }

        //std::cout << "ToLane: " << Pt1 << "," << Pt2 << std::endl;
        Lane tempLane = {Pt1, Pt2};//StartPt, EndPt
        lanes.push_back(tempLane);
    }
}

void DrawPreROI(cv::Mat &laneMat,
                const int offsetX,
                const int offsetY,
                const std::vector<cv::Vec2f> &pHfLanes,
                const int &laneKalmanIdx,
                const int &isChangeLane,
                const LaneDetectorConf &laneDetectorConf)
{
    cv::Point2d  vp, corner_l, corner_r;
    std::vector<Lane> pLanes;

    if(laneKalmanIdx > 1 && !pHfLanes.empty()) {
        //! ROI from predicted lanes
        GetMarkerPoints(laneMat, pHfLanes, vp, corner_l, corner_r, offsetX, offsetY);

        //! Kalman Predicted Lanes in Next Frame
        cv::circle(laneMat, cv::Point2d(vp.x, vp.y), 1, CV_RGB(255,0,0));
        cv::line(laneMat, cv::Point2d(vp.x, vp.y), cv::Point2d(corner_l.x, corner_l.y), CV_RGB(200, 0, 200), 2);
        cv::line(laneMat, cv::Point2d(vp.x, vp.y), cv::Point2d(corner_r.x, corner_r.y), CV_RGB(0, 200, 200), 2);

        if (!isChangeLane) {
            const int proStartXRange = laneDetectorConf.top_range;
            const int proEndXRange = laneDetectorConf.bottom_range;
            HfLanetoLane(laneMat, pHfLanes, pLanes);

            //! The Coordinate reversed to normal xy coordinates & Move to left edge of image
            int lStartX = cvRound(pLanes[0].startPoint.x);
            double lY = pLanes[0].endPoint.y;
            int lEndX;
            if(pHfLanes[0][1] > 0) {
                lEndX = cvRound((lY - (laneMat.rows-offsetY)) * tan(pHfLanes[0][1]));
            } else {
                lEndX = laneMat.cols - cvRound((lY - (laneMat.rows-offsetY)) * -tan(pHfLanes[0][1]));
            }

            cv::line(laneMat, cv::Point(lStartX + proStartXRange + offsetX, offsetY),
                    cv::Point(lStartX - proStartXRange + offsetX, offsetY), CV_RGB(100, 0, 100), 2);
            cv::line(laneMat, cv::Point(lStartX - proStartXRange + offsetX, offsetY),
                    cv::Point(lEndX - proEndXRange + offsetX, laneMat.rows), CV_RGB(100, 0, 100), 2);
            cv::line(laneMat, cv::Point(lEndX - proEndXRange + offsetX, laneMat.rows),
                    cv::Point(lEndX + proEndXRange + offsetX, laneMat.rows), CV_RGB(100, 0, 100), 2);
            cv::line(laneMat, cv::Point(lEndX + proEndXRange + offsetX, laneMat.rows),
                    cv::Point(lStartX + proStartXRange + offsetX, offsetY), CV_RGB(100, 0, 100), 2);


            //! Reversed to normal xy coordinates & Move to right edge of image
            int rStartX = cvRound(pLanes[1].startPoint.x);
            double rY = pLanes[1].endPoint.y;
            int rEndX;
            if(pHfLanes[1][1] > 0) {
                rEndX = cvRound((rY - (laneMat.rows-offsetY)) * tan(pHfLanes[1][1]));
            } else {
                rEndX = laneMat.cols - cvRound((rY - (laneMat.rows-offsetY)) * -tan(pHfLanes[1][1]));
            }

            cv::line(laneMat, cv::Point(rStartX + proStartXRange + offsetX, offsetY),
                            cv::Point(rStartX - proStartXRange + offsetX, offsetY), CV_RGB(0, 100, 100), 2);
            cv::line(laneMat, cv::Point(rStartX - proStartXRange + offsetX, offsetY),
                            cv::Point(rEndX - proEndXRange + offsetX, laneMat.rows), CV_RGB(0, 100, 100), 2);
            cv::line(laneMat, cv::Point(rEndX - proEndXRange + offsetX, laneMat.rows),
                            cv::Point(rEndX + proEndXRange + offsetX, laneMat.rows), CV_RGB(0, 100, 100), 2);
            cv::line(laneMat, cv::Point(rEndX + proEndXRange + offsetX, laneMat.rows),
                            cv::Point(rStartX + proStartXRange + offsetX, offsetY), CV_RGB(0, 100, 100), 2);

            // //! Fill the poly
            // int npoly[] = {4};
            // //! Left Side
            // cv::Point lPoly[1][4];
            // lPoly[0][0] = cv::Point(lStartX+proStartXRange, offsetY);
            // lPoly[0][1] = cv::Point(lStartX-proStartXRange, offsetY);
            // lPoly[0][2] = cv::Point(lEndX-proEndXRange, laneMat.rows);
            // lPoly[0][3] = cv::Point(lEndX+proEndXRange, laneMat.rows);
            // const cv::Point *plPoly[1] = {lPoly[0]};
            // cv::fillPoly(laneMat, plPoly, npoly, 1, CV_RGB(0, 100, 100));

            // //! Right Side
            // cv::Point rPoly[1][4];
            // rPoly[0][0] = cv::Point(rStartX+proStartXRange, offsetY);
            // rPoly[0][1] = cv::Point(rStartX-proStartXRange, offsetY);
            // rPoly[0][2] = cv::Point(rEndX-proEndXRange, laneMat.rows);
            // rPoly[0][3] = cv::Point(rEndX+proEndXRange, laneMat.rows);
            // const cv::Point *prPoly[1] = {rPoly[0]};
            // cv::fillPoly(laneMat, prPoly, npoly, 1, CV_RGB(0, 100, 100));

            // cv::Point poly[1][3];
            // poly[0][0] = cv::Point(vp.x, vp.y);
            // poly[0][1] = cv::Point(corner_l.x, corner_l.y);
            // poly[0][2] = cv::Point(corner_r.x, corner_r.y);
            // const cv::Point *ppoly[1] = {poly[0]};
            // int npoly[] = {3};
            // cv::fillPoly(laneMat, ppoly, npoly, 1, cv::Scalar(255, 0, 0));
        }
    }
}

void GetMarkerPoints(const cv::Mat &laneMat, const std::vector<cv::Vec2f> &hfLanes,
                    cv::Point2d &vp, cv::Point2d &corner_l, cv::Point2d &corner_r, const int offsetX, const int offsetY)
{
    std::vector<Lane> lanes;

    HfLanetoLane(laneMat, hfLanes, lanes);

    std::vector<double> k;// slope
    for (std::vector<Lane>::const_iterator iter = lanes.begin(); iter != lanes.end(); ++iter )
    {
        k.push_back((iter->startPoint.y - iter->endPoint.y)/(iter->startPoint.x - iter->endPoint.x));
    }


    if(!k.empty()) {
        vp.x = (k.at(0)*lanes.at(0).startPoint.x - k.at(1)*lanes.at(1).startPoint.x
                - lanes.at(0).startPoint.y + lanes.at(1).startPoint.y) / (k.at(0)-k.at(1)) + offsetX;

        vp.y = (k.at(0)*lanes.at(1).startPoint.y - k.at(1)*lanes.at(0).startPoint.y
                + k.at(0)*k.at(1)*lanes.at(0).startPoint.x - k.at(0)*k.at(1)*lanes.at(1).startPoint.x) / (k.at(0)-k.at(1)) + offsetY;

        corner_l.y = laneMat.rows;
        corner_r.y = laneMat.rows;
        corner_l.x = (1/k.at(0))*(corner_l.y-vp.y)+vp.x;
        corner_r.x = (1/k.at(1))*(corner_r.y-vp.y)+vp.x;
    }
}

bool sort_smaller(const cv::Vec2f &lane1, const cv::Vec2f &lane2){
    return lane1[1] > lane2[1];
}

void GetLateralOffset(const cv::Mat &laneMat,const double &leftPoint,const double &rightPoint,double &lateralOffset)
{
    // std::cout << "LeftPoint :" << leftPoint << " RightPoint :" << rightPoint << std::endl;
    double laneCenter = (leftPoint + rightPoint)/2;
    double carCenter = (laneMat.cols)/2;//point of car center seen as mid of x(point) in image
    double carOffset = carCenter - laneCenter;//- left + right

    double distance = std::abs(leftPoint - rightPoint);//lane width in pixel
    double LANE_WIDTH = 3.5;
    double CAR_WIDTH = 1.7;
    double maxLateralOffset = (CAR_WIDTH/LANE_WIDTH) * distance/2.0;
    lateralOffset = carOffset / maxLateralOffset;
    lateralOffset = lateralOffset > 1 ? 1 : lateralOffset;
    lateralOffset = lateralOffset < -1 ? -1 : lateralOffset;

}
