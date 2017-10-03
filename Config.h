#ifndef CONFIG_H
#define CONFIG_H

#include "opencv2/opencv.hpp"

// FPS statistics queue lengths
#define PROCESSING_FPS_STAT_QUEUE_LENGTH    32
#define CAPTURE_FPS_STAT_QUEUE_LENGTH       32

// Image buffer size
#define DEFAULT_IMAGE_BUFFER_SIZE           1
// Drop frame if image/frame buffer is full
#define DEFAULT_DROP_FRAMES                 false
// Thread priorities
#define DEFAULT_CAP_THREAD_PRIO             QThread::HighPriority
#define DEFAULT_PROC_THREAD_PRIO            QThread::HighPriority

// IMAGE PROCESSING
// Smooth
#define DEFAULT_SMOOTH_TYPE                 0 // Options: [BLUR=0,GAUSSIAN=1,MEDIAN=2]
#define DEFAULT_SMOOTH_PARAM_1              3
#define DEFAULT_SMOOTH_PARAM_2              3
#define DEFAULT_SMOOTH_PARAM_3              0
#define DEFAULT_SMOOTH_PARAM_4              0
// Dilate
#define DEFAULT_DILATE_ITERATIONS           1
// Erode
#define DEFAULT_ERODE_ITERATIONS            1
// Flip
#define DEFAULT_FLIP_CODE                   0 // Options: [x-axis=0,y-axis=1,both axes=-1]
// Canny
#define DEFAULT_CANNY_THRESHOLD_1           10
#define DEFAULT_CANNY_THRESHOLD_2           00
#define DEFAULT_CANNY_APERTURE_SIZE         3
#define DEFAULT_CANNY_L2GRADIENT            false

struct ThreadStatisticsData{
    int averageFPS;
    int nFramesProcessed;
};

typedef struct _LaneDetectorConf
{
    /* Whether execute IPM */
    int isIPM;  //0: false, 1: ture
    /* Parameters of configuration of camera */
    double m;           //number of rows in the full image.
    double n;           //number of columns in the full image.
    double h;           //the height above the ground (meter).
    double alphaTot;    //the total half viewing angle of the camera.
    double alpha_u;
    double alpha_v;
    double theta0;      //the camera tilted angle (pitch angle)
    double rHorizon;
    /* IPM parameters */
    double ipmX_min;    //meter in IPM x coordinate
    double ipmX_max;    //meter in IPM x coordinate
    double ipmY_max;    //meter in IPM y coordinate
    double ipmY_min;    //meter in IPM y coordinate
    double ipmStep;     //pixel per meter in y coordinate
    double mIPM;        //pixel in IPM y coordinate
    /* The method to use for IPM interpolation */
    int ipmInterpolation;

    /* kernel size to use for filtering */
    unsigned char kernelWidth;
    unsigned char kernelHeight;
    /* the type of grouping to use: 0 for HV lines and 1 for Hough Transform */
    unsigned char groupingType;
    /* the type of feature extracion: 0 for SOBEL_1, 1 for SOBEL_2, 2 for CANNY and 3 for LAPLACE */
    unsigned char filterType;

    /* rhoMin, rhoMax and rhoStep for Hough Transform (pixels) */
    double rhoMin, rhoMax, rhoStep;
    /* thetaMin, thetaMax, thetaStep for Hough Transform (radians) */
    double thetaMin, thetaMax, thetaStep;
    /* range of predicted ROI */
    int top_range;
    int bottom_range;

    /* conditions of checking tracking */
    double vpTop;
    double vpBottom;
    double distCornerMin;
    double distCornerMax;
} LaneDetectorConf;

typedef struct _Lane {
    ///start point
    cv::Point2d startPoint;
    ///end point
    cv::Point2d endPoint;
}Lane;

#endif // CONFIG_H
