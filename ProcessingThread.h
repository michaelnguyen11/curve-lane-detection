#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

// Qt
#include <QtCore/QThread>
#include <QtCore/QTime>
#include <QtCore/QQueue>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// Local
#include "Config.h"
#include "Buffer.h"
#include "MatToQImage.h"
#include "SharedImageBuffer.h"
#include "ProcessingLane.h"

using namespace cv;

class ProcessingThread : public QThread
{
    Q_OBJECT

    public:
        ProcessingThread(SharedImageBuffer *sharedImageBuffer, int deviceNumber);
        void stop();

    private:
        void updateFPS(int);
        ProcessingLane *processingLane;
        SharedImageBuffer *sharedImageBuffer;
        Mat currentFrame;
        Mat currentFrameGrayscale;
        Rect currentROI;
        QImage Qframe;
        QTime t;
        QQueue<int> fps;
        QMutex doStopMutex;
        QMutex processingMutex;
        Size frameSize;
        Point framePoint;
        struct ThreadStatisticsData statsData;
        volatile bool doStop;
        int processingTime;
        int fpsSum;
        int sampleNumber;
        int deviceNumber;
        bool enableFrameProcessing;

    protected:
        void run();

    signals:
        void newFrame(const QImage &frame);
        void updateStatisticsInGUI(struct ThreadStatisticsData);
};

#endif // PROCESSINGTHREAD_H
