#include "ProcessingThread.h"

ProcessingThread::ProcessingThread(SharedImageBuffer *sharedImageBuffer, int deviceNumber) : QThread(), sharedImageBuffer(sharedImageBuffer)
{
    // Save Device Number
    this->deviceNumber=deviceNumber;
    // Initialize members
    doStop=false;
    sampleNumber=0;
    fpsSum=0;
    fps.clear();
    statsData.averageFPS=0;
    statsData.nFramesProcessed=0;
}

void ProcessingThread::run()
{
    while(1)
    {
        ////////////////////////////////
        // Stop thread if doStop=TRUE //
        ////////////////////////////////
        doStopMutex.lock();
        if(doStop)
        {
            doStop=false;
            doStopMutex.unlock();
            break;
        }
        doStopMutex.unlock();
        /////////////////////////////////
        /////////////////////////////////

        // Save processing time
        processingTime=t.elapsed();
        // Start timer (used to calculate processing rate)
        t.start();

        processingMutex.lock();
        // Get frame from queue, store in currentFrame, set ROI
        currentFrame=Mat(sharedImageBuffer->getByDeviceNumber(deviceNumber)->get().clone());
//        Mat mat1, mat2, mat3, finalMat;
//        //Process frame here
//        Sobel(currentFrame, mat1, CV_8U, 1, 0, 3);
//        Sobel(mat1, mat2, CV_8U, 1, 0, 3);
//        medianBlur(mat2, mat3, 3);
//        threshold(mat3, finalMat, 150, 255, cv::THRESH_BINARY);
//        Canny(currentFrame, finalMat, 10, 100, 3);
        processingLane = new ProcessingLane(currentFrame);

        // Convert Mat to QImage
        Qframe=MatToQImage(currentFrame);
        processingMutex.unlock();

        // Inform GUI thread of new frame (QImage)
        emit newFrame(Qframe);

        // Update statistics
        updateFPS(processingTime);
        statsData.nFramesProcessed++;
        // Inform GUI of updated statistics
        emit updateStatisticsInGUI(statsData);
    }
    qDebug() << "Stopping processing thread...";
}

void ProcessingThread::updateFPS(int timeElapsed)
{
    // Add instantaneous FPS value to queue
    if(timeElapsed>0)
    {
        fps.enqueue((int)1000/timeElapsed);
        // Increment sample number
        sampleNumber++;
    }

    // Maximum size of queue is DEFAULT_PROCESSING_FPS_STAT_QUEUE_LENGTH
    if(fps.size()>PROCESSING_FPS_STAT_QUEUE_LENGTH)
        fps.dequeue();

    // Update FPS value every DEFAULT_PROCESSING_FPS_STAT_QUEUE_LENGTH samples
    if((fps.size()==PROCESSING_FPS_STAT_QUEUE_LENGTH)&&(sampleNumber==PROCESSING_FPS_STAT_QUEUE_LENGTH))
    {
        // Empty queue and store sum
        while(!fps.empty())
            fpsSum+=fps.dequeue();
        // Calculate average FPS
        statsData.averageFPS=fpsSum/PROCESSING_FPS_STAT_QUEUE_LENGTH;
        // Reset sum
        fpsSum=0;
        // Reset sample number
        sampleNumber=0;
    }
}

void ProcessingThread::stop()
{
    QMutexLocker locker(&doStopMutex);
    doStop=true;
}
