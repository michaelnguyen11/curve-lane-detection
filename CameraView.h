#ifndef CAMERAVIEW_H
#define CAMERAVIEW_H

// Local
#include "CaptureThread.h"
#include "ProcessingThread.h"
#include "SharedImageBuffer.h"
#include <QWidget>

namespace Ui {
    class CameraView;
}

class CameraView : public QWidget
{
    Q_OBJECT

    public:
        explicit CameraView(QWidget *parent, int deviceNumber, SharedImageBuffer *sharedImageBuffer);
        ~CameraView();
        bool connectToCamera(bool dropFrame, int capThreadPrio, int procThreadPrio, bool createProcThread, int width, int height);

    private:
        Ui::CameraView *ui;
        ProcessingThread *processingThread;
        CaptureThread *captureThread;
        SharedImageBuffer *sharedImageBuffer;
        void stopCaptureThread();
        void stopProcessingThread();
        int deviceNumber;
        bool isCameraConnected;

    public slots:
        void clearImageBuffer();

    private slots:
        void updateFrame(const QImage &frame);
        void updateProcessingThreadStats(struct ThreadStatisticsData statData);
        void updateCaptureThreadStats(struct ThreadStatisticsData statData);

};

#endif // CAMERAVIEW_H
