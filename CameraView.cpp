#include "CameraView.h"
#include "ui_CameraView.h"
// Qt
#include <QMessageBox>

CameraView::CameraView(QWidget *parent, int deviceNumber, SharedImageBuffer *sharedImageBuffer) :
    QWidget(parent),
    ui(new Ui::CameraView),
    sharedImageBuffer(sharedImageBuffer)
{
    // Setup UI
    ui->setupUi(this);
    // Save Device Number
    this->deviceNumber=deviceNumber;
    // Initialize internal flag
    isCameraConnected=false;
    // Set initial GUI state
    ui->frameLabel->setText("No camera connected.");
    ui->imageBufferBar->setValue(0);
    ui->imageBufferLabel->setText("[000/000]");
    ui->captureRateLabel->setText("");
    ui->processingRateLabel->setText("");
    ui->deviceNumberLabel->setText("");
    ui->cameraResolutionLabel->setText("");
    ui->clearImageBufferButton->setDisabled(true);
    // Connect signals/slots
    connect(ui->clearImageBufferButton, SIGNAL(released()), this, SLOT(clearImageBuffer()));

    // Register type
    qRegisterMetaType<struct ThreadStatisticsData>("ThreadStatisticsData");
}

CameraView::~CameraView()
{
    if(isCameraConnected)
    {
        // Stop processing thread
        if(processingThread->isRunning())
            stopProcessingThread();
        // Stop capture thread
        if(captureThread->isRunning())
            stopCaptureThread();

        // Automatically start frame processing (for other streams)
        if(sharedImageBuffer->isSyncEnabledForDeviceNumber(deviceNumber))
            sharedImageBuffer->setSyncEnabled(true);

        // Remove from shared buffer
        sharedImageBuffer->removeByDeviceNumber(deviceNumber);
        // Disconnect camera
        if(captureThread->disconnectCamera())
            qDebug() << "[" << deviceNumber << "] Camera successfully disconnected.";
        else
            qDebug() << "[" << deviceNumber << "] WARNING: Camera already disconnected.";
    }
    // Delete UI
    delete ui;
}

bool CameraView::connectToCamera(bool dropFrameIfBufferFull, int capThreadPrio, int procThreadPrio, bool enableFrameProcessing, int width, int height)
{
    // Set frame label text
    if(sharedImageBuffer->isSyncEnabledForDeviceNumber(deviceNumber))
        ui->frameLabel->setText("Camera connected. Waiting...");
    else
        ui->frameLabel->setText("Connecting to camera...");

    // Create capture thread
    captureThread = new CaptureThread(sharedImageBuffer, deviceNumber, dropFrameIfBufferFull, width, height);
    // Attempt to connect to camera
    if(captureThread->connectToCamera())
    {
        // Create processing thread
        processingThread = new ProcessingThread(sharedImageBuffer, deviceNumber);
        // Setup signal/slot connections
        connect(processingThread, SIGNAL(newFrame(QImage)), this, SLOT(updateFrame(QImage)));
        connect(processingThread, SIGNAL(updateStatisticsInGUI(struct ThreadStatisticsData)), this, SLOT(updateProcessingThreadStats(struct ThreadStatisticsData)));
        connect(captureThread, SIGNAL(updateStatisticsInGUI(struct ThreadStatisticsData)), this, SLOT(updateCaptureThreadStats(struct ThreadStatisticsData)));
        // Start capturing frames from camera
        captureThread->start((QThread::Priority)capThreadPrio);
        // Start processing captured frames (if enabled)
        if(enableFrameProcessing)
            processingThread->start((QThread::Priority)procThreadPrio);

        // Setup imageBufferBar with minimum and maximum values
        ui->imageBufferBar->setMinimum(0);
        ui->imageBufferBar->setMaximum(sharedImageBuffer->getByDeviceNumber(deviceNumber)->maxSize());
        // Enable "Clear Image Buffer" push button
        ui->clearImageBufferButton->setEnabled(true);
        // Set text in labels
        ui->deviceNumberLabel->setNum(deviceNumber);
        ui->cameraResolutionLabel->setText(QString::number(captureThread->getInputSourceWidth())+QString("x")+QString::number(captureThread->getInputSourceHeight()));
        // Set internal flag and return
        isCameraConnected=true;
        // Set frame label text
        if(!enableFrameProcessing)
            ui->frameLabel->setText("Frame processing disabled.");
        return true;
    }
    // Failed to connect to camera
    else
        return false;
}

void CameraView::stopCaptureThread()
{
    qDebug() << "[" << deviceNumber << "] About to stop capture thread...";
    captureThread->stop();
    sharedImageBuffer->wakeAll(); // This allows the thread to be stopped if it is in a wait-state
    // Take one frame off a FULL queue to allow the capture thread to finish
    if(sharedImageBuffer->getByDeviceNumber(deviceNumber)->isFull())
        sharedImageBuffer->getByDeviceNumber(deviceNumber)->get();
    captureThread->wait();
    qDebug() << "[" << deviceNumber << "] Capture thread successfully stopped.";
}

void CameraView::stopProcessingThread()
{
    qDebug() << "[" << deviceNumber << "] About to stop processing thread...";
    processingThread->stop();
    sharedImageBuffer->wakeAll(); // This allows the thread to be stopped if it is in a wait-state
    processingThread->wait();
    qDebug() << "[" << deviceNumber << "] Processing thread successfully stopped.";
}

void CameraView::updateCaptureThreadStats(struct ThreadStatisticsData statData)
{
    // Show [number of images in buffer / image buffer size] in imageBufferLabel
    ui->imageBufferLabel->setText(QString("[")+QString::number(sharedImageBuffer->getByDeviceNumber(deviceNumber)->size())+
                                  QString("/")+QString::number(sharedImageBuffer->getByDeviceNumber(deviceNumber)->maxSize())+QString("]"));
    // Show percentage of image bufffer full in imageBufferBar
    ui->imageBufferBar->setValue(sharedImageBuffer->getByDeviceNumber(deviceNumber)->size());

    // Show processing rate in captureRateLabel
    ui->captureRateLabel->setText(QString::number(statData.averageFPS)+" fps");
    // Show number of frames captured in nFramesCapturedLabel
    ui->nFramesCapturedLabel->setText(QString("[") + QString::number(statData.nFramesProcessed) + QString("]"));
}

void CameraView::updateProcessingThreadStats(struct ThreadStatisticsData statData)
{
    // Show processing rate in processingRateLabel
    ui->processingRateLabel->setText(QString::number(statData.averageFPS)+" fps");
    ui->nFramesProcessedLabel->setText(QString("[") + QString::number(statData.nFramesProcessed) + QString("]"));
}

void CameraView::updateFrame(const QImage &frame)
{
    // Display frame
    ui->frameLabel->setPixmap(QPixmap::fromImage(frame).scaled(ui->frameLabel->width(), ui->frameLabel->height(),Qt::KeepAspectRatio));
}

void CameraView::clearImageBuffer()
{
    if(sharedImageBuffer->getByDeviceNumber(deviceNumber)->clear())
        qDebug() << "[" << deviceNumber << "] Image buffer successfully cleared.";
    else
        qDebug() << "[" << deviceNumber << "] WARNING: Could not clear image buffer.";
}
