#include "MainWindow.h"
#include "ui_MainWindow.h"
// Qt
#include <QLabel>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // Setup UI
    ui->setupUi(this);
    // Set start tab as blank
    QLabel *newTab = new QLabel(ui->tabWidget);
    newTab->setText("No camera connected.");
    newTab->setAlignment(Qt::AlignCenter);
    ui->tabWidget->addTab(newTab, "");
    ui->tabWidget->setTabsClosable(false);
    // Add "Connect to Camera" button to tab
//    connectToCameraButton = new QPushButton();
//    connectToCameraButton->setText("Connect to Camera...");
    //ui->tabWidget->setCornerWidget(connectToCameraButton, Qt::TopLeftCorner);
    //connect(connectToCameraButton,SIGNAL(released()),this, SLOT(connectToCamera()));

    timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this, SLOT(connectToCamera()));
    timer->start(40);

    connect(ui->tabWidget,SIGNAL(tabCloseRequested(int)),this, SLOT(disconnectCamera(int)));
    // Set focus on button
    //connectToCameraButton->setFocus();

    // Connect other signals/slots
//    connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(showAboutDialog()));
//    connect(ui->actionQuit, SIGNAL(triggered()), this, SLOT(close()));
//    connect(ui->actionFullScreen, SIGNAL(toggled(bool)), this, SLOT(setFullScreen(bool)));
    // Create SharedImageBuffer object
    sharedImageBuffer = new SharedImageBuffer();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::connectToCamera()
{
    int nextTabIndex = (deviceNumberMap.size()==0) ? 0 : ui->tabWidget->count();
    openVideo = "/home/hiep/video.mp4";
    int deviceNumber = std::atoi(openVideo.c_str());
    if(!deviceNumberMap.contains(deviceNumber))
    {
        // Create ImageBuffer with user-defined size
        //Buffer<Mat> *imageBuffer = new Buffer<Mat>(cameraConnectDialog->getImageBufferSize());
        // Create ImageBuffer size(byte image buffer)
        Buffer<Mat> *imageBuffer = new Buffer<Mat>(100);
        // Add created ImageBuffer to SharedImageBuffer object
        sharedImageBuffer->add(deviceNumber, imageBuffer, ui->actionSynchronizeStreams->isChecked());
        // Create CameraView
        cameraViewMap[deviceNumber] = new CameraView(ui->tabWidget, deviceNumber, sharedImageBuffer);

        if(cameraViewMap[deviceNumber]->connectToCamera(false, QThread::HighPriority, QThread::HighPriority, true, 1280, 1080))
        {
            // Add to map
            deviceNumberMap[deviceNumber] = nextTabIndex;
            // Allow tabs to be closed
            ui->tabWidget->setTabsClosable(true);
            // If start tab, remove
            if(nextTabIndex==0)
                ui->tabWidget->removeTab(0);

            ui->tabWidget->addTab(cameraViewMap[deviceNumber], "Camera");
            ui->tabWidget->setCurrentWidget(cameraViewMap[deviceNumber]);
            // Prevent user from enabling/disabling stream synchronization after a camera has been connected
            ui->actionSynchronizeStreams->setEnabled(false);
        }
        // Could not connect to camera
        else
        {
            // Explicitly delete widget
            delete cameraViewMap[deviceNumber];
            cameraViewMap.remove(deviceNumber);
            // Remove from shared buffer
            sharedImageBuffer->removeByDeviceNumber(deviceNumber);
            // Explicitly delete ImageBuffer object
            delete imageBuffer;
        }
    }
}

void MainWindow::disconnectCamera(int index)
{
    // Local variable(s)
    bool doDisconnect=true;

    // Disconnect camera
    if(doDisconnect)
    {
        // Save number of tabs
        int nTabs = ui->tabWidget->count();
        // Close tab
        ui->tabWidget->removeTab(index);

        // Delete widget (CameraView) contained in tab
        delete cameraViewMap[deviceNumberMap.key(index)];
        cameraViewMap.remove(deviceNumberMap.key(index));

        // Remove from map
        removeFromMapByTabIndex(deviceNumberMap, index);
        // Update map (if tab closed is not last)
        if(index!=(nTabs-1))
            updateMapValues(deviceNumberMap, index);

        // If start tab, set tab as blank
        if(nTabs==1)
        {
            QLabel *newTab = new QLabel(ui->tabWidget);
            newTab->setText("No camera connected.");
            newTab->setAlignment(Qt::AlignCenter);
            ui->tabWidget->addTab(newTab, "");
            ui->tabWidget->setTabsClosable(false);
            ui->actionSynchronizeStreams->setEnabled(true);
        }
    }
}

bool MainWindow::removeFromMapByTabIndex(QMap<int, int> &map, int tabIndex)
{
    QMutableMapIterator<int, int> i(map);
    while (i.hasNext())
    {
         i.next();
         if(i.value()==tabIndex)
         {
             i.remove();
             return true;
         }
    }
    return false;
}

void MainWindow::updateMapValues(QMap<int, int> &map, int tabIndex)
{
    QMutableMapIterator<int, int> i(map);
    while (i.hasNext())
    {
        i.next();
        if(i.value()>tabIndex)
            i.setValue(i.value()-1);
    }
}
