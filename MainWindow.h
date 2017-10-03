#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Qt
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
// Local
#include "CameraView.h"
#include "Buffer.h"
#include "SharedImageBuffer.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();

    private:
        Ui::MainWindow *ui;
        QPushButton *connectToCameraButton;
        QMap<int, int> deviceNumberMap;
        QMap<int, CameraView*> cameraViewMap;
        SharedImageBuffer *sharedImageBuffer;
        bool removeFromMapByTabIndex(QMap<int, int>& map, int tabIndex);
        void updateMapValues(QMap<int, int>& map, int tabIndex);
        QTimer *timer;
        std::string openVideo;

    public slots:
        void connectToCamera();
        void disconnectCamera(int index);
};

#endif // MAINWINDOW_H
