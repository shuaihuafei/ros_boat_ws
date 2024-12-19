#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"
#include <QProcess>
#include <QDebug>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QRadialGradient>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr,int argc = 0,char** argv=nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QProcess *roscoreProcess;
    QProcess *startDockProcess;
    QProcess *startDockCamProcess;
    QProcess *startDockCamYoloProcess;

    void init_GUI();
    void init_QProcess();

    void startRosCore();
    void startDockCam();
    void startDockCamOrigin();
    void startDockCamYolo();
    void startDockCamYoloImageSubscriber();
    void startDockCamOriginImageSubscriber();
    void dockCamImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void startGoDock();

    int argc_;
    char** argv_;
};

#endif // MAINWINDOW_H
