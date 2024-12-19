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
#include <QTimer>
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
    QTimer *timer_spin;

    void init_QProcess();
    void init_GUI();

    void startBoatCamOrigin();
    void startBoatCamYolo();
    void startDockCam();
    void startDockCamOrigin();
    void startDockCamYolo();
    void startGoDock();
    
    void boatCamImageOriginCallback(const sensor_msgs::ImageConstPtr& msg);
    void boatCamImageYoloCallback(const sensor_msgs::ImageConstPtr& msg);
    void dockCamImageOriginCallback(const sensor_msgs::ImageConstPtr& msg);
    void dockCamImageYoloCallback(const sensor_msgs::ImageConstPtr& msg);

    void rosSpinOnce();

    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber boatCamOriginImageSubscriber;
    ros::Subscriber boatCamYoloImageSubscriber;
    ros::Subscriber dockCamOriginImageSubscriber;
    ros::Subscriber dockCamYoloImageSubscriber;
};

#endif // MAINWINDOW_H
