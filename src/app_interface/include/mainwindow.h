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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

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
    void init_UpdateSubscriber();

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
    void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void localVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void rosSpinOnce();

    void quaternion_to_euler(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);

    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber localPoseSubscriber;
    ros::Subscriber globalPositionSubscriber;
    ros::Subscriber localVelocitySubscriber;
    ros::Subscriber boatCamOriginImageSubscriber;
    ros::Subscriber boatCamYoloImageSubscriber;
    ros::Subscriber dockCamOriginImageSubscriber;
    ros::Subscriber dockCamYoloImageSubscriber;
};

#endif // MAINWINDOW_H
