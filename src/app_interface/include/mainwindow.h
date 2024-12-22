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
#include <QWebEngineView>
#include <QWebChannel>
#include <QDir>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "mapchannel.h"
#include "gaugecolor.h"
#include "gaugecompass.h"

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
    // Qt相关变量
    Ui::MainWindow *ui;
    QProcess *roscoreProcess;
    QProcess *startDockProcess;
    QProcess *startDockCamProcess;
    QProcess *startDockCamYoloProcess;
    QTimer *timer_spin;
    QWebChannel *webChannel;
    MapChannel *mapChannel;
    // SpeedWidget *speedWidget;
    // CompassWidget *compassWidget;

    // 初始化相关
    void init_QProcess();
    void init_Variable();
    void init_GUI();
    void init_map();
    void init_UpdateSubscriber();

    // 按钮启动相关
    void startBoatCamOrigin();
    void startBoatCamYolo();
    void startDockCam();
    void startDockCamOrigin();
    void startDockCamYolo();
    void startGoDock();
    
    // 回调函数相关
    void boatCamImageOriginCallback(const sensor_msgs::ImageConstPtr& msg);
    void boatCamImageYoloCallback(const sensor_msgs::ImageConstPtr& msg);
    void dockCamImageOriginCallback(const sensor_msgs::ImageConstPtr& msg);
    void dockCamImageYoloCallback(const sensor_msgs::ImageConstPtr& msg);
    void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void localVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    // 通过定时器来调用回旋函数
    void rosSpinOnce();

    // 功能函数相关
    void quaternion_to_euler(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);

    // 槽函数相关
    void reloadMap();

    // ROS相关变量
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber localPoseSubscriber;
    ros::Subscriber globalPositionSubscriber;
    ros::Subscriber localVelocitySubscriber;
    ros::Subscriber boatCamOriginImageSubscriber;
    ros::Subscriber boatCamYoloImageSubscriber;
    ros::Subscriber dockCamOriginImageSubscriber;
    ros::Subscriber dockCamYoloImageSubscriber;

    // 作为类中的变量传递使用
    double course;

signals:
    void updateBoatPosition(double lng, double lat, double course);
};

#endif // MAINWINDOW_H
