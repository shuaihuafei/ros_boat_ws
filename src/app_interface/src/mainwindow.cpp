#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>

MainWindow::MainWindow(QWidget *parent,int argc,char** argv)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ros::init(argc, argv, "ControlInterfaceNode");
    if (!nh_) {
        nh_ = std::make_shared<ros::NodeHandle>();  // 延迟初始化
    }
    // 初始化ROS发布者
    target_pos_pub_ = nh_->advertise<sensor_msgs::NavSatFix>("/target_global_position", 10);

    init_QProcess();
    init_Variable();
    init_GUI();
    init_map();

    startBoatCamOrigin();
    startBoatCamYolo();
    startDockCam();
    startDockCamOrigin();
    startDockCamYolo();
    startGoDock();
    startPositionCtrl();

}

void MainWindow::init_QProcess()
{
    roscoreProcess = nullptr;
    startDockProcess = nullptr;
    startDockCamProcess = nullptr;
    startDockCamYoloProcess = nullptr;
    positionControlProcess = nullptr;
}

void MainWindow::init_Variable()
{
    course = 0.0;
}

// 主窗口初始化
void MainWindow::init_GUI()
{
    QPixmap pixmap_logo(":/images/logo.png");
    ui->label_log->setPixmap(pixmap_logo);
    ui->label_log->setScaledContents(true);

    // 设置textEdit的最大显示行
    ui->textEdit_qdebug->document()->setMaximumBlockCount(100);

    // 定时器设置，用来定期调用 ros::spinOnce()
    timer_spin = new QTimer(this);
    connect(timer_spin, &QTimer::timeout, this, &MainWindow::rosSpinOnce);
    timer_spin->start(50); // 每50毫秒调用一次 rosSpinOnce

    // 初始化状态显示等订阅者
    init_UpdateSubscriber();

    init_map();
}

void MainWindow::init_map()
{
    webChannel = new QWebChannel(this);
    mapChannel = new MapChannel(this);
    webChannel->registerObject("passId", mapChannel);
    ui->widget_map->page()->setWebChannel(webChannel);
    
    // 获取ROS功能包路径
    std::string package_path = ros::package::getPath("app_interface");
    QString mapPath = QString::fromStdString(package_path) + "/map/OfflineMap/map.html";
    qDebug() << "Map path:" << mapPath;
    ui->widget_map->load(QUrl::fromLocalFile(mapPath));
    // 将实际的经纬度信息发给地图显示
    connect(this, &MainWindow::updateBoatPosition, mapChannel, &MapChannel::updateBoatPos);
    // 将地图上的航点通过ROS话题发布出去
    connect(mapChannel, &MapChannel::pointsCome, 
            [this](int index, double lng, double lat) {
        qDebug() << "Waypoint:" << index 
                 << "Longitude:" << QString::number(lng,'f',6) 
                 << "Latitude:" << QString::number(lat,'f',6);
        
        // 创建并发布NavSatFix消息
        sensor_msgs::NavSatFix target_msg;
        target_msg.header.stamp = ros::Time::now();
        target_msg.header.frame_id = "map";
        
        target_msg.latitude = lat;
        target_msg.longitude = lng;
        target_msg.altitude = 0.0;
        
        // 设置协方差
        target_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        
        // 发布消息
        if (nh_ && target_pos_pub_) {
            target_pos_pub_.publish(target_msg);
            ROS_INFO("Published target position: lat=%.6f, lon=%.6f", lat, lng);
        } else {
            ROS_WARN("ROS publisher not initialized");
        }
    });

    // 添加ROS消息处理的定时器
    QTimer *ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, []() {
        if (ros::ok()) {
            ros::spinOnce();
        }
    });
    ros_timer->start(100);  // 每100ms处理一次ROS消息
}

void MainWindow::init_UpdateSubscriber()
{
    // 更新经纬度等信息显示
    localPoseSubscriber = nh_->subscribe("/mavros/local_position/pose", 10, &MainWindow::localPoseCallback, this);
    globalPositionSubscriber = nh_->subscribe("/mavros/global_position/global", 10, &MainWindow::globalPositionCallback, this);
    localVelocitySubscriber = nh_->subscribe("/mavros/local_position/velocity_body", 10, &MainWindow::localVelocityCallback, this);
}

void MainWindow::rosSpinOnce()
{
    ros::spinOnce();  // 非阻塞调用 ROS 的回调处理
}

void MainWindow::startBoatCamOrigin()
{
    connect(ui->pushButton_cam_boat_origin, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_cam_boat_origin->setText("关闭相机原图像");
            // 启动 ROS 订阅
            if (!boatCamOriginImageSubscriber) {
                // 订阅 ROS 图像话题
                boatCamOriginImageSubscriber = nh_->subscribe("/camera/color/image_raw", 1, &MainWindow::boatCamImageOriginCallback, this);
                ui->textEdit_qdebug->append("<font color='green'>已开始订阅相机原图像话题</font>");
            }
        } else {
            ui->pushButton_cam_boat_origin->setText("开启相机原图像");
            // 取消订阅 ROS 图像话题
            if (boatCamOriginImageSubscriber) {
                boatCamOriginImageSubscriber.shutdown();
                ui->textEdit_qdebug->append("<font color='red'>已停止订阅相机原图像话题</font>");
            }
        }
    });
}

void MainWindow::startPositionCtrl()
{
    // 确保 positionControlProcess 已经初始化
    positionControlProcess = new QProcess(this);
    connect(ui->pushButton_position_ctrl, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_position_ctrl->setText("关闭位置控制");
            // 启动ROS节点
            positionControlProcess->start("rosrun", QStringList() << "usv_run" << "position_ctrl");
            if (positionControlProcess->waitForStarted()) {
                ui->textEdit_qdebug->append("<font color='green'>位置控制节点已启动</font>");
            }
        } else {
            ui->pushButton_position_ctrl->setText("开启位置控制");
            // 关闭ROS节点
            if (positionControlProcess->state() == QProcess::Running) {
                positionControlProcess->terminate();
                positionControlProcess->waitForFinished();
                ui->textEdit_qdebug->append("<font color='red'>位置控制节点已关闭</font>");
            }
        }
    });
}

void MainWindow::startBoatCamYolo()
{
    connect(ui->pushButton_cam_boat_yolo, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_cam_boat_yolo->setText("关闭yolo识别图像");
            // 启动 ROS 订阅
            if (!boatCamYoloImageSubscriber) {
                // 订阅 ROS 图像话题
                boatCamYoloImageSubscriber = nh_->subscribe("/yolo/predict_dock", 1, &MainWindow::boatCamImageYoloCallback, this);
                ui->textEdit_qdebug->append("<font color='green'>已开始订阅相机原图像话题</font>");
            }
        } else {
            ui->pushButton_cam_boat_yolo->setText("开启yolo识别图像");
            // 取消订阅 ROS 图像话题
            if (boatCamYoloImageSubscriber) {
                boatCamYoloImageSubscriber.shutdown();
                ui->textEdit_qdebug->append("<font color='red'>已停止订阅相机原图像话题</font>");
            }
        }
    });
}

// 回调函数：显示 ROS 图像
void MainWindow::boatCamImageOriginCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        // 使用 cv_bridge 将 ROS 图像转换为 OpenCV 图像
        cv_bridge::CvImagePtr cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // 转换为 QImage
        QImage img = QImage(cvImagePtr->image.data, cvImagePtr->image.cols, cvImagePtr->image.rows,
                            cvImagePtr->image.step, QImage::Format_RGB888);

        // 将 QImage 转换为 QPixmap，并更新 QLabel 显示
        QPixmap pixmap = QPixmap::fromImage(img);
        ui->label_cam_boat_origin->setPixmap(pixmap.scaled(ui->label_cam_boat_origin->size(), Qt::KeepAspectRatio));
        ui->label_cam_boat_origin->setAlignment(Qt::AlignCenter);
    }
    catch (cv_bridge::Exception& e) {
        qDebug() << "cv_bridge exception: " << e.what();
    }
}

// 回调函数：显示 ROS 图像
void MainWindow::boatCamImageYoloCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        // 使用 cv_bridge 将 ROS 图像转换为 OpenCV 图像
        cv_bridge::CvImagePtr cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // 转换为 QImage
        QImage img = QImage(cvImagePtr->image.data, cvImagePtr->image.cols, cvImagePtr->image.rows,
                            cvImagePtr->image.step, QImage::Format_RGB888);

        // 将 QImage 转换为 QPixmap，并更新 QLabel 显示
        QPixmap pixmap = QPixmap::fromImage(img);
        ui->label_cam_boat_yolo->setPixmap(pixmap.scaled(ui->label_cam_boat_yolo->size(), Qt::KeepAspectRatio));
        ui->label_cam_boat_yolo->setAlignment(Qt::AlignCenter);
    }
    catch (cv_bridge::Exception& e) {
        qDebug() << "cv_bridge exception: " << e.what();
    }
}

void MainWindow::startDockCam()
{
    connect(ui->pushButton_start_cam_dock, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_start_cam_dock->setText("关闭坞舱摄像头");
            // 启动 ROS 节点 (roslaunch)
            startDockCamProcess = new QProcess(this);  // 创建 QProcess 对象
            // 使用 bash 启动进程
            QString command = "bash";
            QStringList arguments;
            // 激活 conda 环境并启动 roslaunch
            QString activateEnv = "source ~/miniconda3/bin/activate yolov11 && roslaunch cv usb_cam.launch";
            arguments << "-c" << activateEnv; // 使用 -c 来传递命令
            startDockCamProcess->start(command, arguments);  // 启动 roslaunch

            // 捕获标准输出并显示到 textEdit_qdebug
            connect(startDockCamProcess, &QProcess::readyReadStandardOutput, [=]() {
                QString output = startDockCamProcess->readAllStandardOutput();
                // 去除终端控制字符（颜色编码等）
                output.remove(QRegExp("\\x1B\\[[0-9;]*[mK]"));
                QString coloredOutput = "<font color='green'>" + output + "</font>";
                ui->textEdit_qdebug->append(coloredOutput);  // 将输出追加到 textEdit_qdebug
            });

            // 捕获标准错误并显示到 textEdit_qdebug
            connect(startDockCamProcess, &QProcess::readyReadStandardError, [=]() {
                QString errorOutput = startDockCamProcess->readAllStandardError();
                // 去除终端控制字符（颜色编码等）
                errorOutput.remove(QRegExp("\\x1B\\[[0-9;]*[mK]"));
                QString coloredErrorOutput = "<font color='red'>" + errorOutput + "</font>";
                ui->textEdit_qdebug->append(coloredErrorOutput);  // 将错误输出追加到 textEdit_qdebug
            });

            // 检查 roslaunch 是否启动成功
            if (!startDockCamProcess->waitForStarted()) {
                ui->textEdit_qdebug->append("<font color='red'>启动 入坞摄像头程序 失败！</font>");
            } else {
                ui->textEdit_qdebug->append("<font color='green'>入坞摄像头程序 启动成功！</font>");
            }
        } else {
            ui->pushButton_start_cam_dock->setText("开启坞舱摄像头");
            if (startDockCamProcess && startDockCamProcess->state() == QProcess::Running) {
                startDockCamProcess->terminate();  // 优雅地停止进程
                if (!startDockCamProcess->waitForFinished(3000)) {  // 等待最多3秒
                    startDockCamProcess->kill();  // 如果进程没有正常退出，强制结束进程
                    ui->textEdit_qdebug->append("<font color='red'>入坞摄像头程序 被强制终止</font>");
                } else {
                    ui->textEdit_qdebug->append("<font color='red'>入坞摄像头程序 已停止</font>");
                }
            }
        }
    });
}

void MainWindow::startDockCamOrigin()
{
    connect(ui->pushButton_cam_dock_origin, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_cam_dock_origin->setText("关闭相机原图像");
            // 启动 ROS 订阅
            if (!dockCamOriginImageSubscriber) {
                // 订阅 ROS 图像话题
                dockCamOriginImageSubscriber = nh_->subscribe("/camera_usb/color/image_raw", 1, &MainWindow::dockCamImageOriginCallback, this);
                ui->textEdit_qdebug->append("<font color='green'>已开始订阅相机原图像话题</font>");
            }
        } else {
            ui->pushButton_cam_dock_origin->setText("开启相机原图像");
            // 取消订阅 ROS 图像话题
            if (dockCamOriginImageSubscriber) {
                dockCamOriginImageSubscriber.shutdown();
                ui->textEdit_qdebug->append("<font color='red'>已停止订阅相机原图像话题</font>");
            }
        }
    });
}

void MainWindow::startDockCamYolo()
{
    connect(ui->pushButton_cam_dock_yolo, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_cam_dock_yolo->setText("关闭yolo识别图像");
            // 启动 ROS 节点 (roslaunch)
            startDockCamYoloProcess = new QProcess(this);  // 创建 QProcess 对象
            // 使用 bash 启动进程
            QString command = "bash";
            QStringList arguments;
            // 激活 conda 环境并启动 roslaunch
            QString activateEnv = "source ~/miniconda3/bin/activate yolov11 && roslaunch cv 02boat_angle.launch";
            arguments << "-c" << activateEnv; // 使用 -c 来传递命令
            startDockCamYoloProcess->start(command, arguments);

            // 捕获标准输出并显示到 textEdit_qdebug
            connect(startDockCamYoloProcess, &QProcess::readyReadStandardOutput, [=]() {
                QString output = startDockCamYoloProcess->readAllStandardOutput();
                // 去除终端控制字符（颜色编码等）
                output.remove(QRegExp("\\x1B\\[[0-9;]*[mK]"));
                QString coloredOutput = "<font color='black'>" + output + "</font>";
                ui->textEdit_qdebug->append(coloredOutput);  // 将输出追加到 textEdit_qdebug
            });

            // 捕获标准错误并显示到 textEdit_qdebug
            connect(startDockCamYoloProcess, &QProcess::readyReadStandardError, [=]() {
                QString errorOutput = startDockCamYoloProcess->readAllStandardError();
                // 去除终端控制字符（颜色编码等）
                errorOutput.remove(QRegExp("\\x1B\\[[0-9;]*[mK]"));
                QString coloredErrorOutput = "<font color='red'>" + errorOutput + "</font>";
                ui->textEdit_qdebug->append(coloredErrorOutput);  // 将错误输出追加到 textEdit_qdebug
            });

            // 检查 roslaunch 是否启动成功
            if (!startDockCamYoloProcess->waitForStarted()) {
                ui->textEdit_qdebug->append("<font color='red'>启动 yolo识别图像 失败！</font>");
            } else {
                ui->textEdit_qdebug->append("<font color='green'>yolo识别图像 启动成功！</font>");
                // 启动 ROS 订阅
                if (!dockCamYoloImageSubscriber) {
                    // 订阅 ROS 图像话题
                    dockCamYoloImageSubscriber = nh_->subscribe("/yolo/predict_boat", 1, &MainWindow::dockCamImageYoloCallback, this);
                    ui->textEdit_qdebug->append("<font color='green'>已开始订阅相机原图像话题</font>");
                }
            }
        } else {
            ui->pushButton_cam_dock_yolo->setText("开启yolo识别图像");
            // 取消订阅 ROS 图像话题
            if (dockCamYoloImageSubscriber) {
                dockCamYoloImageSubscriber.shutdown();
                ui->textEdit_qdebug->append("<font color='red'>已停止订阅相机原图像话题</font>");
            }
            if (startDockCamYoloProcess && startDockCamYoloProcess->state() == QProcess::Running) {
                startDockCamYoloProcess->terminate();  // 优雅地停止进程
                if (!startDockCamYoloProcess->waitForFinished(3000)) {  // 等待最多3秒
                    startDockCamYoloProcess->kill();  // 如果进程没有正常退出，强制结束进程
                    ui->textEdit_qdebug->append("<font color='red'>yolo识别图像 被强制终止</font>");
                } else {
                    ui->textEdit_qdebug->append("<font color='red'>yolo识别图像 已停止</font>");
                }
            }
        }
    });
}

// 回调函数：显示 ROS 图像
void MainWindow::dockCamImageOriginCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        // 使用 cv_bridge 将 ROS 图像转换为 OpenCV 图像
        cv_bridge::CvImagePtr cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // 转换为 QImage
        QImage img = QImage(cvImagePtr->image.data, cvImagePtr->image.cols, cvImagePtr->image.rows,
                            cvImagePtr->image.step, QImage::Format_RGB888);

        // 将 QImage 转换为 QPixmap，并更新 QLabel 显示
        QPixmap pixmap = QPixmap::fromImage(img);
        ui->label_cam_dock_origin->setPixmap(pixmap.scaled(ui->label_cam_dock_origin->size(), Qt::KeepAspectRatio));
        ui->label_cam_dock_origin->setAlignment(Qt::AlignCenter);
    }
    catch (cv_bridge::Exception& e) {
        qDebug() << "cv_bridge exception: " << e.what();
    }
}

// 回调函数：显示 ROS 图像
void MainWindow::dockCamImageYoloCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        // 使用 cv_bridge 将 ROS 图像转换为 OpenCV 图像
        cv_bridge::CvImagePtr cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

        // 转换为 QImage
        QImage img = QImage(cvImagePtr->image.data, cvImagePtr->image.cols, cvImagePtr->image.rows,
                            cvImagePtr->image.step, QImage::Format_RGB888);

        // 将 QImage 转换为 QPixmap，并更新 QLabel 显示
        QPixmap pixmap = QPixmap::fromImage(img);
        ui->label_cam_dock_yolo->setPixmap(pixmap.scaled(ui->label_cam_dock_yolo->size(), Qt::KeepAspectRatio));
        ui->label_cam_dock_yolo->setAlignment(Qt::AlignCenter);
    }
    catch (cv_bridge::Exception& e) {
        qDebug() << "cv_bridge exception: " << e.what();
    }
}

// 用于四元数转换的辅助函数，将四元数转换为欧拉角（roll, pitch, yaw）
void MainWindow::quaternion_to_euler(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
    // 计算欧拉角
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // 夹角范围 [-pi/2, pi/2]
    else
        pitch = asin(sinp);

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void MainWindow::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 获取位置
    const geometry_msgs::Point& position = msg->pose.position;

    // 获取四元数方向
    const geometry_msgs::Quaternion& orientation = msg->pose.orientation;

    // 转换四元数为欧拉角
    double roll, pitch, yaw;
    quaternion_to_euler(orientation, roll, pitch, yaw);

    course = 90 - yaw * 180.0 / M_PI;
    if (course < 0)
        course = 360 + course;

    ui->label_ENU_X_value->setText(QString::number(position.x, 'f', 2) + "m");
    ui->label_ENU_Y_value->setText(QString::number(position.y, 'f', 2) + "m");
    ui->label_yaw_value->setText(QString::number(course, 'f', 2) + "°");

    ui->widget_attitude->setProperty("value", course);
}

void MainWindow::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    double latitude = msg->latitude;    // 获取纬度
    double longitude = msg->longitude;  // 获取经度

    ui->label_latitude_value->setText(QString::number(latitude, 'f', 6) + "°");
    ui->label_longitude_value->setText(QString::number(longitude, 'f', 6) + "°");

    emit updateBoatPosition(longitude, latitude, course);
}

void MainWindow::localVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    double v_body_x = msg->twist.linear.x;

    ui->label_throttle_value->setText(QString::number(v_body_x, 'f', 2) + "m/s");
    ui->widget_speed->setProperty("value", v_body_x);
}

// 启动入坞相关程序
void MainWindow::startGoDock()
{
    connect(ui->pushButton_start_dock, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_start_dock->setText("停止入坞程序");
            // 启动 ROS 节点 (roslaunch)
            startDockProcess = new QProcess(this);  // 创建 QProcess 对象
            QString command = "roslaunch";
            QStringList arguments;
            arguments << "usv_run" << "007actuator_ctrl_dock_boat.launch";  // roslaunch 命令和参数
            startDockProcess->start(command, arguments);  // 启动 roslaunch

            // 捕获标准输出并显示到 textEdit_qdebug
            connect(startDockProcess, &QProcess::readyReadStandardOutput, [=]() {
                QString output = startDockProcess->readAllStandardOutput();
                // 去除终端控制字符（颜色编码等）
                output.remove(QRegExp("\\x1B\\[[0-9;]*[mK]"));
                QString coloredOutput = "<font color='green'>" + output + "</font>";
                ui->textEdit_qdebug->append(coloredOutput);  // 将输出追加到 textEdit_qdebug
            });

            // 捕获标准错误并显示到 textEdit_qdebug
            connect(startDockProcess, &QProcess::readyReadStandardError, [=]() {
                QString errorOutput = startDockProcess->readAllStandardError();
                // 去除终端控制字符（颜色编码等）
                errorOutput.remove(QRegExp("\\x1B\\[[0-9;]*[mK]"));
                QString coloredErrorOutput = "<font color='red'>" + errorOutput + "</font>";
                ui->textEdit_qdebug->append(coloredErrorOutput);  // 将错误输出追加到 textEdit_qdebug
            });

            // 检查 roslaunch 是否启动成功
            if (!startDockProcess->waitForStarted()) {
                ui->textEdit_qdebug->append("<font color='red'>启动 入坞程序 失败！</font>");
            } else {
                ui->textEdit_qdebug->append("<font color='green'>入坞程序 启动成功！</font>");
            }
        } else {
            ui->pushButton_start_dock->setText("启动入坞程序");
            if (startDockProcess && startDockProcess->state() == QProcess::Running) {
                startDockProcess->terminate();  // 优雅地停止进程
                if (!startDockProcess->waitForFinished(3000)) {  // 等待最多3秒
                    startDockProcess->kill();  // 如果进程没有正常退出，强制结束进程
                    ui->textEdit_qdebug->append("<font color='red'>入坞程序 被强制终止</font>");
                } else {
                    ui->textEdit_qdebug->append("<font color='red'>入坞程序 已停止</font>");
                }
            }
        }
        
    });
}


MainWindow::~MainWindow()
{
    delete ui;
    delete webChannel;
    delete mapChannel;
}
