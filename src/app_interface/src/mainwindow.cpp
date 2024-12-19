#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent,int argc,char** argv)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , argc_(argc)  // 初始化 argc_
    , argv_(argv)  // 初始化 argv_
{
    ui->setupUi(this);

    init_QProcess();
    init_GUI();

    startRosCore();
    startDockCam();
    startDockCamOrigin();
    startDockCamYolo();
    startGoDock();
}

// 主窗口初始化
void MainWindow::init_GUI()
{
    QPixmap pixmap_logo(":/images/logo.jpg");
    ui->label_log->setPixmap(pixmap_logo);
    ui->label_log->setScaledContents(true);

    QPixmap pixmap_attitude(":/images/attitude.png");
    ui->label_attitude->setPixmap(pixmap_attitude);
    ui->label_attitude->setScaledContents(true);

    QPixmap pixmap_speed(":/images/speed.png");
    ui->label_speed->setPixmap(pixmap_speed);
    ui->label_speed->setScaledContents(true);

    QPixmap pixmap_map(":/images/map.png");
    ui->label_map->setPixmap(pixmap_map);
    ui->label_map->setScaledContents(true);

    // 获取 QLabel 的宽高
    int width = ui->label_status->width();
    int height = ui->label_status->height();

    // 创建一个与 QLabel 同大小的 QPixmap
    QPixmap pixmap_status(width, height);
    pixmap_status.fill(Qt::transparent); // 填充透明背景

    // 使用 QPainter 绘制绿色圆
    QPainter painter(&pixmap_status);
    painter.setRenderHint(QPainter::Antialiasing); // 抗锯齿

    // 设置渐变
    QRadialGradient gradient(QPointF(width / 2, height / 2), std::min(width, height) / 2);
    gradient.setColorAt(0.0, QColor(60, 179, 113));
    gradient.setColorAt(1.0, QColor(34, 139, 34, 0));

    // 应用渐变画刷
    painter.setBrush(QBrush(gradient)); // 设置填充为绿色
    painter.setPen(Qt::NoPen);   // 无边框

    // 计算圆心和半径
    int radius = std::min(width, height) / 2; // 圆的半径为 QLabel 边长的 1/4
    int centerX = width / 2;
    int centerY = height / 2;

    // 在 QLabel 中心绘制圆
    painter.drawEllipse(centerX - radius, centerY - radius, 2 * radius, 2 * radius);

    // 将绘制好的 QPixmap 设置为 QLabel 的内容
    ui->label_status->setPixmap(pixmap_status);
}

void MainWindow::init_QProcess()
{
    roscoreProcess = nullptr;
    startDockProcess = nullptr;
    startDockCamProcess = nullptr;
    startDockCamYoloProcess = nullptr;
}

// 当本机为rosmaster时需要启动roscore
void MainWindow::startRosCore()
{
    connect(ui->radioButton_roscore_true, &QRadioButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            // 启动 roscore
            if (!roscoreProcess) {
                roscoreProcess = new QProcess(this);  // 创建 QProcess 对象
            }
            roscoreProcess->start("roscore");  // 启动 roscore
            if (!roscoreProcess->waitForStarted()) {  // 检查 roscore 是否启动成功
                ui->textEdit_qdebug->append("<font color='red'>启动 roscore 失败！</font>");
            } else {
                ui->textEdit_qdebug->append("<font color='red'>roscore 已启动</font>");
                ros::init(argc_, argv_, "ControlInterfaceNode");
            }
        } else {
            // 停止 roscore
            if (roscoreProcess && roscoreProcess->state() == QProcess::Running) {
                roscoreProcess->terminate();  // 优雅地停止进程
                if (!roscoreProcess->waitForFinished(3000)) {  // 等待最多3秒
                    roscoreProcess->kill();  // 如果进程没有正常退出，强制结束进程
                    ui->textEdit_qdebug->append("<font color='red'>roscore 被强制终止</font>");
                } else {
                    ui->textEdit_qdebug->append("<font color='red'>roscore 已停止</font>");
                }
            }
        }
    });
}

void MainWindow::startDockCam()
{
    connect(ui->pushButton_start_cam_dock, &QPushButton::toggled, [=](bool toggled_flag) {
        if (toggled_flag) {
            ui->pushButton_start_cam_dock->setText("关闭坞舱摄像头");
            // 启动 ROS 节点 (roslaunch)
            startDockCamProcess = new QProcess(this);  // 创建 QProcess 对象
            QString command = "roslaunch";
            QStringList arguments;
            arguments << "cv" << "usb_cam.launch";  // roslaunch 命令和参数
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
            startDockCamOriginImageSubscriber();  // 启动图像订阅
        } else {
            ui->pushButton_cam_dock_origin->setText("开启相机原图像");

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
            QString command = "roslaunch";
            QStringList arguments;
            arguments << "cv" << "02boat_angle.launch";  // roslaunch 命令和参数
            startDockCamYoloProcess->start(command, arguments);  // 启动 roslaunch

            // 捕获标准输出并显示到 textEdit_qdebug
            connect(startDockCamYoloProcess, &QProcess::readyReadStandardOutput, [=]() {
                QString output = startDockCamYoloProcess->readAllStandardOutput();
                // 去除终端控制字符（颜色编码等）
                output.remove(QRegExp("\\x1B\\[[0-9;]*[mK]"));
                QString coloredOutput = "<font color='green'>" + output + "</font>";
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
                startDockCamYoloImageSubscriber();  // 启动图像订阅
            }
        } else {
            ui->pushButton_cam_dock_yolo->setText("开启yolo识别图像");
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

// 启动 ROS 图像订阅
void MainWindow::startDockCamOriginImageSubscriber()
{
    // 初始化 ROS 节点（假设已经在其他地方初始化过 ROS）
    ros::NodeHandle nh;
    
    // 订阅 ROS 图像话题
    ros::Subscriber dockCamOriginImageSubscriber = nh.subscribe("/camera_usb/color/image_raw", 1, &MainWindow::dockCamImageCallback, this);

    ros::spin();
}

// 启动 ROS 图像订阅
void MainWindow::startDockCamYoloImageSubscriber()
{
    // 初始化 ROS 节点（假设已经在其他地方初始化过 ROS）
    ros::NodeHandle nh;
    
    // 订阅 ROS 图像话题
    ros::Subscriber dockCamYoloImageSubscriber = nh.subscribe("/yolo/predict_boat", 1, &MainWindow::dockCamImageCallback, this);

    ros::spin();
}

// 订阅 ROS 图像话题的回调函数
void MainWindow::dockCamImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        // 使用 cv_bridge 将 ROS 图像转换为 OpenCV 图像
        cv_bridge::CvImagePtr cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 转换为 QImage
        QImage img = QImage(cvImagePtr->image.data, cvImagePtr->image.cols, cvImagePtr->image.rows, 
                            cvImagePtr->image.step, QImage::Format_RGB888);

        // 将 QImage 转换为 QPixmap，并更新 QLabel 显示
        QPixmap pixmap = QPixmap::fromImage(img);
        ui->label_cam_dock->setPixmap(pixmap.scaled(ui->label_cam_dock->size(), Qt::KeepAspectRatio));
    }
    catch (cv_bridge::Exception& e) {
        qDebug() << "cv_bridge exception: " << e.what();
    }
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
}
