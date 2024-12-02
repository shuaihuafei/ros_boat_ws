#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QPixmap>
#include <QPainter>
#include <QRadialGradient>

MainWindow::MainWindow(QWidget *parent,int argc,char** argv)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ros::init(argc, argv, "ControlTurtlesimNode");

    ros::NodeHandle nh;

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


MainWindow::~MainWindow()
{
    delete ui;
}
