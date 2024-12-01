#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent,int argc,char** argv)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initROS(argc,argv);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initROS(int argc,char** argv)
{
    ros::init(argc, argv, "ControlTurtlesimNode");
    ros::NodeHandle nh;
}
