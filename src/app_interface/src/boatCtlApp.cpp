#include <ros/ros.h>
#include <QApplication>
#include <mainwindow.h>

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    MainWindow w(nullptr,argc,argv);
    w.show();

    return a.exec();
}
