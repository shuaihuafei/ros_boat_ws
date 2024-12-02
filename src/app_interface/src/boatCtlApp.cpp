#include <ros/ros.h>
#include <QApplication>
#include <mainwindow.h>

int main(int argc, char **argv)
{
    QApplication::addLibraryPath(R"(/home/shuai/Qt/5.15.2/gcc_64/plugins)");
    QApplication a(argc, argv);
    MainWindow w(nullptr,argc,argv);
    w.show();

    return a.exec();
}
