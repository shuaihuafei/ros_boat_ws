#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"

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

private:
    void initROS(int argc, char **argv);
};

#endif // MAINWINDOW_H
