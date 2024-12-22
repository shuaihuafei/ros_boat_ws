#include <ros/ros.h>
#include <QApplication>
#include <mainwindow.h>

void MyMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QString logText;
    // 选择日志级别，并构造日志文本
    switch (type) {
    case QtDebugMsg:
        logText = QString("[Debug]%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")+ ":" + msg);
        break;
    case QtInfoMsg:
        logText = QString("[Info]%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")+ ":" + msg);
        break;
    case QtWarningMsg:
        logText = QString("[Warning]%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")+ ":" + msg);
        break;
    case QtCriticalMsg:
        logText = QString("[Critical]%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")+ ":" + msg);
        break;
    case QtFatalMsg:
        logText = QString("[Fatal]%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")+ ":" + msg);
        break;
    default:
        break;
    }
    // 写入日志文件
    QFile logFile("./debug.txt");
    logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
    QTextStream textStream(&logFile);
    textStream << logText << endl;
    logFile.close();
};

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    qInstallMessageHandler(MyMessageOutput);
    MainWindow w(nullptr,argc,argv);
    w.show();

    return a.exec();
}
