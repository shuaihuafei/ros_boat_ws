#include <QApplication>
#include <mainwindow.h>
#include <QMutex>
#include <QFile>
#include <QTextStream>
#include <QDateTime>

void LogMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static QMutex mutex;
    mutex.lock();

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
    QFile logFile("./logs/debug.txt");
    logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
    QTextStream textStream(&logFile);
    textStream << logText << endl;
    logFile.close();

    mutex.unlock();
};

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    qInstallMessageHandler(LogMessageOutput);
    MainWindow w(nullptr,argc,argv);
    w.show();

    return a.exec();
}
