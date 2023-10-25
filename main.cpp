#include "mainwindow.h"

#include <QApplication>
#include <QMetaType>

#include "datapack.h"

#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <QtMsgHandler>
#include <QMessageLogContext>
#include <QMutex>

void myMsgOutput(QtMsgType type, const QMessageLogContext &context, const QString& msg)
{
    static QMutex mutex;
    mutex.lock();
    QString time=QDateTime::currentDateTime().toString(QString("[ yyyy-MM-dd HH:mm:ss:zzz ]"));
    QString mmsg;
    switch(type)
    {
    case QtDebugMsg:
        mmsg=QString("%1: Debug:\t%2 (file:%3, line:%4, func: %5)").arg(time).arg(msg).arg(QString(context.file)).arg(context.line).arg(QString(context.function));
        break;
    case QtInfoMsg:
        mmsg=QString("%1: Info:\t%2 (file:%3, line:%4, func: %5)").arg(time).arg(msg).arg(QString(context.file)).arg(context.line).arg(QString(context.function));
        break;
    case QtWarningMsg:
        mmsg=QString("%1: Warning:\t%2 (file:%3, line:%4, func: %5)").arg(time).arg(msg).arg(QString(context.file)).arg(context.line).arg(QString(context.function));
        break;
    case QtCriticalMsg:
        mmsg=QString("%1: Critical:\t%2 (file:%3, line:%4, func: %5)").arg(time).arg(msg).arg(QString(context.file)).arg(context.line).arg(QString(context.function));
        break;
    case QtFatalMsg:
        mmsg=QString("%1: Fatal:\t%2 (file:%3, line:%4, func: %5)").arg(time).arg(msg).arg(QString(context.file)).arg(context.line).arg(QString(context.function));
        abort();
    }
    QFile file("/home/developer/debug.txt");
    file.open(QIODevice::ReadWrite | QIODevice::Append);
    QTextStream stream(&file);
    stream << mmsg << "\r\n";
    file.flush();
    file.close();
    mutex.unlock();
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    //qInstallMessageHandler(myMsgOutput);

    // register msg data type
    qRegisterMetaType<FrameMsg>("FrameMsg");
    qRegisterMetaType<FrameMsg>("FrameMsg&");
    qRegisterMetaType<SerialMsg>("SerialMsg&");
    qRegisterMetaType<SerialMsg>("SerialMsg");
    qRegisterMetaType<CamMsg>("CamMsg");
    qRegisterMetaType<CamMsg>("CamMsg&");
    qRegisterMetaType<CamCheckInfo>("CamCheckInfo");
    qRegisterMetaType<CamCheckInfo>("CamCheckInfo&");

    MainWindow window;
    window.show();

    return app.exec();
}
