#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QElapsedTimer>
#include "./ui_mainwindow.h"

#include <map>

#include "datapack.h"
#include "camthread.h"
#include "serialthread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void GetImageSlot(const FrameMsg &data);

private slots:
    void on_save_img_btn_clicked();
    void on_start_calculation_btn_clicked();
    void on_realtime_play_btn_clicked();

    void on_start_calibration_btn_clicked();

signals:
    void SerialSignal(const SerialMsg &msg);
    void SaveImgSignal(const std::string& name) const;
    void RealtimeSignal();

private:
    Ui::MainWindow  *ui;
    QTimer          *_timer;
    QMutex          _lock; // thread lock
    //
    CamThread       *_cam_thread[CAM_NUM] = { nullptr };
    SerialThread    *_serial_thread = nullptr;
    //
    std::string             _path;
    int64_t                 _frameCount = 0;
    std::map<int, cv::Mat>  _img;
    std::map<int, cv::Mat>  _imgRe;
    std::map<int, QImage>   _imgFrame;
    std::map<int, QLabel*>  _id_frame_map;
    //
    SerialMsg       _serialMsg; // send
    CamCheckInfo    _infoCam;
};

#endif // MAINWINDOW_H
