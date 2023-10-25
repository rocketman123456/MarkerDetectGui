#include "mainwindow.h"
#include <QFile>
#include <QIntValidator>
#include <QDoubleValidator>

static const int widths[] = {
    640,
    640,
};
static const int heights[] = {
    480,
    480,
};
static const int ids[] = {
    0,
    2,
};
static const int frame_width = 640;
static const int frame_height = 480;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->IDEdit->setValidator(new QIntValidator(-1, 10, this));
    ui->MarkerEdit->setValidator(new QDoubleValidator(0.0,1000.0,6,this));
    ui->BlankEdit->setValidator(new QDoubleValidator(0.0,1000.0,6,this));
    ui->DicEdit->setValidator(new QIntValidator(0, 3, this));
    ui->CalibrationEdit->setValidator(new QIntValidator(0, 1000, this));
    ui->XEdit->setValidator(new QIntValidator(0, 1000, this));
    ui->YEdit->setValidator(new QIntValidator(0, 1000, this));

    QLabel* frame_ptr[] = {
        ui->label_frame_1,
        ui->label_frame_2,
    };
    bool state[CAM_NUM] = {false};

    for (int i = 0; i < CAM_NUM; ++i)
    {
        _cam_thread[i] = new CamThread(widths[i], heights[i], ids[i], true, this);
        _id_frame_map[ids[i]] = frame_ptr[i];
        // main events
        connect(_cam_thread[i], &CamThread::FrameSignal, this, &MainWindow::GetImageSlot);
        connect(this, &MainWindow::SaveImgSignal, _cam_thread[i], &CamThread::SaveImgSlot);
        // gui specify
        connect(this, &MainWindow::SerialSignal, _cam_thread[i], &CamThread::ReceiveMsgSlot);
        connect(this, &MainWindow::RealtimeSignal, _cam_thread[i], &CamThread::RealTimePlaySlot);
        // on destory
        connect(this, &MainWindow::destroyed, _cam_thread[i], &CamThread::StopSlot);
        connect(_cam_thread[i], &CamThread::finished, _cam_thread[i], &CamThread::deleteLater);
    }

    bool _state = true;
    for (int i = 0; i < CAM_NUM; ++i)
    {
        state[i] = _cam_thread[i]->Init();
        if(!state[i])
            qDebug() << "Cam " << i << " Init Failed.";
        _state = _state & state[i];
    }
    if(!_state)
        exit(1);
    for (int i = 0; i < CAM_NUM; ++i)
    {
        _cam_thread[i]->start();
    }

    QFile file("/home/developer/com_name.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
       qDebug() << "Can't open the file!";
       exit(1);
    }
    QTextStream in(&file);
    QString line = in.readLine();
    qDebug() << line;

    _serial_thread = new SerialThread(line, false, this);
    connect(this, &MainWindow::destroyed, _serial_thread, &SerialThread::StopSlot);
    connect(_serial_thread, &SerialThread::finished, _serial_thread, &SerialThread::deleteLater);
    for (int i = 0; i < CAM_NUM; ++i)
    {
        connect(_serial_thread, &SerialThread::SerialSignal, _cam_thread[i], &CamThread::ReceiveMsgSlot);
        connect(_cam_thread[i], &CamThread::CamSignal, _serial_thread, &SerialThread::ReceiveMsgSlot);
        connect(_serial_thread, &SerialThread::CamCheckSignal, _cam_thread[i], &CamThread::CheckStateSlot);
    }

    _state = _serial_thread->Init();
    if(!_state)
    {
        qDebug() << "Serial Init Failed.";
        exit(1);
    }
    _serial_thread->start();

    bool self_check = false;
    if(self_check)
    {
        // 定时自检程序
        _timer = new QTimer(this);
        connect(_timer, &QTimer::timeout, _serial_thread, &SerialThread::CheckResult);
        _timer->start(1*1000*3);
    }
}

MainWindow::~MainWindow()
{
    for (int i = 0; i < CAM_NUM; ++i)
    {
        _cam_thread[i]->StopSlot();
        _cam_thread[i]->wait();
    }
    _serial_thread->StopSlot();
    _serial_thread->wait();

    delete ui;
}

void MainWindow::GetImageSlot(const FrameMsg &data)
{
    QMutexLocker locker(&_lock);
    int32_t id = data.id;
    _img[id] = *data.img;
    if(!_img[id].empty())
    {
        cv::resize(_img[id], _imgRe[id], cv::Size(frame_width, frame_height), 0, 0, cv::INTER_LINEAR);
        cv::cvtColor(_imgRe[id], _imgRe[id], cv::COLOR_BGR2RGB);
        _imgFrame[id] = QImage(
                    static_cast<const unsigned char*>(_imgRe[id].data),
                    frame_width, frame_height, QImage::Format_RGB888);
        if(_id_frame_map.find(id) != _id_frame_map.end() && _id_frame_map[id] != nullptr)
            _id_frame_map[id]->setPixmap(QPixmap::fromImage(_imgFrame[id]));
    }
}

void MainWindow::on_save_img_btn_clicked()
{
    _frameCount++;
    _path.clear();
    _path = "/home/developer/Pictures/img_save/";
    _path += std::to_string(static_cast<int>(_frameCount));
    emit SaveImgSignal(_path);
}

void MainWindow::on_start_calculation_btn_clicked()
{
    _serialMsg.command = 0x70;
    _serialMsg.id = ui->IDEdit->text().toInt();
    _serialMsg.labelsz = ui->MarkerEdit->text().toDouble();
    _serialMsg.dic_id = ui->DicEdit->text().toInt();
    qDebug() << "Marker Size:" << _serialMsg.labelsz;
    emit SerialSignal(_serialMsg);
}

void MainWindow::on_realtime_play_btn_clicked()
{
    emit RealtimeSignal();
}

void MainWindow::on_start_calibration_btn_clicked()
{
    _serialMsg.command = 0x74;
    _serialMsg.id = ui->IDEdit->text().toInt();
    _serialMsg.dic_id = ui->DicEdit->text().toInt();
    _serialMsg.x = ui->XEdit->text().toInt();
    _serialMsg.y = ui->YEdit->text().toInt();
    _serialMsg.sz_marker = ui->MarkerEdit->text().toDouble();
    _serialMsg.sz_blank = ui->BlankEdit->text().toDouble();
    _serialMsg.frame_count = ui->CalibrationEdit->text().toDouble();
    emit SerialSignal(_serialMsg);
}
