#ifndef CAMTHREAD_H
#define CAMTHREAD_H

#include <QThread>
#include <QMutex>
#include <QDebug>
#include <QTime>
#include <QElapsedTimer>

#include <iostream>
#include <stdio.h>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>

#include <aruco/aruco.h>
//#include <aruco/aruco_cvversioning.h>
#include "calibrator.h"

#include "datapack.h"

class CamThread : public QThread
{
    Q_OBJECT
public:
    explicit CamThread(int32_t w,int32_t h,int32_t id, bool gui = true, QObject* parent = nullptr);
    ~CamThread();

    bool Init();

    inline void SetDictionaryForCalibrate(int dic_id) { _dictionary =cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dic_id)); }
    void SetDictionaryNormal(int type);
    void SetDictionaryFractal(int type);

    void CalAruco(int method, int fast_mode = 0);
    bool StartCalibration(int x, int y, double ls, double lm, int frame, int dic_id);
    bool PostCalibration();

    bool readDetectorParameters(const std::string &filename,
        cv::Ptr<cv::aruco::DetectorParameters> &params);
    bool saveDetectorParameters(const std::string &filename,
        cv::Ptr<cv::aruco::DetectorParameters> &params);
    bool saveCameraParams(const std::string &filename,
        cv::Size imageSize,
        float aspectRatio, int flags,
        const cv::Mat &cameraMatrix,
        const cv::Mat &distCoeffs,
        double totalAvgErr);
    bool readCameraParameters(const std::string &filename,
        cv::Mat &camMatrix, cv::Mat &distCoeffs);

protected:
    void FindSecondmaxValue(std::map<uint32_t,aruco::MarkerPoseTracker> track, uint32_t& id_1, uint32_t& id_2);
    double GetDistance(aruco::MarkerPoseTracker t);
    bool CheckCamState();

protected:
    //QThread的虚函数
    //线程处理函数
    //不能直接调用，通过start()间接调用
    void run();

public slots:
    void StopSlot() { QMutexLocker locker(&_lock); _isStop = true; }
    void SaveImgSlot(const std::string &name);
    void ReceiveMsgSlot(const SerialMsg &msg);
    void CheckStateSlot();
    void RealTimePlaySlot() {QMutexLocker locker(&_lock); _isRealtimePlay = !_isRealtimePlay;}

signals:
    void FrameSignal(const FrameMsg &data);
    void CamSignal(const CamMsg &msg);

protected:
    // camera parameter
    int32_t             _width = 0;
    int32_t             _height = 0;
    int32_t             _cam_id = -1;
    int32_t             _fps = 0;
    int32_t             _fps_dt = 0;
    // state flag
    bool                _isCalibrated = false;
    bool                _isRealtimePlay = false;
    bool                _isUseGui = true;
    bool                _isCalibrating = false;
    bool                _isStop = false;
    bool                _isCamRun = true;
    bool                _isCal = false;
    double              _time_use = 0.0;
    // thread lock
    QMutex              _lock;
    QElapsedTimer       _timer;
    // camera variable
    cv::VideoCapture    _cam;
    cv::Mat             _img;
    cv::Mat             _camMatrix;
    cv::Mat             _distCoeffs;
    // camera parameter
    std::string         _pathCamParam;
    aruco::CameraParameters _camParam;
    aruco::MarkerMap        _markerMap;
    // message
    FrameMsg            _frameMsg; // send
    SerialMsg           _serialMsg; // get
    CamMsg              _camMsg; // send
    // for calibration
    int                                     _total_count = 0;
    int                                     _count = 0;
    //bool                                    _refindStrategy = false;
    cv::Ptr<cv::aruco::Dictionary>          _dictionary;
    //std::vector<std::vector<aruco::Marker>> _allMarkers;
    //cv::Ptr<cv::aruco::DetectorParameters>  _params;
    //aruco::MarkerDetector                   TheMarkerDetector;
    aruco::Calibrator                       _calibrator;
    // collect data from each frame
    cv::Size                                _imgSize;
    // for aruco-fractal mark : origin lib
    std::map<int,double>                    _labelSize;
    cv::Mat                                 _rvecsf, _tvecsf;
    aruco::FractalDetector                  _detectorF;
    aruco::MarkerDetector                   _detector;
    std::vector<aruco::Marker>              _marker;
    std::map<uint32_t, aruco::MarkerPoseTracker> _tracker;
};

aruco::CameraParameters cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,float *currRepjErr=0, aruco::MarkerMap *inmmap=0);

#endif // CAMTHREAD_H
