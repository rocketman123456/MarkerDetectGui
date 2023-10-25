#include "camthread.h"

// 默认摄像头参数，用于调试
static cv::Mat mtx = (cv::Mat_<float>(3, 3)
               << 1.40866789e+03 ,  0.00000000e+00   ,6.83099799e+02,
               0 ,  1.40766322e+03 ,  3.77912727e+02,
               0.0, 0.0, 1.0);
static cv::Mat dist = (cv::Mat_<float>(5, 1)
                << -5.20241866e-01 , 4.41151420e-01 , 1.36131386e-02,
                -7.52254517e-05,  -1.87191381e-01);

CamThread::CamThread(int32_t w,int32_t h,int32_t id, bool gui, QObject *parent)
    : QThread(parent), _width(w), _height(h), _cam_id(id), _isUseGui(gui)
{
    _frameMsg.id = id;
    _camMsg.id = id;
}

CamThread::~CamThread()
{
    _cam.release();
}

bool CamThread::Init()
{
    _pathCamParam.clear();
    _pathCamParam = "/home/developer/cam_parameter";
    _pathCamParam += std::to_string(static_cast<int32_t>(_cam_id));
    _pathCamParam += ".yaml";
    qDebug() << _pathCamParam.c_str();

    _isCalibrated = readCameraParameters(_pathCamParam, _camMatrix, _distCoeffs);
    if(_isCalibrated)
        _camParam.setParams(_camMatrix, _distCoeffs, cv::Size(_width, _height));
    else
        _camParam.setParams(mtx, dist, cv::Size(_width, _height));

    // this is default dictionary
    SetDictionaryForCalibrate(cv::aruco::DICT_4X4_50);
    SetDictionaryNormal(0);
    SetDictionaryFractal(0);

    _labelSize.clear();
    _tracker.clear();

    _cam.open(_cam_id);
    //_cam >> _img;
    //qDebug() << _img.rows << " " << _img.cols;
    _cam.set(cv::CAP_PROP_FRAME_WIDTH, _width);
    _cam.set(cv::CAP_PROP_FRAME_HEIGHT, _height);
    _fps = (int32_t)_cam.get(cv::CAP_PROP_FPS);
    _fps_dt = 1000 / _fps;
    qDebug() << "Cam " << _cam_id << ":" << _fps << " Init Finished";
    return _cam.isOpened();
}

void CamThread::run()
{
    while(1)
    {
        msleep(_fps_dt);
        if(_isCamRun)
            _cam >> _img;
        if(_isCalibrating)
        {
            cv::Mat imageCopy;
            // detect markers
            std::vector<aruco::Marker> detected_markers = _detector.detect(_img);

            // draw results
            //_img.copyTo(imageCopy);
            if(detected_markers.size() > 0)
            {
                qDebug() << "Add Frame: " << _count;
                _calibrator.addView(detected_markers);
                _count++;
            }
            //_img = imageCopy;

            if(_count >= _total_count)
            {
                bool temp = PostCalibration();
                _camMsg.respond = 0x90 + _cam_id;
                _camMsg.state = temp;
                emit CamSignal(_camMsg);
            }
        }
        if(_isUseGui && _isRealtimePlay)
        {
            _frameMsg.img = &_img;
            emit FrameSignal(_frameMsg);
        }
        QMutexLocker locker(&_lock);
        if(_isStop)
        {
            qDebug() << "Cam " << _cam_id << " Thread Finished";
            break;
        }
    }
}

// 标定程序
bool CamThread::StartCalibration(
        int x,
        int y,
        double ls,
        double lm,
        int frame,
        int dic_id)
{
    qDebug() << "Start Calibration";
    float markerLength = ls;
    int dictionaryId = dic_id;

    SetDictionaryForCalibrate(dictionaryId);
    _calibrator.setParams(cv::Size(_width, _height), markerLength, "");
    _detector.setDetectionMode(aruco::DM_NORMAL);

    _total_count = frame;
    _count = 0;

    QMutexLocker locker(&_lock);
    _isCalibrating = true;
    _fps_dt = 100;
    return true;
}

bool CamThread::PostCalibration()
{
    qDebug() << "Post Calibration";
    bool result = _calibrator.getCalibrationResults(_camParam);

    QMutexLocker locker(&_lock);
    _isCalibrating = false;
    _isCalibrated = true;
    _camParam.saveToFile(_pathCamParam);
    _fps_dt = 1000 / _fps;
    return result;
}

void CamThread::CalAruco(int method, int fast_mode)
{
    _timer.restart();

    if(!_img.data)
    {
        _isStop = true;
        qDebug() << "Img Size Error";
        return;
    }

    cv::Mat frame, frame_show;
    if(_isCalibrated)
        undistort(_img, frame, _camMatrix, _distCoeffs);
    else
        frame = _img;
    if(_isUseGui)
    {
        frame.copyTo(frame_show);
    }

    switch (fast_mode) {
    default:
        _detector.setDetectionMode(aruco::DM_NORMAL);
        break;
    case 0:
        _detector.setDetectionMode(aruco::DM_NORMAL);
        break;
    case 1:
        _detector.setDetectionMode(aruco::DM_FAST);
        break;
    case 2:
        _detector.setDetectionMode(aruco::DM_VIDEO_FAST);
        break;
    }

    _marker.clear();
    if(method == 0)
        _marker = _detector.detect(frame);
    else if(method == 1)
        if(_detectorF.detect(frame))
            _marker = _detectorF.getMarkers();

    _tracker.clear();
    for (auto& marker : _marker)
    {
        if(_labelSize.find(marker.id) != _labelSize.end())
            _tracker[marker.id].estimatePose(marker, _camParam, _labelSize[marker.id]);
        else
            _tracker[marker.id].estimatePose(marker, _camParam, _serialMsg.labelsz);
        //std::cout << _tracker[marker.id].getRTMatrix() << std::endl;
    }
    _time_use = _timer.elapsed();

    // 准备数据
    _camMsg.dt = _time_use;
    if(_isUseGui)
    {
        for (unsigned int i = 0; i < _marker.size(); i++)
            _marker[i].draw(frame_show, cv::Scalar(0, 0, 255), 2);
        _img = frame_show;
        _frameMsg.img = &_img;
        emit FrameSignal(_frameMsg);
    }
    // 编码数据用于传输
    QMutexLocker locker(&_lock);
    _camMsg.clear();
    if(_tracker.size() == 1)
    {
        _tracker.begin();
        auto mat = _tracker.begin()->second.getRTMatrix();
        auto tvec = _tracker.begin()->second.getTvec();
        auto revc = _tracker.begin()->second.getRvec();
        if(!mat.empty())
        {
            _camMsg.label_id[0] = _tracker.begin()->first;
            _camMsg.x[0] = tvec.at<float>(0,0);
            _camMsg.y[0] = tvec.at<float>(0,1);
            _camMsg.z[0] = tvec.at<float>(0,2);
            _camMsg.q1[0] = revc.at<float>(0,0);
            _camMsg.q2[0] = revc.at<float>(1,0);
            _camMsg.q3[0] = revc.at<float>(2,0);
        }
    }
    else if(_tracker.size() >= 2)
    {
        uint32_t id_max_first = 0;
        uint32_t id_max_second = 0;
        FindSecondmaxValue(_tracker, id_max_first, id_max_second);
        auto mat_1 = _tracker[id_max_first].getRTMatrix();
        auto mat_2 = _tracker[id_max_second].getRTMatrix();
        auto tvec_1 = _tracker[id_max_first].getTvec();
        auto revc_1 = _tracker[id_max_first].getRvec();
        auto tvec_2 = _tracker[id_max_second].getTvec();
        auto revc_2 = _tracker[id_max_second].getRvec();
        if(!mat_1.empty())
        {
            _camMsg.label_id[0] = id_max_first;
            _camMsg.x[0] = tvec_1.at<float>(0,0);
            _camMsg.y[0] = tvec_1.at<float>(0,1);
            _camMsg.z[0] = tvec_1.at<float>(0,2);
            _camMsg.q1[0] = revc_1.at<float>(0,0);
            _camMsg.q2[0] = revc_1.at<float>(1,0);
            _camMsg.q3[0] = revc_1.at<float>(2,0);
        }
        if(!mat_2.empty())
        {
            _camMsg.label_id[1] = id_max_second;
            _camMsg.x[1] = tvec_2.at<float>(0,0);
            _camMsg.y[1] = tvec_2.at<float>(0,1);
            _camMsg.z[1] = tvec_2.at<float>(0,2);
            _camMsg.q1[1] = revc_2.at<float>(0,0);
            _camMsg.q2[1] = revc_2.at<float>(1,0);
            _camMsg.q3[1] = revc_2.at<float>(2,0);
        }
    }

    qDebug()<<"cam id "<<_camMsg.id;
    qDebug()<<"label id "<<_camMsg.label_id[0];
    qDebug()<<"label x "<<_camMsg.x[0];
    qDebug()<<"label y "<<_camMsg.y[0];
    qDebug()<<"label z "<<_camMsg.z[0];
    qDebug()<<"label q1 "<<_camMsg.q1[0];
    qDebug()<<"label q2 "<<_camMsg.q2[0];
    qDebug()<<"label q3 "<<_camMsg.q3[0];
    qDebug()<<"label id "<<_camMsg.label_id[1];
    qDebug()<<"label x "<<_camMsg.x[1];
    qDebug()<<"label y "<<_camMsg.y[1];
    qDebug()<<"label z "<<_camMsg.z[1];
    qDebug()<<"label q1 "<<_camMsg.q1[1];
    qDebug()<<"label q2 "<<_camMsg.q2[1];
    qDebug()<<"label q3 "<<_camMsg.q3[1];
}

void CamThread::ReceiveMsgSlot(const SerialMsg &msg)
{
    bool tempb = false;
    _serialMsg = msg;
    _camMsg.clear();

    switch(_serialMsg.command)
    {
    default:
        _camMsg.respond = 0x7f;
        emit CamSignal(_camMsg);
        break;
    case 0x7e:
        _camMsg.respond = 0x7e;
        emit CamSignal(_camMsg);
        break;
    case 0x7d:
        _camMsg.respond = 0x7d;
        emit CamSignal(_camMsg);
        break;
    case 'a':
        _camMsg.respond = 'a';
        emit CamSignal(_camMsg);
        break;
    case 0x72:
        _camMsg.respond = 0x73;
        _isCal = false;
        emit CamSignal(_camMsg);
        break;
    case 0x74:
        // calibration
        if(/*msg.id == -1 ||*/ msg.id == _cam_id)
        {
            if(msg.id == -1)
                _camMsg.respond = 0x75;
            else
                _camMsg.respond = 0x90 + _cam_id;
            tempb = StartCalibration(_serialMsg.x, _serialMsg.y,
                                     _serialMsg.sz_marker, _serialMsg.sz_blank,
                                     _serialMsg.frame_count, _serialMsg.dic_id);
            //_camMsg.state = tempb;
            //emit CamSignal(_camMsg);
        }
        break;
    case 0x78:
        // restart
        _camMsg.respond = 0x78;
        emit CamSignal(_camMsg);
        break;
    case 0x7a:
        // stop program
        _camMsg.respond = 0x7a;
        emit CamSignal(_camMsg);
        break;
    case 0x80:
        _camMsg.respond = 0x80;
        _labelSize.insert(std::pair<int,double>(_serialMsg.label_id, _serialMsg.labelsz));
        emit CamSignal(_camMsg);
        break;
    // start calculation
    case 0x70:
        if(_serialMsg.id == -1 || _serialMsg.id == _cam_id)
        {
            _isCal = true;
            if(_serialMsg.id == -1)
            {
                qDebug() << _cam_id << " Calculation";
                _camMsg.respond = 0x71;
            }
            else
            {
                _camMsg.respond = 0x7c;
            }
            emit CamSignal(_camMsg);
            SetDictionaryNormal(_serialMsg.dic_id);
            CalAruco(0);
            _camMsg.respond = 0x76;
            _camMsg.result_time = _time_use;
            emit CamSignal(_camMsg);
        }
        break;
    // start calculation fractal
    case 0x82:
        if(_serialMsg.id == -1 || _serialMsg.id == _cam_id)
        {
            _isCal = true;
            if(_serialMsg.id == -1)
                _camMsg.respond = 0x71;
            else
                _camMsg.respond = 0x7c;
            emit CamSignal(_camMsg);
            SetDictionaryFractal(_serialMsg.label_type);
            CalAruco(1);
            _camMsg.respond = 0x76;
            _camMsg.result_time = _time_use;
            emit CamSignal(_camMsg);
        }
        break;
    }
}

void CamThread::SaveImgSlot(const std::string &name)
{
    QMutexLocker locker(&_lock);
    auto temp = name;
    temp += "_";
    temp += std::to_string(static_cast<int>(_cam_id));
    temp += ".jpg";
    qDebug() << "image save: " << temp.c_str();
    cv::imwrite(temp.c_str(), _img);
    //emit EnableButtonSignal();
}

// 读取标定板数据
bool CamThread::readDetectorParameters(
        const std::string &filename,
        cv::Ptr<cv::aruco::DetectorParameters> &params)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}
// 存储标定板数据
bool CamThread::saveDetectorParameters(
        const std::string &filename,
        cv::Ptr<cv::aruco::DetectorParameters> &params)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened())
        return false;
    fs << "adaptiveThreshWinSizeMin" << params->adaptiveThreshWinSizeMin;
    fs << "adaptiveThreshWinSizeMax" << params->adaptiveThreshWinSizeMax;
    fs << "adaptiveThreshWinSizeStep" << params->adaptiveThreshWinSizeStep;
    fs << "adaptiveThreshConstant" << params->adaptiveThreshConstant;
    fs << "minMarkerPerimeterRate" << params->minMarkerPerimeterRate;
    fs << "maxMarkerPerimeterRate" << params->maxMarkerPerimeterRate;
    fs << "polygonalApproxAccuracyRate" << params->polygonalApproxAccuracyRate;
    fs << "minCornerDistanceRate" << params->minCornerDistanceRate;
    fs << "minDistanceToBorder" << params->minDistanceToBorder;
    fs << "minMarkerDistanceRate" << params->minMarkerDistanceRate;
    fs << "cornerRefinementMethod" << params->cornerRefinementMethod;
    fs << "cornerRefinementWinSize" << params->cornerRefinementWinSize;
    fs << "cornerRefinementMaxIterations" << params->adaptiveThreshWinSizeMin;
    fs << "cornerRefinementMinAccuracy" << params->cornerRefinementMinAccuracy;
    fs << "markerBorderBits" << params->markerBorderBits;
    fs << "perspectiveRemovePixelPerCell" << params->perspectiveRemovePixelPerCell;
    fs << "perspectiveRemoveIgnoredMarginPerCell" << params->perspectiveRemoveIgnoredMarginPerCell;
    fs << "maxErroneousBitsInBorderRate" << params->maxErroneousBitsInBorderRate;
    fs << "minOtsuStdDev" << params->minOtsuStdDev;
    fs << "errorCorrectionRate" << params->errorCorrectionRate;
    return true;
}
// 读取相机参数
bool CamThread::readCameraParameters(
        const std::string &filename,
        cv::Mat &camMatrix,
        cv::Mat &distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        return false;
    }
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}
// 存储相机参数
bool CamThread::saveCameraParams(
        const std::string &filename,
        cv::Size imageSize,
        float aspectRatio,
        int flags,
        const cv::Mat &cameraMatrix,
        const cv::Mat &distCoeffs,
        double totalAvgErr)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened())
    {
        return false;
    }
    time_t tt;
    time(&tt);
    struct tm* t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & cv::CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;
    if(flags != 0)
    {
        sprintf(buf, "flags: %s%s%s%s",
                flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }
    fs << "flags" << flags;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

void CamThread::SetDictionaryFractal(int type)
{
    switch(type)
    {
    case 0:
        _detectorF.setConfiguration("FRACTAL_2L_6");
        break;
    case 1:
        _detectorF.setConfiguration("FRACTAL_3L_6");
        break;
    case 2:
        _detectorF.setConfiguration("FRACTAL_4L_6");
        break;
    case 3:
        _detectorF.setConfiguration("FRACTAL_5L_6");
        break;
    }
}
void CamThread::SetDictionaryNormal(int type)
{
    switch(type)
    {
    case 0:
        _detector.setDictionary("ARUCO_MIP_36h12");
        break;
    case 1:
        _detector.setDictionary("ARUCO");
        break;
    case 2:
        _detector.setDictionary("ARUCO_MIP_25h7");
        break;
    case 3:
        _detector.setDictionary("ARUCO_MIP_16h3");
        break;
    }
}

void CamThread::CheckStateSlot()
{
    _camMsg.respond = 0x77;
    _camMsg.state = CheckCamState();
    emit CamSignal(_camMsg);
}

bool CamThread::CheckCamState()
{
    QMutexLocker locker(&_lock);
    _isStop = !_cam.isOpened();
    return _cam.isOpened();
}

double CamThread::GetDistance(aruco::MarkerPoseTracker t)
{
    auto mat = t.getRTMatrix();
    if(!mat.empty())
    {
        auto tvec = t.getTvec();
        double Z = tvec.at<double>(0,0) * tvec.at<double>(0,0) +
                   tvec.at<double>(1,0) * tvec.at<double>(1,0) +
                   tvec.at<double>(2,0) * tvec.at<double>(2,0);
        return Z;
    }
    else
    {
        return 100*100*100*100;
    }
}

// 寻找最有效的两个码 -- shortest
void CamThread::FindSecondmaxValue(
        std::map<uint32_t, aruco::MarkerPoseTracker> track,
        uint32_t& id_1, uint32_t& id_2)
{
    std::map<uint32_t, aruco::MarkerPoseTracker>::iterator iter;
    iter = track.begin();
    iter++;

    uint32_t i_max = track.begin()->first;
    uint32_t i_max_s = iter->first;
    double dist_max = GetDistance(track.begin()->second);
    double dist_max_s = GetDistance(iter->second);

    iter = track.begin();
    while(iter != track.end())
    {
        double temp = GetDistance(iter->second);
        if(temp <= dist_max)
        {
            dist_max = temp;
            i_max = iter->first;
        }
        else if(temp > dist_max && temp <= dist_max_s)
        {
            dist_max_s = temp;
            i_max_s = iter->first;
        }
        iter++;
    }
    id_1 = i_max;
    id_2 = i_max_s;
}
