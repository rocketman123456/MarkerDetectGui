#ifndef DATAPACK_H
#define DATAPACK_H

#include <QMetaType>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

#define CAM_NUM 1

// 与窗口通信
struct FrameMsg
{
    int32_t id = -1;
    cv::Mat *img = nullptr;

    void clear()
    {
        id = -1;
        img = nullptr;
    }
};

// 串口线程发送给摄像头线程
struct SerialMsg
{
    // serial data
    uint8_t command = 0;
    int32_t id = -1;
    int32_t dic_id = 0;
    uint8_t label_id = 0;
    double labelsz = 0.0;
    // 标定用数据
    int32_t x = 0;
    int32_t y = 0;
    double sz_blank = 0.0;
    double sz_marker = 0.0;
    int32_t frame_count = 0;
    int16_t label_type = 0;

    void clear()
    {
        command = 0;
        id = -1;
        dic_id = 0;
        label_id = 0;
        labelsz = 0.0;
        x = 0;
        y = 0;
        sz_blank = 0.0;
        sz_marker = 0.0;
        frame_count = 0;
        label_type = 0;
    }
};

// 摄像头线程发送给串口线程
struct CamMsg
{
    int32_t id = -1;
    uint8_t respond = 0;
    bool state = false;
    double result_time = 0.0;
    // 计算结果返回
    uint8_t label_id[2];
    float x[2],y[2],z[2];
    float q1[2],q2[2],q3[2],q4[2];
    uint32_t dt = 0;

    void clear()
    {
        label_id[0] = 0xff;
        x[0] = 0;
        y[0] = 0;
        z[0] = 0;
        q1[0] = 0;
        q2[0] = 0;
        q3[0] = 0;
        label_id[1] = 0xff;
        x[1] = 0;
        y[1] = 0;
        z[1] = 0;
        q1[1] = 0;
        q2[1] = 0;
        q3[1] = 0;
    }
};

// 存储自检信息等
struct CamCheckInfo
{
    std::map<int32_t, bool> result;
    std::map<int32_t, bool> state;
    std::map<int32_t, double> dt;

    void clear()
    {
        for(auto it=result.begin();it!=result.end();++it)
        {
            it->second = false;
        }
        for(auto it=state.begin();it!=state.end();++it)
        {
             it->second = false;
        }
    }

    int sumResult()
    {
        int _sum = 0;
        for(auto it=result.begin();it!=result.end();++it)
        {
            _sum += it->second;
        }
        return _sum;
    }

    int sumState()
    {
        int _sum = 0;
        for(auto it=state.begin();it!=state.end();++it)
        {
            _sum += it->second;
        }
        return _sum;
    }
};

Q_DECLARE_METATYPE(FrameMsg)
Q_DECLARE_METATYPE(SerialMsg)
Q_DECLARE_METATYPE(CamMsg)
Q_DECLARE_METATYPE(CamCheckInfo)
#endif // DATAPACK_H
