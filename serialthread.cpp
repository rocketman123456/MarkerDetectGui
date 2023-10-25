#include "serialthread.h"

#include <QCoreApplication>

#include <unistd.h>
#include <sys/reboot.h>

uint8_t CheckSum(QByteArray &buf)
{
    uint8_t sum = 0;
    for (int i = 2; i < buf.size(); ++i)
    {
        sum += buf[i];
    }
    return sum;
}

SerialThread::~SerialThread()
{
    if(_useSerial)
    {
        _serial->clear();
        _serial->close();
        _serial->deleteLater();
        delete _serial;
    }
}

bool SerialThread::Init()
{
    //QMutexLocker locker(&_lock);
    _serial = new QSerialPort(this);
    if(_useSerial)
    {
        //qDebug() << _port;
        //_serial->setPortName(_port);
        //_isStop = !_serial->open(QIODevice::ReadWrite);
        _isStop = true;
        foreach (const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
        {
            _serial->setPort(info);
            if(_serial->open(QIODevice::ReadWrite))
            {
                qDebug() << "Open: " << info.portName();
                _isStop = false;
                break;
            }
        }
        qDebug() << _isStop;
        if(!_isStop)
        {
            _serial->setBaudRate(QSerialPort::Baud115200);
            _serial->setDataBits(QSerialPort::Data8);
            _serial->setStopBits(QSerialPort::OneStop);
            _serial->setParity(QSerialPort::NoParity);
            _serial->setFlowControl(QSerialPort::NoFlowControl);

            connect(_serial, &QSerialPort::readyRead, this, &SerialThread::ReadDataSlot);

            // 发送启动成功消息
            //_bufSend.push_back("hello Vision!");
            _bufSend.clear();
            _bufSend.push_back((char)0x55);
            _bufSend.push_back((char)0xaa);
            _bufSend.push_back((char)0x00);
            _bufSend.push_back((char)0x46);
            _bufSend.push_back((char)0x47);
            // start init finished
            _bufSend.push_back((char)0x7b);
            _bufSend.push_back((char)0x00);
            // cal check sum
            _bufSend.push_back((char)0x00);
            _bufSend[2] = _bufSend.size()-4;
            auto sum = CheckSum(_bufSend);
            _bufSend[_bufSend.size()-1] = sum;
            // send data
            _serial->write(_bufSend);
            _bufSend.clear();
        }

        return !_isStop;
    }
    else
    {
        return true;
    }
}

void SerialThread::run()
{
    while(1)
    {
        msleep(500);
        QMutexLocker locker(&_lock);
        if(_isStop)
        {
            qDebug() << "Serial Thread Exit.";
            break;
        }
    }
}

void SerialThread::ReadDataSlot()
{
    if(_useSerial)
    {
        _bufRead = _serial->readAll();
        _hexData = _bufRead.toHex();
    }
    uint8_t convertBit[8] = { 0 };
    double tempd = 0.0;
    int tempi = 0;
    if(!_bufRead.isEmpty())
    {
        qDebug() << "Get: " << _hexData;

        auto sum = CheckSum(_bufRead);
        if(sum != _bufRead.back())
        {
            qDebug() << "Check sum error.";
            _serialMsg.command = 0x7e;
            emit SerialSignal(_serialMsg);
            return;
        }
        if(_bufRead.size() != _bufRead[2] + 4)
        {
            qDebug() << "Length error.";
            _serialMsg.command = 0x7d;
            emit SerialSignal(_serialMsg);
            return;
        }

        // 解析数据
        int index = 5;
        switch(static_cast<int>(_bufRead[index]))
        {
        default:
            _serialMsg.command = 0x7f;
            break;
        // test command
        case 'a':
            _serialMsg.command = 'a';
            break;
        case 0x70:
            // start cal
            _serialMsg.command = 0x70;
            if(static_cast<int>(_bufRead[index + 2]) != 0xff)
                _serialMsg.id = _bufRead[index + 2];
            else
                _serialMsg.id =-1;
            _serialMsg.dic_id = _bufRead[index + 3];
            break;
        case 0x72:
            // stop cal
            _serialMsg.command = 0x72;
            break;
        case 0x74:
            // start calibration
            _serialMsg.command = 0x74;
            // set cam id
            if(static_cast<int>(_bufRead[index + 2]) != 0xff)
                _serialMsg.id = _bufRead[index + 2];
            else
                _serialMsg.id =-1;
            // num x
            tempi = 0;
            for(int i = 0; i < 4; ++i)
            {
                convertBit[i] = _bufRead[index + 3 + i];
            }
            copyBit2Int(convertBit, tempi);
            _serialMsg.x = tempi;
            // num y
            tempi = 0;
            for(int i = 0; i < 4; ++i)
            {
                convertBit[i] = _bufRead[index + 7 + i];
            }
            copyBit2Int(convertBit, tempi);
            _serialMsg.y = tempi;
            // size marker
            tempd = 0.0;
            for(int i = 0; i < 8; ++i)
            {
                convertBit[i] = _bufRead[index + 11 + i];
            }
            copyBit2Double(convertBit, tempd);
            _serialMsg.sz_marker = tempd;
            // size blank
            tempd = 0.0;
            for(int i = 0; i < 8; ++i)
            {
                convertBit[i] = _bufRead[index + 19 + i];
            }
            copyBit2Double(convertBit, tempd);
            _serialMsg.sz_blank = tempd;
            // frame_count
            tempi = 0;
            for(int i = 0; i < 4; ++i)
            {
                convertBit[i] = _bufRead[index + 27 + i];
            }
            copyBit2Int(convertBit, tempi);
            _serialMsg.frame_count = tempi;
            // dic id
            _serialMsg.dic_id = _bufRead[index + 31];
            break;
        case 0x78:
            _serialMsg.command = 0x78;
            // 同步磁盘数据,将缓存数据回写到硬盘,以防数据丢失
            sync();
            reboot(RB_AUTOBOOT);
            break;
        case 0x7a:
            _serialMsg.command = 0x7a;
            QCoreApplication::quit();
            break;
        // set label data
        case 0x80:
            _serialMsg.command = 0x80;
            // label id
            _serialMsg.label_id = _bufRead[index + 2];
            // set label size
            for(int i = 0; i < 8; ++i)
            {
                convertBit[i] = _bufRead[index + 2 + i];
            }
            tempd = 0.0;
            copyBit2Double(convertBit, tempd);
            _serialMsg.labelsz = tempd;
            break;
        case 0x82:
            _serialMsg.command = 0x82;
            if(static_cast<int>(_bufRead[index + 2]) != 0xff)
                _serialMsg.id = _bufRead[index + 2];
            else
                _serialMsg.id =-1;
            _serialMsg.label_type = _bufRead[index + 3];
            break;
        }
        emit SerialSignal(_serialMsg);
    }
    _bufRead.clear();
}

void SerialThread::ReceiveMsgSlot(const CamMsg &msg)
{
    QMutexLocker locker(&_lock);
    qDebug() << "GetCam: " << msg.id << " " << msg.respond;
    _camMsg = msg;

    bool canSend = false;
    bool isBothGet = false;
    _infoCam.result[_camMsg.id] = true;
    int total_result = _infoCam.sumResult();
    isBothGet = (total_result == CAM_NUM);

    // need 8 bit to store a double
    // need 4 bit to store int
    // need 4 bit to store float
    uint8_t convertBit[8] = { 0 };

    _bufSend.clear();
    _bufSend.push_back((char)0x55);
    _bufSend.push_back((char)0xaa);
    // this is total command length
    _bufSend.push_back((char)0x00);
    _bufSend.push_back((char)0x46);
    _bufSend.push_back((char)0x47);

    // 编码数据用于发送
    switch(_camMsg.respond)
    {
    // unknown command
    default:
        if(isBothGet)
        {
            _bufSend.push_back((char)0x7f);
            _bufSend.push_back((char)0x00);
            canSend = true;
        }
        break;
    // chenksum error
    case 0x7e:
        if(isBothGet)
        {
            _bufSend.push_back((char)0x7e);
            _bufSend.push_back((char)0x00);
            canSend = true;
        }
        break;
    // total length error
    case 0x7d:
        if(isBothGet)
        {
            _bufSend.push_back((char)0x7d);
            _bufSend.push_back((char)0x00);
            canSend = true;
        }
        break;
    // test
    case 'a':
        if(isBothGet)
        {
            _bufSend.push_back((char)0x01);
            _bufSend.push_back((char)0x00);
            _bufSend.push_back((char)0x02);
            canSend = true;
        }
        break;
    // self check
    case 0x77:
        _infoCam.state[msg.id] = msg.state;
        if(isBothGet)
        {
            _bufSend.push_back((char)0x77);
            _bufSend.push_back((char)0x00);
            if(_infoCam.state[0])
            {
                _bufSend.push_back((char)0x01);
            }
            else
            {
                _bufSend.push_back((char)0x02);
            }
            if(_infoCam.state[1])
            {
                _bufSend.push_back((char)0x01);
            }
            else
            {
                _bufSend.push_back((char)0x02);
            }
            _bufSend.push_back((char)0xaa);
            _bufSend.push_back((char)0xbb);
            canSend = true;
        }
        break;
    case 0x73:
        // stop calculation
        if(isBothGet)
        {
            _bufSend.push_back((char)0x73);
            _bufSend.push_back((char)0x00);
            canSend = true;
        }
        break;
    // start calculation
    case 0x71:
    // single cam calculation
    case 0x7c:
        _bufSend.push_back((char)0x71);
        _bufSend.push_back((char)0x00);
        _infoCam.result[_camMsg.id] = false;
        canSend = true;
        break;
    // calculation result
    // once send one data
    case 0x76:
        /*qDebug()<<"cam id "<<_camMsg.id;
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
        qDebug()<<"label q3 "<<_camMsg.q3[1];*/

        _bufSend.push_back((char)0x76);
        _bufSend.push_back((char)0x00);
        _bufSend.push_back((char)_camMsg.id);
        for(int i = 0; i < 2; ++i)
        {
            // label id
            _bufSend.push_back((char)_camMsg.label_id[i]);
            // pos
            copyFloat2Bit(_camMsg.x[i], convertBit);
            for(int i = 0; i < sizeof(float); ++i)
            {
                _bufSend.push_back((char)convertBit[i]);
            }
            copyFloat2Bit(_camMsg.y[i], convertBit);
            for(int i = 0; i < sizeof(float); ++i)
            {
                _bufSend.push_back((char)convertBit[i]);
            }
            copyFloat2Bit(_camMsg.z[i], convertBit);
            for(int i = 0; i < sizeof(float); ++i)
            {
                _bufSend.push_back((char)convertBit[i]);
            }
            // rotation
            copyFloat2Bit(_camMsg.q1[i], convertBit);
            for(int i = 0; i < sizeof(float); ++i)
            {
                _bufSend.push_back((char)convertBit[i]);
            }
            copyFloat2Bit(_camMsg.q2[i], convertBit);
            for(int i = 0; i < sizeof(float); ++i)
            {
                _bufSend.push_back((char)convertBit[i]);
            }
            copyFloat2Bit(_camMsg.q3[i], convertBit);
            for(int i = 0; i < sizeof(float); ++i)
            {
                _bufSend.push_back((char)convertBit[i]);
            }
            // dt
            copyInt2Bit(_camMsg.dt, convertBit);
            for(int i = 0; i < sizeof(int); ++i)
            {
                _bufSend.push_back((char)convertBit[i]);
            }
        }
        _infoCam.result[_camMsg.id] = false;
        canSend = true;
        break;
    // calibration result
    case 0x75:
        if(isBothGet)
        {
            _bufSend.push_back((char)0x75);
            _bufSend.push_back((char)0x00);
            _bufSend.push_back((char)_camMsg.state);
            canSend = true;
        }
        break;
    // calibration result single cam
    case 0x90:
    case 0x91:
    case 0x92:
    case 0x93:
    case 0x94:
    case 0x95:
    case 0x96:
    case 0x97:
    case 0x99:
    case 0x9a:
    case 0x9b:
        _bufSend.push_back((char)0x75);
        _bufSend.push_back((char)0x00);
        _bufSend.push_back((char)_camMsg.state);
        _infoCam.result[_camMsg.id] = false;
        canSend = true;
        break;
    case 0x78:
        // reboot
        break;
    case 0x7a:
        // stop program
        break;
    // get label data
    case 0x80:
        if(isBothGet)
        {
            _bufSend.push_back((char)0x81);
            _bufSend.push_back((char)0x00);
            canSend = true;
        }
        break;
    }

    if(isBothGet)
        _infoCam.clear();

    if(canSend)
    {
        // for gui
        emit EnableButtonSignal();
        // cal check sum
        _bufSend.push_back((char)0x00);
        _bufSend[2] = _bufSend.size() - 4;
        auto sum = CheckSum(_bufSend);
        _bufSend[_bufSend.size() - 1] = sum;
        if(_useSerial)
            _serial->write(_bufSend);
        _hexData = _bufSend.toHex();
        qDebug() << "Send: " << _hexData;
        _bufSend.clear();
    }
}
