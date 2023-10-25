#ifndef SERIALTHREAD_H
#define SERIALTHREAD_H

#include <QThread>
#include <QMutex>
#include <QDebug>
#include <QTimer>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <iostream>
#include <string>
#include <stdio.h>

#include "datapack.h"

class SerialThread : public QThread
{
    Q_OBJECT
public:
    explicit SerialThread(const QString &port, bool isEnable, QObject* parent = nullptr)
        : QThread(parent), _port(port), _useSerial(isEnable) { /*_serial = new QSerialPort();*/ }
    ~SerialThread();

    bool Init();

signals:
    void SerialSignal(const SerialMsg &msg);
    void CamCheckSignal();
    void EnableButtonSignal();

public slots:
    void StopSlot() { QMutexLocker locker(&_lock); _isStop = true; }
    void ReadDataSlot();
    void ReceiveMsgSlot(const CamMsg &msg);
    void CheckResult() { emit CamCheckSignal(); }

protected:
    void run();
    inline void copyDouble2Bit(double t, uint8_t *bits) { memcpy(bits, &t, sizeof(double)); }
    inline void copyBit2Double(uint8_t *bits, double &t) { memcpy(&t, bits, sizeof(double)); }
    inline void copyInt2Bit(int t, uint8_t *bits) { memcpy(bits, &t, sizeof(int)); }
    inline void copyBit2Int(uint8_t *bits, int &t) { memcpy(&t, bits, sizeof(int)); }
    inline void copyFloat2Bit(float t, uint8_t *bits) { memcpy(bits, &t, sizeof(int)); }
    inline void copyBit2Float(uint8_t *bits, float &t) { memcpy(&t, bits, sizeof(int)); }

protected:
    QString         _port;
    QByteArray      _bufRead;
    QByteArray      _bufSend;
    QByteArray      _hexData;
    QSerialPort*    _serial;

    QMutex          _lock;
    QTimer*         _timer;

    bool            _useSerial = false;
    bool            _isStop = false;

    SerialMsg       _serialMsg; // send
    CamMsg          _camMsg; // get
    CamCheckInfo    _infoCam;
};

#endif // SERIALTHREAD_H
