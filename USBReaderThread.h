#ifndef USBREADERTHREAD_H
#define USBREADERTHREAD_H


#include <QTimer>
#include <QThread>
#include <QMutex>
#include <QQueue>
#include <QResizeEvent>
#include <QShowEvent>
#include <QDateTime>
#include <QElapsedTimer>
#include <QDebug>
#include <libusb-1.0/libusb.h>
#include "qcustomplot.h"
#include <memory>
#include <atomic>
#include <deque>
#include <algorithm>
#include "USBDebugHelper.h"

// USB设备信息结构
struct USBDeviceInfo {
    libusb_device* device;
    uint16_t vendorId;
    uint16_t productId;
    QString manufacturer;
    QString product;
    QString description;

    USBDeviceInfo() : device(nullptr), vendorId(0), productId(0) {}
};

// USB数据读取线程
class USBReaderThread : public QThread
{
    Q_OBJECT

public:
    explicit USBReaderThread(QObject *parent = nullptr);
    ~USBReaderThread();

    bool setDevice(libusb_device* device);
    void stopReading();

    // 配置参数
    void setEndpoint(uint8_t endpoint) { m_endpoint = endpoint; }
    void setChunkSize(int size) { m_chunkSize = size; }
    void setTimeout(int timeout) { m_timeout = timeout; }

protected:
    void run() override;

signals:
    void dataReceived(const QByteArray& data);
    void errorOccurred(const QString& error);
    void statisticsUpdated(quint64 bytesPerSecond, quint32 samplesPerSecond);

private:
    libusb_context* m_context;
    libusb_device_handle* m_deviceHandle;
    std::atomic<bool> m_stopRequested;

    uint8_t m_endpoint;
    int m_chunkSize;
    int m_timeout;

    // 性能统计
    quint64 m_bytesReceived;
    quint32 m_samplesReceived;
    qint64 m_lastStatTime;

    qint64 m_lastStatTime2;
};

#endif // USBREADERTHREAD_H
