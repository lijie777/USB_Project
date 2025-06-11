#include "USBReaderThread.h"
#include <algorithm>
#include <cstring>

#define ENDPOINT 0X86      //接收端点
#define CHUNKSIZE 4*1024  //接收的缓存
#define TIMEOUT 10         //接收数据的超时时间

#define SLEEPTIME 1000     //线程睡眠时间(ms)

// 每次读取4096个字节，每次睡眠时间为800，传输速率大概为5KB/s
// 每次读取4096个字节，每次睡眠时间为1000，传输速率大概为4KB/s

// USB读取线程实现
USBReaderThread::USBReaderThread(QObject* parent)
    : QThread(parent),
      m_context(nullptr),
      m_deviceHandle(nullptr),
      m_stopRequested(false),
      m_endpoint(ENDPOINT),  // 默认输入端点
      m_chunkSize(CHUNKSIZE),
      m_timeout(TIMEOUT),
      m_bytesReceived(0),
      m_samplesReceived(0),
      m_lastStatTime(0)
{
}

USBReaderThread::~USBReaderThread()
{
    stopReading();
    if (m_deviceHandle) {
        libusb_close(m_deviceHandle);
    }
    if (m_context) {
        libusb_exit(m_context);
    }
}

bool USBReaderThread::setDevice(libusb_device* device)
{
    qDebug() << "=== 开始设置USB设备 ===";

    // 首先分析设备
    USBDebugHelper::analyzeDevice(device);

    // 初始化libusb上下文
    int result = libusb_init(&m_context);
    if (result < 0) {
        emit errorOccurred(QString("无法初始化libusb: %1").arg(libusb_strerror((libusb_error)result)));
        return false;
    }

    // 打开设备
    result = libusb_open(device, &m_deviceHandle);
    if (result < 0) {
        emit errorOccurred(QString("无法打开USB设备: %1").arg(libusb_strerror((libusb_error)result)));
        libusb_exit(m_context);
        m_context = nullptr;
        return false;
    }

    qDebug() << "设备打开成功";

    // 检查设备状态
    USBDebugHelper::checkDeviceStatus(m_deviceHandle);

    // 分离内核驱动（如果需要）
    for (int interface = 0; interface < 2; interface++) {
        if (libusb_kernel_driver_active(m_deviceHandle, interface) == 1) {
            qDebug() << QString("分离接口 %1 的内核驱动").arg(interface);
            result = libusb_detach_kernel_driver(m_deviceHandle, interface);
            if (result < 0) {
                qDebug() << QString("无法分离接口 %1 的内核驱动: %2").arg(interface).arg(libusb_strerror((libusb_error)result));
            }
        }
    }

    // 设置配置
    result = libusb_set_configuration(m_deviceHandle, 1);
    if (result < 0) {
        qDebug() << QString("设置配置失败: %1").arg(libusb_strerror((libusb_error)result));
        // 不一定是致命错误，继续尝试
    } else {
        qDebug() << "设置配置成功";
    }

    // 声明接口
    result = libusb_claim_interface(m_deviceHandle, 0);
    if (result < 0) {
        emit errorOccurred(QString("无法声明USB接口: %1").arg(libusb_strerror((libusb_error)result)));
        return false;
    }

    qDebug() << "接口声明成功";

    // 查找工作的端点
    uint8_t workingEndpoint = USBDebugHelper::findWorkingEndpoint(m_deviceHandle);
    if (workingEndpoint != 0) {
        m_endpoint = workingEndpoint;
        qDebug() << QString("使用端点: 0x%1").arg(m_endpoint, 2, 16, QChar('0'));
    } else {
        qDebug() << QString("未找到工作端点，使用默认端点: 0x%1").arg(m_endpoint, 2, 16, QChar('0'));
    }

    // 获取端点最大包大小并调整块大小
    int maxPacketSize = USBDebugHelper::getEndpointMaxPacketSize(device, m_endpoint);
    if (maxPacketSize > 0) {
        // 设置块大小为最大包大小的倍数
//        m_chunkSize = maxPacketSize * 8;  // 8倍最大包大小
        qDebug() << QString("端点最大包大小: %1, 设置块大小: %2").arg(maxPacketSize).arg(m_chunkSize);
    }

    // 尝试激活设备
//    if (USBDebugHelper::tryActivateDevice(m_deviceHandle)) {
//        qDebug() << "设备激活命令执行成功";
//    }

    qDebug() << "=== USB设备设置完成 ===";
    return true;
}

void USBReaderThread::stopReading()
{
    m_stopRequested = true;
    wait(3000);  // 等待最多3秒
}

void USBReaderThread::run()
{
    qDebug() << "=== USB读取线程开始 ===";
    qDebug() << QString("配置: 端点=0x%1, 块大小=%2, 超时=%3ms").arg(m_endpoint, 2, 16, QChar('0')).arg(m_chunkSize).arg(m_timeout);

    m_stopRequested = false;
    m_bytesReceived = 0;
    m_samplesReceived = 0;
    m_lastStatTime = QDateTime::currentMSecsSinceEpoch();

    QByteArray buffer;
    buffer.resize(m_chunkSize);

    int consecutiveTimeouts = 0;
    int maxConsecutiveTimeouts = 10;  // 连续超时限制

    while (!m_stopRequested) {
        int actualLength = 0;

        // 从USB设备读取数据
        int result =
            libusb_bulk_transfer(m_deviceHandle, m_endpoint, reinterpret_cast<unsigned char*>(buffer.data()),
                                 m_chunkSize, &actualLength, m_timeout);

        if (result == LIBUSB_SUCCESS && actualLength > 0) {
            // 重置超时计数器
            consecutiveTimeouts = 0;

            // 更新统计信息
            m_bytesReceived += actualLength;
            m_samplesReceived += actualLength / 2;  // 每2字节一个样本

            // 发送接收到的数据
            QByteArray receivedData = buffer.left(actualLength);
            emit dataReceived(receivedData);

            qDebug() << "=====================接收到一次数据==========================";

            // 更新性能统计（每秒一次）
            qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
            if (currentTime - m_lastStatTime >= 1000) {

                quint64 bytesPerSecond = m_bytesReceived * 1000 / (currentTime - m_lastStatTime);
                quint32 samplesPerSecond = m_samplesReceived * 1000 / (currentTime - m_lastStatTime);

                emit statisticsUpdated(bytesPerSecond, samplesPerSecond);

//                qDebug() << QString("性能统计: %1 KB/s, %2 样本/s").arg(bytesPerSecond / 1024.0, 0, 'f', 1).arg(samplesPerSecond);

                m_bytesReceived = 0;
                m_samplesReceived = 0;
                m_lastStatTime = currentTime;
            }

        }else if(result == LIBUSB_SUCCESS && actualLength == 0){//libusb_bulk_transfer不设置超时，可能会执行(读取速率大于发送速率)
            qDebug()<<"USB控制器缓冲区为空，立即返回success，actualLength=0，不等待，不超时";
        }else if (result == LIBUSB_ERROR_TIMEOUT) {
            // 超时处理
            consecutiveTimeouts++;

            if (consecutiveTimeouts == 1) {
                qDebug() << "USB传输超时，这可能是正常的，下面将检测是否连续10次超时...";
            }

            if (consecutiveTimeouts >= maxConsecutiveTimeouts) {
                qDebug() << QString("连续超时 %1 次，检查设备状态").arg(consecutiveTimeouts);
            }

            continue;  // 超时是正常的，继续循环

        }else if (result == LIBUSB_ERROR_OVERFLOW){
            qDebug()<< "USB控制器缓冲区溢出，数据溢出，可能丢失数据,即将退出采集";

            m_stopRequested = true;
        }
        else if (result != LIBUSB_SUCCESS) {
            if (m_stopRequested)
                break;

            QString errorMsg = USBDebugHelper::analyzeTransferError(result);
            qDebug() << "USB传输错误:" << errorMsg;

            // 根据错误类型采取不同措施
            if (result == LIBUSB_ERROR_PIPE) {
                qDebug() << "尝试清除端点STALL状态";
                libusb_clear_halt(m_deviceHandle, m_endpoint);
                continue;
            } else if (result == LIBUSB_ERROR_NO_DEVICE) {
                emit errorOccurred("USB设备已断开连接");
                break;
            } else {
                emit errorOccurred(errorMsg);
                break;
            }
        }

//        qint64 currentTime2 = QDateTime::currentMSecsSinceEpoch();

//        qDebug()<< "接收一次数据的间隔时间(秒)："<< (currentTime2 - m_lastStatTime2) / 1000;

        // 短暂休眠以避免100%CPU使用
        msleep(SLEEPTIME);

//        m_lastStatTime2 = currentTime2;
    }

    qDebug() << "=== USB读取线程结束 ===";
}
