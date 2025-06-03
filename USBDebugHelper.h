// USBDebugHelper.h - USB调试辅助类
#ifndef USBDEBUGHELPER_H
#define USBDEBUGHELPER_H

#include <libusb-1.0/libusb.h>
#include <QDebug>
#include <QString>
#include <QList>

struct EndpointInfo {
    uint8_t address;
    uint8_t type;
    uint16_t maxPacketSize;
    QString direction;
    QString typeString;
};

class USBDebugHelper
{
public:
    // 分析USB设备的详细信息
    static void analyzeDevice(libusb_device* device);

    // 查找所有可用端点
    static QList<EndpointInfo> findAllEndpoints(libusb_device* device);

    // 测试不同端点是否有数据
    static bool testEndpoint(libusb_device_handle* handle, uint8_t endpoint, int timeout = 1000);

    // 尝试不同的端点地址
    static uint8_t findWorkingEndpoint(libusb_device_handle* handle);

    // 检查设备状态
    static void checkDeviceStatus(libusb_device_handle* handle);

    // 发送激活命令的通用尝试
    static bool tryActivateDevice(libusb_device_handle* handle);

    // 获取端点最大包大小
    static int getEndpointMaxPacketSize(libusb_device* device, uint8_t endpoint);

    // 详细的传输错误分析
    static QString analyzeTransferError(int result);

private:
    static QString getTransferTypeString(uint8_t type);
    static QString getDirectionString(uint8_t address);
};

#endif
