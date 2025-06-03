// USBDebugHelper.cpp - 实现文件
#include "USBDebugHelper.h"
#include <QThread>
#include "cyusb.h"

void USBDebugHelper::analyzeDevice(libusb_device* device)
{
    qDebug() << "=== USB设备详细分析 ===";

    struct libusb_device_descriptor desc;
    int result = libusb_get_device_descriptor(device, &desc);
    if (result < 0) {
        qDebug() << "无法获取设备描述符:" << libusb_strerror((libusb_error)result);
        return;
    }

    qDebug() << QString("VID:PID = %1:%2")
                .arg(desc.idVendor, 4, 16, QChar('0'))
                .arg(desc.idProduct, 4, 16, QChar('0'));
    qDebug() << "设备类别:" << desc.bDeviceClass;
    qDebug() << "配置数量:" << desc.bNumConfigurations;

    // 分析所有配置
    for (int config_idx = 0; config_idx < desc.bNumConfigurations; config_idx++) {
        struct libusb_config_descriptor* config;
        result = libusb_get_config_descriptor(device, config_idx, &config);
        if (result < 0) continue;

        qDebug() << QString("配置 %1:").arg(config_idx);
        qDebug() << "  接口数量:" << config->bNumInterfaces;

        // 分析所有接口
        for (int if_idx = 0; if_idx < config->bNumInterfaces; if_idx++) {
            const struct libusb_interface* interface = &config->interface[if_idx];
            qDebug() << QString("  接口 %1 (备用设置数: %2):").arg(if_idx).arg(interface->num_altsetting);

            // 分析所有备用设置
            for (int alt_idx = 0; alt_idx < interface->num_altsetting; alt_idx++) {
                const struct libusb_interface_descriptor* altsetting = &interface->altsetting[alt_idx];
                qDebug() << QString("    备用设置 %1:").arg(alt_idx);
                qDebug() << "      接口类别:" << altsetting->bInterfaceClass;
                qDebug() << "      端点数量:" << altsetting->bNumEndpoints;

                // 分析所有端点
                for (int ep_idx = 0; ep_idx < altsetting->bNumEndpoints; ep_idx++) {
                    const struct libusb_endpoint_descriptor* endpoint = &altsetting->endpoint[ep_idx];

                    QString direction = (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) ? "IN" : "OUT";
                    QString type = getTransferTypeString(endpoint->bmAttributes & 3);

                    qDebug() << QString("        端点 %1: 地址=0x%2, 方向=%3, 类型=%4, 最大包=%5")
                                .arg(ep_idx)
                                .arg(endpoint->bEndpointAddress, 2, 16, QChar('0'))
                                .arg(direction)
                                .arg(type)
                                .arg(endpoint->wMaxPacketSize);
                }
            }
        }

        libusb_free_config_descriptor(config);
    }

    qDebug() << "=== 分析完成 ===";
}

QList<EndpointInfo> USBDebugHelper::findAllEndpoints(libusb_device* device)
{
    QList<EndpointInfo> endpoints;

    struct libusb_config_descriptor* config;
    int result = libusb_get_active_config_descriptor(device, &config);
    if (result < 0) {
        qDebug() << "无法获取活动配置描述符";
        return endpoints;
    }

    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface* interface = &config->interface[i];
        for (int j = 0; j < interface->num_altsetting; j++) {
            const struct libusb_interface_descriptor* altsetting = &interface->altsetting[j];
            for (int k = 0; k < altsetting->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor* endpoint = &altsetting->endpoint[k];

                EndpointInfo info;
                info.address = endpoint->bEndpointAddress;
                info.type = endpoint->bmAttributes & 3;
                info.maxPacketSize = endpoint->wMaxPacketSize;
                info.direction = getDirectionString(endpoint->bEndpointAddress);
                info.typeString = getTransferTypeString(info.type);

                endpoints.append(info);
            }
        }
    }

    libusb_free_config_descriptor(config);
    return endpoints;
}

bool USBDebugHelper::testEndpoint(libusb_device_handle* handle, uint8_t endpoint, int timeout)
{
    qDebug() << QString("测试端点 0x%1...").arg(endpoint, 2, 16, QChar('0'));

    unsigned char buffer[16*1024];
    int actual_length;

    int result = libusb_bulk_transfer(handle, endpoint, buffer, sizeof(buffer), &actual_length, timeout);

//    int result = cyusb_bulk_transfer(handle,  endpoint, buffer,sizeof(buffer),&actual_length, timeout);

    if (result == LIBUSB_SUCCESS) {
        qDebug() << QString("端点 0x%1 成功接收 %2 字节数据")
                    .arg(endpoint, 2, 16, QChar('0'))
                    .arg(actual_length);

        // 显示前16字节数据
        QString hexData;
        for (int i = 0; i < qMin(16, actual_length); i++) {
            hexData += QString("%1 ").arg(buffer[i], 2, 16, QChar('0'));
        }
        qDebug() << "数据内容:" << hexData;

        return true;
    } else if (result == LIBUSB_ERROR_TIMEOUT) {
        qDebug() << QString("端点 0x%1 超时 (可能无数据)").arg(endpoint, 2, 16, QChar('0'));
        return false;
    } else {
        qDebug() << QString("端点 0x%1 错误: %2")
                    .arg(endpoint, 2, 16, QChar('0'))
                    .arg(libusb_strerror((libusb_error)result));
        return false;
    }
}

uint8_t USBDebugHelper::findWorkingEndpoint(libusb_device_handle* handle)
{
    qDebug() << "尝试查找工作的端点...";

    // 常见的输入端点地址
    uint8_t commonEndpoints[] = {/*0x81, 0x82, 0x83, 0x84, 0x85,*/ 0x86/*, 0x87, 0x88*/};

    for (uint8_t endpoint : commonEndpoints) {
        if (testEndpoint(handle, endpoint, 1000)) {  // 500ms超时快速测试
            qDebug() << QString("找到工作端点: 0x%1").arg(endpoint, 2, 16, QChar('0'));
            return endpoint;
        }
    }

    qDebug() << "未找到工作的端点";
    return 0;
}

void USBDebugHelper::checkDeviceStatus(libusb_device_handle* handle)
{
    qDebug() << "=== 检查设备状态 ===";

    // 检查设备配置
    int config;
    int result = libusb_get_configuration(handle, &config);
    if (result == LIBUSB_SUCCESS) {
        qDebug() << "当前配置:" << config;
    } else {
        qDebug() << "无法获取配置:" << libusb_strerror((libusb_error)result);
    }

    // 检查接口状态
    for (int interface = 0; interface < 4; interface++) {
        if (libusb_kernel_driver_active(handle, interface) == 1) {
            qDebug() << QString("接口 %1: 内核驱动已激活").arg(interface);
        } else {
            qDebug() << QString("接口 %1: 内核驱动未激活").arg(interface);
        }
    }

    qDebug() << "=== 状态检查完成 ===";
}

bool USBDebugHelper::tryActivateDevice(libusb_device_handle* handle)
{
    qDebug() << "尝试激活设备...";

    // 尝试一些常见的激活命令
    struct {
        uint8_t bmRequestType;
        uint8_t bRequest;
        uint16_t wValue;
        uint16_t wIndex;
        const char* description;
    } activationCommands[] = {
        {LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT, 0x01, 0x0001, 0x0000, "厂商命令1"},
        {LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT, 0x09, 0x0200, 0x0000, "HID设置报告"},
        {LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT, LIBUSB_REQUEST_SET_FEATURE, 0x0001, 0x0000, "设置特性"},
    };

    for (const auto& cmd : activationCommands) {
        qDebug() << "尝试:" << cmd.description;

        int result = libusb_control_transfer(handle,
                                           cmd.bmRequestType,
                                           cmd.bRequest,
                                           cmd.wValue,
                                           cmd.wIndex,
                                           nullptr, 0, 1000);

        if (result >= 0) {
            qDebug() << "命令执行成功";
            // 等待一下让设备响应
            QThread::msleep(100);
            return true;
        } else {
            qDebug() << "命令失败:" << libusb_strerror((libusb_error)result);
        }
    }

    return false;
}

int USBDebugHelper::getEndpointMaxPacketSize(libusb_device* device, uint8_t endpoint)
{
    QList<EndpointInfo> endpoints = findAllEndpoints(device);

    for (const auto& ep : endpoints) {
        if (ep.address == endpoint) {
            return ep.maxPacketSize;
        }
    }

    return 64;  // 默认值
}

QString USBDebugHelper::analyzeTransferError(int result)
{
    switch (result) {
        case LIBUSB_SUCCESS:
            return "传输成功";
        case LIBUSB_ERROR_TIMEOUT:
            return "传输超时 - 设备可能未发送数据或端点地址错误";
        case LIBUSB_ERROR_PIPE:
            return "管道错误 - 端点可能处于STALL状态";
        case LIBUSB_ERROR_NO_DEVICE:
            return "设备已断开连接";
        case LIBUSB_ERROR_BUSY:
            return "设备忙 - 资源被占用";
        case LIBUSB_ERROR_INVALID_PARAM:
            return "参数无效 - 检查端点地址和缓冲区";
        case LIBUSB_ERROR_ACCESS:
            return "权限不足 - 检查设备权限";
        case LIBUSB_ERROR_OVERFLOW:
            return "数据溢出 - 缓冲区太小";
        default:
            return QString("未知错误: %1").arg(libusb_strerror((libusb_error)result));
    }
}

QString USBDebugHelper::getTransferTypeString(uint8_t type)
{
    switch (type) {
        case LIBUSB_TRANSFER_TYPE_CONTROL: return "控制";
        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS: return "同步";
        case LIBUSB_TRANSFER_TYPE_BULK: return "批量";
        case LIBUSB_TRANSFER_TYPE_INTERRUPT: return "中断";
        default: return "未知";
    }
}

QString USBDebugHelper::getDirectionString(uint8_t address)
{
    return (address & LIBUSB_ENDPOINT_IN) ? "IN" : "OUT";
}

