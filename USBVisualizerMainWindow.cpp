// USBVisualizerMainWindow.cpp
#include "USBVisualizerMainWindow.h"
#include <QApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSplitter>
#include <QVBoxLayout>
#include <algorithm>
#include <cstring>
#include <QElapsedTimer>

/*`m_maxBufferSize` 是绘图缓冲区的大小，它控制：
- 图表上最多显示多少个数据点
- 当数据点超过这个数量时，会删除最旧的数据点（FIFO队列）
- 默认值是2000，意味着图表最多同时显示2000个采样点
- 可以通过UI中的"缓冲区大小"调整（10-20000范围）
*/

#define YDefaultValue 4096     //Y轴默认值
#define MaxBufferSizeValue 50000 //绘图缓冲区最大大小

#define GROUPHEIGHT 140

// 主窗口实现
USBVisualizerMainWindow::USBVisualizerMainWindow(QWidget* parent)
    : QMainWindow(parent),
      m_centralWidget(nullptr),
      m_plotGroup(nullptr),
      m_customPlot(nullptr),
      m_dataGraph(nullptr),
      m_usbContext(nullptr),
      m_readerThread(nullptr),
      m_sampleCounter(0),
      m_maxBufferSize(4000),
      m_deviceConnected(false),
      m_dataCollection(false),
      m_currentDataRate(0),
      m_currentSampleRate(0),
      m_triangleDetector(nullptr)
{
    setWindowTitle("USB数据实时可视化系统 - 三角波检测版本");
    setMinimumSize(1200, 800);
    resize(1600, 1000);  // 增加宽度以容纳更多状态信息

    // 初始化libusb
    int result = libusb_init(&m_usbContext);
    if (result < 0) {
        QMessageBox::critical(this, "错误", QString("无法初始化libusb: %1").arg(libusb_strerror((libusb_error)result)));
        QApplication::quit();
        return;
    }

    // 按正确顺序初始化
    setupUI();
    setupPlot();
    refreshDevices();

    //设置默认参数
    setupTriangleDetector();

    // 设置绘图更新定时器
    m_plotUpdateTimer = new QTimer(this);
    connect(m_plotUpdateTimer, &QTimer::timeout, this, &USBVisualizerMainWindow::updatePlot);
//    m_plotUpdateTimer->start(10);  // 10FPS更新率

    qDebug() << "主窗口初始化完成";
}

USBVisualizerMainWindow::~USBVisualizerMainWindow()
{
    // 停止记录线程
    if (m_readerThread) {
        m_readerThread->stopReading();
        delete m_readerThread;
    }

    // 释放设备引用
    for (const auto& deviceInfo : m_deviceList) {
        if (deviceInfo.device) {
            libusb_unref_device(deviceInfo.device);
        }
    }

    if (m_usbContext) {
        libusb_exit(m_usbContext);
    }
}

// 修改后的 setupUI() 函数
void USBVisualizerMainWindow::setupUI()
{
    m_centralWidget = new QWidget(this);
    setCentralWidget(m_centralWidget);

    // 主布局
    m_mainLayout = new QVBoxLayout(m_centralWidget);
    m_mainLayout->setSpacing(10);
    m_mainLayout->setContentsMargins(10, 10, 10, 10);

    // 上部控制面板 - 增加高度以容纳更多状态信息
    QWidget* controlPanel = new QWidget();
    controlPanel->setFixedHeight(170);  // 增加高度
    controlPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_mainLayout->addWidget(controlPanel);

    // 控制面板的网格布局
    QGridLayout* controlGridLayout = new QGridLayout(controlPanel);
    controlGridLayout->setSpacing(8);

    // 第一行：设备控制
    m_deviceGroup = new QGroupBox("设备控制", this);
    m_deviceGroup->setFixedHeight(GROUPHEIGHT);
    controlGridLayout->addWidget(m_deviceGroup, 0, 0);

    QVBoxLayout* deviceLayout = new QVBoxLayout(m_deviceGroup);

    // 设备选择行
    QHBoxLayout* deviceSelectLayout = new QHBoxLayout();
    deviceSelectLayout->addWidget(new QLabel("USB设备:"));

    m_deviceCombo = new QComboBox(this);
    m_deviceCombo->setMinimumWidth(220);
    deviceSelectLayout->addWidget(m_deviceCombo, 1);

    m_refreshBtn = new QPushButton("刷新", this);
    m_refreshBtn->setMaximumWidth(60);
    deviceSelectLayout->addWidget(m_refreshBtn);

    deviceLayout->addLayout(deviceSelectLayout);

    // 连接控制行
    QHBoxLayout* connectLayout = new QHBoxLayout();
    m_connectBtn = new QPushButton("连接", this);
    m_disconnectBtn = new QPushButton("断开", this);
    m_disconnectBtn->setEnabled(false);

    connectLayout->addWidget(m_connectBtn);
    connectLayout->addWidget(m_disconnectBtn);
    connectLayout->addStretch();

    deviceLayout->addLayout(connectLayout);

    // 数据采集控制
    m_dataGroup = new QGroupBox("数据采集", this);
    m_dataGroup->setFixedHeight(GROUPHEIGHT + 30);
    controlGridLayout->addWidget(m_dataGroup, 0, 1);

    QVBoxLayout* dataLayout = new QVBoxLayout(m_dataGroup);

    QHBoxLayout* dataControlLayout = new QHBoxLayout();
    m_startBtn = new QPushButton("开始采集", this);
    m_stopBtn = new QPushButton("停止采集", this);
    m_clearBtn = new QPushButton("清空数据", this);

    m_startBtn->setEnabled(false);
    m_stopBtn->setEnabled(false);

    dataControlLayout->addWidget(m_startBtn);
    dataControlLayout->addWidget(m_stopBtn);
    dataLayout->addLayout(dataControlLayout);

    QHBoxLayout* clearLayout = new QHBoxLayout();
    clearLayout->addWidget(m_clearBtn);
    clearLayout->addStretch();
    dataLayout->addLayout(clearLayout);

    // 图表配置
    m_configGroup = new QGroupBox("图表配置", this);
    m_configGroup->setFixedHeight(GROUPHEIGHT+30);
    controlGridLayout->addWidget(m_configGroup, 0, 2);

    QVBoxLayout* configLayout = new QVBoxLayout(m_configGroup);

    // Y轴范围配置
    QHBoxLayout* yRangeLayout = new QHBoxLayout();
    yRangeLayout->addWidget(new QLabel("Y轴最小值:"));
    m_yMinSpin = new QSpinBox(this);
    m_yMinSpin->setRange(-32768, 65535);
    m_yMinSpin->setValue(0);
    m_yMinSpin->setMaximumWidth(80);
    yRangeLayout->addWidget(m_yMinSpin);

    yRangeLayout->addWidget(new QLabel("最大值:"));
    m_yMaxSpin = new QSpinBox(this);
    m_yMaxSpin->setRange(0, 65535);
    m_yMaxSpin->setValue(4096);
    m_yMaxSpin->setMaximumWidth(80);
    yRangeLayout->addWidget(m_yMaxSpin);

    configLayout->addLayout(yRangeLayout);

    // 自动缩放和缓冲区配置
    QHBoxLayout* optionsLayout = new QHBoxLayout();
    m_autoScaleCheck = new QCheckBox("自动缩放", this);
    m_autoScaleCheck->setChecked(false);
    optionsLayout->addWidget(m_autoScaleCheck);

    optionsLayout->addWidget(new QLabel("缓冲区大小:"));
    m_bufferSizeSpin = new QSpinBox(this);
    m_bufferSizeSpin->setRange(10, MaxBufferSizeValue);
    m_bufferSizeSpin->setValue(m_maxBufferSize);
    m_bufferSizeSpin->setMaximumWidth(80);
    optionsLayout->addWidget(m_bufferSizeSpin);

    configLayout->addLayout(optionsLayout);

    // 应用配置按钮
    QPushButton* applyConfigBtn = new QPushButton("应用配置", this);
    configLayout->addWidget(applyConfigBtn);

    // 系统状态信息
    QGroupBox* statusGroup = new QGroupBox("系统状态", this);
    statusGroup->setFixedHeight(GROUPHEIGHT);
    controlGridLayout->addWidget(statusGroup, 0, 3);

    QVBoxLayout* statusLayout = new QVBoxLayout(statusGroup);

    m_statusLabel = new QLabel("状态: 未连接");
    m_dataRateLabel = new QLabel("数据率: 0 KB/s");
    m_sampleCountLabel = new QLabel("采样数: 0");
    m_bufferUsageLabel = new QLabel("缓冲区: 0%");

    // 三角波检测状态信息组 - 主要状态
    QGroupBox* triangleMainGroup = new QGroupBox("三角波状态", this);
    triangleMainGroup->setFixedHeight(GROUPHEIGHT);
    controlGridLayout->addWidget(triangleMainGroup, 0, 4);

    QVBoxLayout* triangleMainLayout = new QVBoxLayout(triangleMainGroup);

    m_triangleStatusLabel = new QLabel();
    m_triangleStatusLabel->setWordWrap(true);
    m_triangleStatusLabel->setFixedWidth(140);
    m_learningProgressLabel = new QLabel("学习进度: 等待数据...");
    m_learningProgressLabel->setWordWrap(true);

    triangleMainLayout->addWidget(m_learningProgressLabel);    
    triangleMainLayout->addWidget(m_triangleStatusLabel);
    triangleMainLayout->addStretch();

    // 三角波详细状态信息组 - 新增
    QGroupBox* triangleDetailGroup = new QGroupBox("检测详情", this);
    triangleDetailGroup->setFixedHeight(GROUPHEIGHT);
    triangleDetailGroup->setFixedWidth(120);
    controlGridLayout->addWidget(triangleDetailGroup, 0, 5);

    QVBoxLayout* triangleDetailLayout = new QVBoxLayout(triangleDetailGroup);

    m_initializationStatusLabel = new QLabel("初始化: 等待稳定...");
    m_initializationStatusLabel->setWordWrap(true);

    m_cycleValidityLabel = new QLabel("周期质量: 待评估");
    m_cycleValidityLabel->setWordWrap(true);

    m_detectionQualityLabel = new QLabel("检测质量: 初始化中");
    m_detectionQualityLabel->setWordWrap(true);

    triangleDetailLayout->addWidget(m_initializationStatusLabel);
    triangleDetailLayout->addWidget(m_cycleValidityLabel);
    triangleDetailLayout->addWidget(m_detectionQualityLabel);
    triangleDetailLayout->addStretch();

    // 设置状态标签样式
    QFont statusFont;
    statusFont.setPointSize(8);
    m_statusLabel->setFont(statusFont);
    m_dataRateLabel->setFont(statusFont);
    m_sampleCountLabel->setFont(statusFont);
    m_bufferUsageLabel->setFont(statusFont);
    m_triangleStatusLabel->setFont(statusFont);
    m_learningProgressLabel->setFont(statusFont);
    m_initializationStatusLabel->setFont(statusFont);
    m_cycleValidityLabel->setFont(statusFont);
    m_detectionQualityLabel->setFont(statusFont);

    statusLayout->addWidget(m_statusLabel);
    statusLayout->addWidget(m_dataRateLabel);
    statusLayout->addWidget(m_sampleCountLabel);
    statusLayout->addWidget(m_bufferUsageLabel);
    statusLayout->addStretch();

    // 图表区域
    m_plotGroup = new QGroupBox("实时数据图表", this);
    m_plotGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_mainLayout->addWidget(m_plotGroup, 1);

    // 图表容器布局
    QVBoxLayout* plotContainerLayout = new QVBoxLayout(m_plotGroup);
    plotContainerLayout->setContentsMargins(5, 5, 5, 5);

    // 创建占位符，稍后在setupPlot()中替换
    QLabel* plotPlaceholder = new QLabel("图表正在初始化...", this);
    plotPlaceholder->setAlignment(Qt::AlignCenter);
    plotPlaceholder->setMinimumHeight(400);
    plotPlaceholder->setStyleSheet("QLabel { border: 1px dashed #666; color: #888; font-size: 16px; }");
    plotContainerLayout->addWidget(plotPlaceholder);

    // 数据抽取配置 - 新增
    QHBoxLayout* decimationLayout = new QHBoxLayout();
    decimationLayout->addWidget(new QLabel("数据抽取:"));

    m_dataDecimationSpin = new QSpinBox(this);
    m_dataDecimationSpin->setRange(1, 100000);
    m_dataDecimationSpin->setValue(1);  // 默认不抽取
    m_dataDecimationSpin->setMaximumWidth(50);
    m_dataDecimationSpin->setToolTip("连续N个点的平均值作为一个点显示(最大值100000)");
    decimationLayout->addWidget(m_dataDecimationSpin);

    m_applyDecimationBtn = new QPushButton("应用", this);
    m_applyDecimationBtn->setMaximumWidth(60);
    decimationLayout->addWidget(m_applyDecimationBtn);

    decimationLayout->addWidget(new QLabel("点差值"));
    //当前点和上个点的差值
    m_deltaValueSpin = new QSpinBox(this);
    m_deltaValueSpin->setRange(1, 100000);
    m_deltaValueSpin->setValue(3);  // 默认不做差值
    m_deltaValueSpin->setMaximumWidth(50);
    decimationLayout->addWidget(m_deltaValueSpin);

    m_applyDeltaBtn = new QPushButton("应用", this);
    m_applyDeltaBtn->setMaximumWidth(60);
    decimationLayout->addWidget(m_applyDeltaBtn);

    decimationLayout->addStretch();
    configLayout->addLayout(decimationLayout);

    QHBoxLayout* sleepLayout = new QHBoxLayout();
    sleepLayout->addWidget(new QLabel("线程睡眠:"));

    m_sleepTimeSpin = new QSpinBox(this);
    m_sleepTimeSpin->setRange(10, 100000);     // 10ms到5000ms
    m_sleepTimeSpin->setValue(1000);         // 默认1000ms
    m_sleepTimeSpin->setToolTip("最大值100000");
    m_sleepTimeSpin->setSuffix(" ms");
    m_sleepTimeSpin->setMaximumWidth(100);
    m_sleepTimeSpin->setToolTip("USB接收线程睡眠时间\n数值越小接收速度越快");
    sleepLayout->addWidget(m_sleepTimeSpin);

    m_applyThreadConfigBtn = new QPushButton("应用", this);
    m_applyThreadConfigBtn->setMaximumWidth(60);
    sleepLayout->addWidget(m_applyThreadConfigBtn);

    sleepLayout->addStretch();
    configLayout->addLayout(sleepLayout);

    m_debugCheckBox = new QCheckBox("调试打印", this);
    m_debugCheckBox->setChecked(false);  // 默认关闭
    m_debugCheckBox->setToolTip("开启/关闭控制台调试信息输出");
    sleepLayout->addWidget(m_debugCheckBox);
    sleepLayout->addStretch();

    QHBoxLayout* bufferReadLayout = new QHBoxLayout();
    bufferReadLayout->addWidget(new QLabel("读取数据大小(字节):"));

    m_bufferReadSpin = new QSpinBox(this);
    m_bufferReadSpin->setRange(1024, 1024*1024*10);
    m_bufferReadSpin->setValue(40960);
    m_bufferReadSpin->setToolTip("最大值1020*1024*10");
    m_bufferReadSpin->setMaximumWidth(100);
    bufferReadLayout->addWidget(m_bufferReadSpin);

    m_applyBufferReadBtn = new QPushButton("应用", this);
    m_applyBufferReadBtn->setMaximumWidth(60);
    bufferReadLayout->addWidget(m_applyBufferReadBtn);

    bufferReadLayout->addStretch();
    dataLayout->addLayout(bufferReadLayout);


    QHBoxLayout* timerLayout = new QHBoxLayout();
    timerLayout->addWidget(new QLabel("渲染定时器间隔(ms):"));

    m_timerPaintEnginSpin = new QSpinBox(this);
    m_timerPaintEnginSpin->setRange(1, 10000);
    m_timerPaintEnginSpin->setValue(10);
    m_timerPaintEnginSpin->setMaximumWidth(100);
    timerLayout->addWidget(m_timerPaintEnginSpin);

    m_applytimerPaintEnginBtn = new QPushButton("应用", this);
    m_applytimerPaintEnginBtn->setMaximumWidth(60);
    timerLayout->addWidget(m_applytimerPaintEnginBtn);

    timerLayout->addStretch();
    dataLayout->addLayout(timerLayout);

    // 连接信号槽
    connect(m_refreshBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::refreshDevices);
    connect(m_connectBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::connectDevice);
    connect(m_disconnectBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::disconnectDevice);
    connect(m_startBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::startDataCollection);
    connect(m_stopBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::stopDataCollection);
    connect(m_clearBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::clearData);
    connect(applyConfigBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::updatePlotSettings);
    connect(m_applyDecimationBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::updateDecimationSettings);
    connect(m_applyThreadConfigBtn, &QPushButton::clicked, this, &USBVisualizerMainWindow::applySleepTimeConfig);
    connect(m_debugCheckBox, &QCheckBox::toggled, this, &USBVisualizerMainWindow::onDebugPrintToggled);

    connect(m_applyDeltaBtn, &QPushButton::clicked, [this](){m_deltaValue = m_deltaValueSpin->value();});

    connect(m_applyBufferReadBtn, &QPushButton::clicked, [this](){
        m_bufferReadValue = m_bufferReadSpin->value();
        // 检查是否有USB线程在运行
        if (!m_readerThread || !m_deviceConnected) {
            QMessageBox::warning(this, "警告", "请先连接USB设备");
            return;
        }
        m_readerThread->setChunkSize(m_bufferReadValue);
    });

    connect(m_applytimerPaintEnginBtn, &QPushButton::clicked, [this](){
        m_timerPaintEnginValue = m_timerPaintEnginSpin->value();
    });


    qDebug() << "UI界面创建完成";

}

void USBVisualizerMainWindow::setupPlot()
{
    // 获取图表组的布局
    QVBoxLayout* plotLayout = qobject_cast<QVBoxLayout*>(m_plotGroup->layout());
    if (!plotLayout) {
        qDebug() << "错误：无法获取图表组布局";
        return;
    }

    // 移除占位符
    QLayoutItem* item;
    while ((item = plotLayout->takeAt(0)) != nullptr) {
        if (item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

    // 创建QCustomPlot
    m_customPlot = new QCustomPlot(this);
    m_customPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_customPlot->setMinimumHeight(400);

    // 设置深色主题
    m_customPlot->setBackground(QBrush(QColor(25, 25, 25)));
    m_customPlot->axisRect()->setBackground(QBrush(QColor(35, 35, 35)));

    // 设置坐标轴样式
    QPen axisPen(Qt::white, 1);
    m_customPlot->xAxis->setBasePen(axisPen);
    m_customPlot->yAxis->setBasePen(axisPen);
    m_customPlot->xAxis->setTickPen(axisPen);
    m_customPlot->yAxis->setTickPen(axisPen);
    m_customPlot->xAxis->setSubTickPen(QPen(Qt::gray, 1));
    m_customPlot->yAxis->setSubTickPen(QPen(Qt::gray, 1));
    m_customPlot->xAxis->setTickLabelColor(Qt::white);
    m_customPlot->yAxis->setTickLabelColor(Qt::white);
    m_customPlot->xAxis->setLabelColor(Qt::white);
    m_customPlot->yAxis->setLabelColor(Qt::white);

    // 设置标签
    m_customPlot->xAxis->setLabel("时间 (样本数)");
    m_customPlot->yAxis->setLabel("数值 (0-65535)");

    // 设置网格
    QPen gridPen(QColor(80, 80, 80), 1, Qt::SolidLine);
    QPen subGridPen(QColor(50, 50, 50), 1, Qt::DotLine);

    m_customPlot->xAxis->grid()->setPen(gridPen);
    m_customPlot->yAxis->grid()->setPen(gridPen);
    m_customPlot->xAxis->grid()->setSubGridPen(subGridPen);
    m_customPlot->yAxis->grid()->setSubGridPen(subGridPen);
    m_customPlot->xAxis->grid()->setSubGridVisible(true);
    m_customPlot->yAxis->grid()->setSubGridVisible(true);

    // 创建数据图形 - 绿色线条
    m_dataGraph = m_customPlot->addGraph();
    QPen graphPen(QColor(0, 255, 0), 2);  // 绿色，2像素宽
    m_dataGraph->setPen(graphPen);
    m_dataGraph->setName("USB数据");

    // 设置初始范围
    m_customPlot->xAxis->setRange(0, m_maxBufferSize);
    m_customPlot->yAxis->setRange(0, 65535);

    // 启用交互
    m_customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    // 添加图例
    m_customPlot->legend->setVisible(true);
    m_customPlot->legend->setBrush(QBrush(QColor(50, 50, 50, 200)));
    m_customPlot->legend->setBorderPen(QPen(Qt::white));
    m_customPlot->legend->setTextColor(Qt::white);
    m_customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignRight);

    // 将图表添加到布局
    plotLayout->addWidget(m_customPlot);

    // 确保图表能够正确重绘
    m_customPlot->replot();

    qDebug() << "图表创建完成";
}


void USBVisualizerMainWindow::applySleepTimeConfig()
{
    int newSleepTime = m_sleepTimeSpin->value();

    // 检查是否有USB线程在运行
    if (!m_readerThread || !m_deviceConnected) {
        QMessageBox::warning(this, "警告", "请先连接USB设备");
        return;
    }

    // 应用新的睡眠时间
    m_readerThread->setSleepFactor(newSleepTime);

    qDebug() << QString("USB线程睡眠时间已设置为: %1ms").arg(newSleepTime);

}

void USBVisualizerMainWindow::onDebugPrintToggled(bool enabled)
{
    m_isPrintEnabled = enabled;
}

void USBVisualizerMainWindow::refreshDevices()
{
    qDebug() << "开始刷新USB设备列表";

    // 清理旧设备列表
    for (const auto& deviceInfo : m_deviceList) {
        if (deviceInfo.device) {
            libusb_unref_device(deviceInfo.device);
        }
    }
    m_deviceList.clear();

    libusb_device** deviceList;
    ssize_t deviceCount = libusb_get_device_list(m_usbContext, &deviceList);

    if (deviceCount < 0) {
        QMessageBox::warning(this, "警告", "无法获取USB设备列表");
        return;
    }

    qDebug() << "发现" << deviceCount << "个USB设备";

    for (ssize_t i = 0; i < deviceCount; i++) {
        libusb_device* device = deviceList[i];
        libusb_device_descriptor desc;

        if (libusb_get_device_descriptor(device, &desc) == LIBUSB_SUCCESS) {
            USBDeviceInfo deviceInfo;
            deviceInfo.device = device;
            deviceInfo.vendorId = desc.idVendor;
            deviceInfo.productId = desc.idProduct;

            // 尝试获取字符串描述符
            libusb_device_handle* handle;
            if (libusb_open(device, &handle) == LIBUSB_SUCCESS) {
                unsigned char buffer[256];

                if (desc.iManufacturer > 0 && libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, buffer, sizeof(buffer)) >= 0) {
                    deviceInfo.manufacturer = QString::fromUtf8(reinterpret_cast<char*>(buffer));
                } else {
                    deviceInfo.manufacturer = "Unknown";
                }

                if (desc.iProduct > 0 && libusb_get_string_descriptor_ascii(handle, desc.iProduct, buffer, sizeof(buffer)) >= 0) {
                    deviceInfo.product = QString::fromUtf8(reinterpret_cast<char*>(buffer));
                } else {
                    deviceInfo.product = "Unknown";
                }

                libusb_close(handle);
            } else {
                deviceInfo.manufacturer = "Unknown";
                deviceInfo.product = "Unknown";
            }

            deviceInfo.description = QString("%1 - %2 (%3:%4)")
                                         .arg(deviceInfo.manufacturer)
                                         .arg(deviceInfo.product)
                                         .arg(deviceInfo.vendorId, 4, 16, QChar('0'))
                                         .arg(deviceInfo.productId, 4, 16, QChar('0'));

            // 增加引用计数以保持设备有效
            libusb_ref_device(device);
            m_deviceList.append(deviceInfo);
        }
    }

    libusb_free_device_list(deviceList, 1);
    updateDeviceCombo();

    qDebug() << "设备列表刷新完成，共" << m_deviceList.size() << "个设备";
}

void USBVisualizerMainWindow::updateDeviceCombo()
{
    m_deviceCombo->clear();

    for (const auto& deviceInfo : m_deviceList) {
        m_deviceCombo->addItem(deviceInfo.description);
    }

    if (m_deviceList.isEmpty()) {
        m_deviceCombo->addItem("未发现USB设备");
    }
}

void USBVisualizerMainWindow::connectDevice()
{
    int selectedIndex = m_deviceCombo->currentIndex();
    if (selectedIndex < 0 || selectedIndex >= m_deviceList.size()) {
        QMessageBox::warning(this, "警告", "请选择一个USB设备");
        return;
    }

    const USBDeviceInfo& deviceInfo = m_deviceList[selectedIndex];

    qDebug() << "尝试连接设备:" << deviceInfo.description;

    // 创建USB读取线程
    m_readerThread = new USBReaderThread(this);

    connect(m_readerThread, &USBReaderThread::dataReceived, this, &USBVisualizerMainWindow::onDataReceived);
    connect(m_readerThread, &USBReaderThread::errorOccurred, this, &USBVisualizerMainWindow::onUSBError);
    //实时显示传输速率等信息
    connect(m_readerThread, &USBReaderThread::statisticsUpdated, this, &USBVisualizerMainWindow::onStatisticsUpdated);

    // 设置初始抽取倍数
    if (m_readerThread->setDevice(deviceInfo.device)) {
        m_deviceConnected = true;
        m_currentDeviceInfo = deviceInfo.description;

        m_connectBtn->setEnabled(false);
        m_disconnectBtn->setEnabled(true);
        m_startBtn->setEnabled(true);

        m_statusLabel->setText("状态: 已连接 - " + deviceInfo.description);


        m_decimationFactor = m_dataDecimationSpin->value();
        m_deltaValue = m_deltaValueSpin->value();
        m_bufferReadValue = m_bufferReadSpin->value();
        m_timerPaintEnginValue = m_timerPaintEnginSpin->value();
        // 设置初始抽取倍数
//        m_readerThread->setDecimationFactor(m_decimationFactor);//暂时弃用
        m_readerThread->setChunkSize(m_bufferReadValue);
        m_readerThread->setSleepFactor(m_sleepTimeSpin->value());

        // 【新增】重置三角波检测相关状态和颜色
        QString waitingText = "学习进度: 等待数据...";
        m_learningProgressLabel->setText(waitingText);
        updateLearningProgressColor(waitingText);

        m_triangleStatusLabel->setText("");
        updateTriangleStatusColor("");

        QString initText = "初始化: 等待稳定...";
        m_initializationStatusLabel->setText(initText);
        updateInitializationStatusColor(initText);

        QString cycleText = "周期质量: 待评估";
        m_cycleValidityLabel->setText(cycleText);
        updateCycleValidityColor(cycleText);

        QString detectionText = "检测质量: 初始化中";
        m_detectionQualityLabel->setText(detectionText);
        updateDetectionQualityColor(detectionText);

        QMessageBox::information(this, "成功", "USB设备连接成功");
        qDebug() << "设备连接成功";
    } else {
        delete m_readerThread;
        m_readerThread = nullptr;
        QMessageBox::critical(this, "错误", "无法连接USB设备");
        qDebug() << "设备连接失败";
    }
}

void USBVisualizerMainWindow::disconnectDevice()
{
    qDebug() << "断开USB设备连接";

    stopDataCollection();

    if (m_readerThread) {
        m_readerThread->stopReading();
        delete m_readerThread;
        m_readerThread = nullptr;
    }

    m_deviceConnected = false;
    m_connectBtn->setEnabled(true);
    m_disconnectBtn->setEnabled(false);
    m_startBtn->setEnabled(false);

    m_statusLabel->setText("状态: 未连接");
    m_triangleStatusLabel->setText("");
    m_dataRateLabel->setText("数据率: 0 KB/s");

    // 【新增】重置所有状态颜色
    resetAllStatusColors();

    // 重置状态文本
    m_learningProgressLabel->setText("学习进度: 等待数据...");
    m_triangleStatusLabel->setText("");
    m_initializationStatusLabel->setText("初始化: 等待稳定...");
    m_cycleValidityLabel->setText("周期质量: 待评估");
    m_detectionQualityLabel->setText("检测质量: 初始化中");

    QMessageBox::information(this, "成功", "USB设备已断开");
}

void USBVisualizerMainWindow::startDataCollection()
{
    if (!m_readerThread || !m_deviceConnected) {
        QMessageBox::warning(this, "警告", "请先连接USB设备");
        return;
    }

    if (m_dataCollection) {
        return;
    }
    qDebug() << "开始数据采集";

    m_dataCollection = true;
    m_startBtn->setEnabled(false);
    m_stopBtn->setEnabled(true);

    // 清空现有数据
    clearData();

    // 启动USB读取线程
    m_readerThread->start();

    m_statusLabel->setText("状态: 正在采集数据 - " + m_currentDeviceInfo);

    // 【新增】更新检测状态
    QString detectionText = "检测质量: 准备中...";
    m_detectionQualityLabel->setText(detectionText);
    updateDetectionQualityColor(detectionText);
}

void USBVisualizerMainWindow::stopDataCollection()
{
    if (!m_dataCollection) {
        return;
    }

    qDebug() << "停止数据采集";

    m_dataCollection = false;
    m_startBtn->setEnabled(m_deviceConnected);
    m_stopBtn->setEnabled(false);

    if (m_plotUpdateTimer->isActive())
    {
        m_plotUpdateTimer->stop();
    }

    if (m_readerThread) {
        m_readerThread->stopReading();
    }

    if (m_deviceConnected) {
        m_statusLabel->setText("状态: 已连接 - " + m_currentDeviceInfo);
    }
}

void USBVisualizerMainWindow::clearData()
{
    qDebug() << "清空数据缓冲区";

    QMutexLocker locker(&m_dataMutex);

    m_dataBuffer.clear();
    m_sampleCounter = 0;

    // 重置环形缓冲区
    m_ringBufferWritePos = 0;
    m_ringBufferFull = false;

    // 重新初始化绘图缓冲区
    if (m_plotDataX.size() == m_maxBufferSize && m_plotDataY.size() == m_maxBufferSize) {
        // 只清空Y数据，X坐标保持不变
        std::fill(m_plotDataY.begin(), m_plotDataY.end(), 0.0);
    } else {
        // 大小不匹配，重新初始化
        m_plotDataX.clear();
        m_plotDataY.clear();
        m_plotDataX.resize(m_maxBufferSize);
        m_plotDataY.resize(m_maxBufferSize);

        for (int i = 0; i < m_maxBufferSize; i++) {
            m_plotDataX[i] = static_cast<double>(i);
            m_plotDataY[i] = 0.0;
        }
    }

    // 清空图表
    if (m_dataGraph) {
        m_dataGraph->data()->clear();
    }
    if (m_customPlot) {
        m_customPlot->xAxis->setLabel("采样序号");
        m_customPlot->replot();
    }

    m_sampleCountLabel->setText("采样数: 0");
    m_bufferUsageLabel->setText("缓冲区: 0%");

    // 重置三角波检测器
    if (m_triangleDetector) {
        m_triangleDetector->reset();
        // 【修改】更新状态文本和颜色
        QString waitingText = "学习进度: 等待数据...";
        m_learningProgressLabel->setText(waitingText);
        updateLearningProgressColor(waitingText);

        m_triangleStatusLabel->setText("");
        updateTriangleStatusColor("");

        QString initText = "初始化: 等待稳定...";
        m_initializationStatusLabel->setText(initText);
        updateInitializationStatusColor(initText);

        QString cycleText = "周期质量: 待评估";
        m_cycleValidityLabel->setText(cycleText);
        updateCycleValidityColor(cycleText);

        QString detectionText = "检测质量: 初始化中";
        m_detectionQualityLabel->setText(detectionText);
        updateDetectionQualityColor(detectionText);
    }
}


void USBVisualizerMainWindow::updatePlotSettings()
{
    qDebug() << "更新图表设置";

    int oldBufferSize = m_maxBufferSize;
    m_maxBufferSize = m_bufferSizeSpin->value();

    if (oldBufferSize != m_maxBufferSize) {
        QMutexLocker locker(&m_dataMutex);

        qDebug() << QString("缓冲区大小从 %1 改为 %2").arg(oldBufferSize).arg(m_maxBufferSize);

        // 保存当前数据（如果需要）
        std::deque<double> tempData;
        if (m_ringBufferFull || m_ringBufferWritePos > 0) {
            // 按正确顺序提取当前数据
            if (!m_ringBufferFull) {
                // 未满，直接复制
                for (int i = 0; i < m_ringBufferWritePos; i++) {
                    tempData.push_back(m_plotDataY[i]);
                }
            } else {
                // 已满，从最旧数据开始
                int readPos = m_ringBufferWritePos;
                for (int i = 0; i < oldBufferSize; i++) {
                    tempData.push_back(m_plotDataY[readPos]);
                    readPos = (readPos + 1) % oldBufferSize;
                }
            }
        }

        // 重新初始化缓冲区
        m_plotDataX.clear();
        m_plotDataY.clear();
        m_plotDataX.resize(m_maxBufferSize);
        m_plotDataY.resize(m_maxBufferSize);

        for (int i = 0; i < m_maxBufferSize; i++) {
            m_plotDataX[i] = static_cast<double>(i);
            m_plotDataY[i] = 0.0;
        }

        // 恢复数据
        m_ringBufferWritePos = 0;
        m_ringBufferFull = false;

        int restoreCount = qMin(static_cast<int>(tempData.size()), m_maxBufferSize);
        for (int i = 0; i < restoreCount; i++) {
            m_plotDataY[m_ringBufferWritePos] = tempData[i];
            m_ringBufferWritePos++;
        }

        if (m_ringBufferWritePos >= m_maxBufferSize) {
            m_ringBufferWritePos = 0;
            m_ringBufferFull = true;
        }

        qDebug() << QString("恢复了 %1 个数据点到新缓冲区").arg(restoreCount);
    }

    if (m_customPlot && !m_autoScaleCheck->isChecked()) {
        m_customPlot->yAxis->setRange(m_yMinSpin->value(), m_yMaxSpin->value());
        m_customPlot->replot();
    }
}


void USBVisualizerMainWindow::onDataReceived(const QByteArray& data)
{
    QElapsedTimer timer;
    timer.start();

    QMutexLocker locker(&m_dataMutex);

    static qint64 pointsNum = 0;
    // 批量处理，减少函数调用开销
    const uint8_t* rawData = reinterpret_cast<const uint8_t*>(data.constData());

    // 将接收到的字节数据转换为uint16_t值
    for (int i = 0; i < data.size() - 1; i += 2) {

        // 使用指针算术，避免多次调用 operator[]，如果数据是大端序（高字节在前）
        uint16_t rawValue = (rawData[i] << 8) | rawData[i + 1];

        uint16_t value = rawValue & 0x0FFF;

        //首先过滤差值不符合要求的数据
        if (std::abs(m_lastValue2 - value) < m_deltaValue)
        {
            continue;
        }

        if (m_decimationFactor == 1)
        {
            // 将数据添加到三角波检测器
            if (m_triangleDetector) {
                m_triangleDetector->addDataPoint(value, QDateTime::currentMSecsSinceEpoch());
            }

            // 无抽取，直接处理
            m_dataBuffer.enqueue(value);

            if (m_isPrintEnabled)
            {
                pointsNum++;
                qDebug()<<"一次读取数据的点数（无数据抽取）："<<pointsNum << "值："<< value;
            }
         } else {
             // 执行数据抽取
             m_decimationSum += value;
             m_decimationCounter++;

             if (m_decimationCounter >= m_decimationFactor)
             {
                 // 计算平均值
//                 uint16_t averageValue = static_cast<uint16_t>(m_decimationSum / m_decimationFactor);
                 uint16_t averageValue = value;

                 //过滤一样的数据
                 if (m_lastValue == averageValue)
                     continue;

                 // 将数据添加到三角波检测器
                 if (m_triangleDetector) {
                     m_triangleDetector->addDataPoint(value, QDateTime::currentMSecsSinceEpoch());
                 }

                 m_dataBuffer.enqueue(averageValue);
                 // 重置计数器
                 m_decimationCounter = 0;
                 m_decimationSum = 0;

                 if (m_isPrintEnabled)
                 {
                     pointsNum++;
                     qDebug()<<"一次读取数据的点数（有数据抽取）："<<pointsNum << "值："<< value;
                 }
                m_lastValue = value;
             }
         }
         m_lastValue2 = value;
    }
    pointsNum = 0;

    qint64 elapsed = timer.elapsed();

    qDebug()<< "onDataReceived triggered, m_dataBuffer size:" << m_dataBuffer.size() <<"  Function execution time:"<<elapsed;

    //TODO:缓存区有数据了以后就开始绘图，保证绘图的时间小于数据接收一次的时间
    if (!m_plotUpdateTimer->isActive())
    {
        m_plotUpdateTimer->start(m_timerPaintEnginValue);
        qDebug()<<"已启动渲染定时器，开始绘制...";
    }
    else{
        qDebug()<<"m_plotUpdateTimer is Active ";
    }

    quint64 elementSize = sizeof(uint16_t); // 2 字节
    quint64 queueSizeInBytes = static_cast<quint64>(m_dataBuffer.size()) * elementSize;

    // 阈值 4GB,防止内存泄露
    quint64 limitBytes = 4ULL * 1024 * 1024 * 1024;

    if (queueSizeInBytes > limitBytes)
    {
        QMessageBox::warning(nullptr, "内存警告", "当前队列占用内存超过 4GB，即将停止采集，强烈建议重启软件重新采集！");
        stopDataCollection();
    }
}


void USBVisualizerMainWindow::processDecimatedData(uint16_t averageValue)
{
    // 将抽取后的平均值加入显示缓冲区
    m_dataBuffer.enqueue(averageValue);

    // 可选：记录抽取信息用于调试
    static int debugCounter = 0;
    if (++debugCounter % 1000 == 0) {
        qDebug() << QString("数据抽取: %1个点的平均值 = %2")
                   .arg(m_decimationFactor)
                   .arg(averageValue);
    }
}

void USBVisualizerMainWindow::updateDecimationSettings()
{
    int newDecimationFactor = m_dataDecimationSpin->value();

    if (newDecimationFactor == m_decimationFactor) {
        return; // 没有变化
    }

    qDebug() << QString("更新数据抽取设置: %1 → %2")
               .arg(m_decimationFactor)
               .arg(newDecimationFactor);

    // 更新抽取参数
    m_decimationFactor = newDecimationFactor;

    // 同步设置USB读取线程的抽取倍数和chunksize
    if (m_readerThread && m_deviceConnected) {
        m_readerThread->setDecimationFactor(m_decimationFactor);
    }

    // 重置抽取状态
    m_decimationCounter = 0;
    m_decimationSum = 0;

    // 清空当前显示缓冲区（可选）
    if (m_plotDataX.size() != 0)
    {
        if (QMessageBox::question(this, "清空缓冲区",
                                 "是否清空当前显示缓冲区以应用新的抽取设置？")
            == QMessageBox::Yes) {
            clearData();
        }
    }


    // 更新状态显示
    QString statusMsg = QString("数据抽取: %1点平均 | 显示采样率: %2 Hz")
                       .arg(m_decimationFactor)
                       .arg(m_currentSampleRate / m_decimationFactor);

    qDebug() << statusMsg;

    // 可选：在状态栏显示当前抽取设置
    if (m_decimationFactor > 1) {
        setWindowTitle(QString("USB数据实时可视化系统 - 抽取比例 1:%1").arg(m_decimationFactor));
    } else {
        setWindowTitle("USB数据实时可视化系统 - 全精度模式");
    }
}

void USBVisualizerMainWindow::onUSBError(const QString& error)
{
    qDebug() << "USB错误:" << error;
    QMessageBox::critical(this, "USB错误", error);
    stopDataCollection();
}

void USBVisualizerMainWindow::onStatisticsUpdated(quint64 bytesPerSecond, quint32 samplesPerSecond)
{
    m_currentDataRate = bytesPerSecond;
    m_currentSampleRate = samplesPerSecond;

    // 在主线程中更新UI
    QMetaObject::invokeMethod(this, "updateStatusLabels", Qt::QueuedConnection);
}

void USBVisualizerMainWindow::updateStatusLabels()
{
    double dataRateKB = m_currentDataRate / 1024.0;

    // 显示原始采样率和显示采样率
    QString dataRateText;
    if (m_decimationFactor > 1) {
        dataRateText = QString("数据率: %1 KB/s (%2→%3 样本/s, 抽取1:%4)")
                      .arg(dataRateKB, 0, 'f', 1)
                      .arg(m_currentSampleRate)
                      .arg(m_currentSampleRate / m_decimationFactor)
                      .arg(m_decimationFactor);
    } else {
        dataRateText = QString("数据率: %1 KB/s (%2 样本/s)")
                      .arg(dataRateKB, 0, 'f', 1)
                      .arg(m_currentSampleRate);
    }

    m_dataRateLabel->setText(dataRateText);
    m_sampleCountLabel->setText(QString("采样数: %1").arg(m_sampleCounter));

    // 计算缓冲区使用率
    int bufferUsage = !m_ringBufferFull ?
        (m_ringBufferWritePos * 100) / m_maxBufferSize : 100;

    m_bufferUsageLabel->setText(QString("缓冲区: %1%").arg(bufferUsage));

}

void USBVisualizerMainWindow::updatePlot()
{
    QElapsedTimer timer;
    timer.start();

    if (!m_dataCollection || !m_customPlot || !m_dataGraph) {
        return;
    }

    processDataBuffer();

    if (m_plotDataX.empty() || m_plotDataY.empty()) {
        return;
    }

    // 检查是否有新数据
    static int lastSampleCount = 0;
    if (m_sampleCounter == lastSampleCount) {
        qDebug()<<"数据已被处理完，暂时没有新数据，即将停止定时器...";
        m_plotUpdateTimer->stop();
        return;  // 没有新数据
    }
    lastSampleCount = m_sampleCounter;

    // 转换数据格式用于绘图
    QVector<double> xData, yData;

    if (!m_ringBufferFull) {
        // 缓冲区未满，只显示已写入的数据
        int dataCount = m_ringBufferWritePos;
        if (dataCount == 0) {
            return;  // 还没有数据
        }

        xData.reserve(dataCount);
        yData.reserve(dataCount);

        for (int i = 0; i < dataCount; i++) {
            xData.append(m_plotDataX[i]);
            yData.append(m_plotDataY[i]);
        }
    } else {
        // 缓冲区已满，需要按正确顺序重组数据
        xData.reserve(m_maxBufferSize);
        yData.reserve(m_maxBufferSize);

        // 从最旧的数据开始（写入位置的下一个位置）
        int readPos = m_ringBufferWritePos;

        for (int i = 0; i < m_maxBufferSize; i++) {
            xData.append(static_cast<double>(i));  // X坐标保持0到maxBufferSize-1
            yData.append(m_plotDataY[readPos]);

            readPos++;
            if (readPos >= m_maxBufferSize) {
                readPos = 0;
            }
        }
    }


    // 更新图表数据
    m_dataGraph->setData(xData, yData);  // true表示数据已排序


    // 固定X轴范围为0到m_maxBufferSize
    m_customPlot->xAxis->setRange(0, m_maxBufferSize);

    // 更新X轴标签，显示实际的采样编号范围
    if (m_sampleCounter > m_maxBufferSize) {
        int startSample = m_sampleCounter - m_maxBufferSize + 1;
        int endSample = m_sampleCounter;
        m_customPlot->xAxis->setLabel(QString("采样序号 (#%1 - #%2)")
                                      .arg(startSample)
                                      .arg(endSample));
    } else {
        m_customPlot->xAxis->setLabel(QString("采样序号 (共%1个)").arg(m_sampleCounter));
    }

    // 自动缩放Y轴
    if (m_autoScaleCheck->isChecked() && !yData.isEmpty()) {
        auto minMaxY = std::minmax_element(yData.begin(), yData.end());
        double yRange = *minMaxY.second - *minMaxY.first;
        double yMargin = qMax(50.0, yRange * 0.1);
        double yMin = qMax(0.0, *minMaxY.first - yMargin);
        double yMax = qMin(4095.0, *minMaxY.second + yMargin);
        m_customPlot->yAxis->setRange(yMin, yMax);
    } else {
        m_customPlot->yAxis->setRange(m_yMinSpin->value(), m_yMaxSpin->value());
    }

    // 重绘图表
    m_customPlot->replot();

    qint64 elapsed = timer.elapsed();

    //其实绘制300个点也就20ms左右的处理时间
    if (m_isPrintEnabled)
    {
         qDebug() << "updatePlot Function execution time:"<< elapsed;
    }

    qDebug() << "updatePlot triggered ended, m_dataBuffer size:" << m_dataBuffer.size();
}

void USBVisualizerMainWindow::processDataBuffer()
{
    QElapsedTimer timer;
    timer.start();

    QMutexLocker locker(&m_dataMutex);

    // 确保缓冲区已初始化到正确大小
    if (m_plotDataX.size() != m_maxBufferSize || m_plotDataY.size() != m_maxBufferSize)
    {
        m_plotDataX.clear();
        m_plotDataY.clear();

        // 预分配空间
        m_plotDataX.resize(m_maxBufferSize);
        m_plotDataY.resize(m_maxBufferSize);

        // 初始化X坐标（固定不变）
        for (int i = 0; i < m_maxBufferSize; i++) {
            m_plotDataX[i] = static_cast<double>(i);
            m_plotDataY[i] = 0.0;  // 初始Y值为0
        }

        m_ringBufferWritePos = 0;
        m_ringBufferFull = false;
    }

    // 批量处理缓冲区中的数据
    int processCount = qMin(300, m_dataBuffer.size());

    if (processCount == 0) {
        return;  // 没有数据需要处理
    }

    for (int i = 0; i < processCount; i++) {
        if (m_dataBuffer.isEmpty()) {
            break;
        }

        uint16_t value = m_dataBuffer.dequeue();

        // 直接写入环形缓冲区的当前位置
        m_plotDataY[m_ringBufferWritePos] = static_cast<double>(value);

        // 更新写入位置
        m_ringBufferWritePos++;
        if (m_ringBufferWritePos >= m_maxBufferSize) {
            m_ringBufferWritePos = 0;
            m_ringBufferFull = true;//这里的环形缓冲区指的是m_plotDataY而不是m_dataBuffer，m_dataBuffer的大小由chunkSize决定
        }

        m_sampleCounter++;
    }

    qint64 elapsed = timer.elapsed();
}



void USBVisualizerMainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);

    // 确保图表在窗口大小改变时正确重绘
    if (m_customPlot) {
        QTimer::singleShot(100, [this]() { m_customPlot->replot(); });
    }
}

void USBVisualizerMainWindow::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);

    // 窗口显示后确保图表正确渲染
    if (m_customPlot) {
        QTimer::singleShot(200, [this]() { m_customPlot->replot(); });
    }
}
/********************************异常监测记录结果*************************************/
void USBVisualizerMainWindow::onRecorderThreadFinished(int totalPoints, const QString& filename)
{
    LOG_INFO_CL("异常记录线程完成: {} 个数据点", totalPoints);
    LOG_INFO_CL("异常记录完成，文件: {}", filename);

    // 恢复UI状态
    updateRecordingUI(false);

    // 生成异常报告
    QString anomalyTypeStr = getAnomalyTypeString(m_currentAnomalyTrigger.type);

    LOG_INFO_CL("=== 异常报告摘要 ===");
    LOG_INFO_CL("异常类型: {}", anomalyTypeStr);
    LOG_INFO_CL("严重程度: {}%", int(m_currentAnomalyTrigger.severity * 100));
    LOG_INFO_CL("触发值: {}", m_currentAnomalyTrigger.triggerValue);
    LOG_INFO_CL("记录时长: {} ms", QDateTime::currentMSecsSinceEpoch() - m_recordingStartTime);
    LOG_INFO_CL("数据文件: {}", filename);
    LOG_INFO_CL("==================");

    // 显示完成消息
    QMessageBox::information(this, "异常记录完成",
                           QString("异常数据已保存到:\n%1\n\n"
                                  "异常类型: %2\n"
                                  "严重程度: %3%\n"
                                  "共记录: %4 个数据点")
                           .arg(filename)
                           .arg(anomalyTypeStr)
                           .arg(int(m_currentAnomalyTrigger.severity * 100))
                           .arg(totalPoints));

    // 在图表上添加记录完成标记（可选）
    if (m_customPlot && !m_plotDataX.empty()) {
        // 添加结束标记
        QCPItemLine* endMarker = new QCPItemLine(m_customPlot);
        endMarker->start->setCoords(m_plotDataX.back(), m_customPlot->yAxis->range().lower);
        endMarker->end->setCoords(m_plotDataX.back(), m_customPlot->yAxis->range().upper);
        endMarker->setPen(QPen(Qt::green, 2, Qt::DashLine));

        m_customPlot->replot();
    }
}

void USBVisualizerMainWindow::onRecorderThreadError(const QString& error)
{
    LOG_ERROR_CL("异常记录错误: {}", error);
    LOG_ERROR_CL("记录失败: {}", error);

    // 恢复UI状态
    updateRecordingUI(false);

    // 显示错误消息
    QMessageBox::critical(this, "记录错误",
                         QString("异常记录过程中发生错误:\n%1").arg(error));
}

/***********************************异常监测代码*************************************/

void USBVisualizerMainWindow::setupTriangleDetector()
{
    m_triangleDetector = new OptimizedTriangleAnomalyDetector(this);

    // 连接信号槽
    connect(m_triangleDetector, &OptimizedTriangleAnomalyDetector::anomalyDetected,
            this, &USBVisualizerMainWindow::onTriangleAnomalyDetected);
    connect(m_triangleDetector, &OptimizedTriangleAnomalyDetector::learningProgress,
            this, &USBVisualizerMainWindow::onTriangleLearningProgress);
    connect(m_triangleDetector, &OptimizedTriangleAnomalyDetector::learningCompleted,
            this, &USBVisualizerMainWindow::onTriangleLearningCompleted);

    // 设置检测阈值（可根据需要调整）
    m_triangleDetector->setAnomalyThresholds(3.0, 3.0, 2.0, 0.3);

    // 异常记录线程
    m_recorderThread = new AnomalyRecorderThread(this);
    connect(m_recorderThread, &AnomalyRecorderThread::recordingFinished, this, &USBVisualizerMainWindow::onRecorderThreadFinished);
    connect(m_recorderThread, &AnomalyRecorderThread::recordingError, this, &USBVisualizerMainWindow::onRecorderThreadError);

    qDebug() << "三角波检测器初始化完成";
}

// 添加新的槽函数：
void USBVisualizerMainWindow::onTriangleAnomalyDetected(const TriangleAnomalyResult& anomaly)
{
    qDebug() << "检测到三角波异常：" << anomaly.description;

    // 生成异常记录文件名
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString anomalyTypeStr = getAnomalyTypeString(anomaly.type);
    m_anomalyRecordFileName = QString("anomaly_%1_%2.csv")
                                .arg(timestamp)
                                .arg(anomalyTypeStr);

    // 启动异常记录线程
    if (m_recorderThread && !m_recorderThread->isRunning()) {
        m_currentAnomalyTrigger = anomaly;
        m_recordingStartTime = QDateTime::currentMSecsSinceEpoch();

        if (m_recorderThread->startRecording(m_anomalyRecordFileName, anomaly)) {
            qDebug() << "异常记录已启动：" << m_anomalyRecordFileName;

            // 更新UI状态
            updateRecordingUI(true);

            // 在图表中标记异常点（可选）
            markAnomalyOnChart(anomaly);
        } else {
            qDebug() << "启动异常记录失败";
        }
    }

    // 更新状态显示
    QString statusText = QString("异常: %1 (严重度: %2%)")
                            .arg(anomalyTypeStr)
                            .arg(anomaly.severity * 100, 0, 'f', 1);
    m_triangleStatusLabel->setText(statusText);
    updateTriangleStatusColor(statusText);
}

void USBVisualizerMainWindow::onTriangleLearningProgress(int current, int total)
{
    QString studyProgress = QString("学习进度: %1/%2 (%3%)")
            .arg(current)
            .arg(total)
            .arg(current * 100 / total);

    m_learningProgressLabel->setText(studyProgress);

    updateLearningProgressColor(studyProgress);

    qDebug()<<"学习进度：" << studyProgress;
}

void USBVisualizerMainWindow::onTriangleLearningCompleted(const TriangleStats& stats)
{
    QString completedText = "学习完成！√";
    m_learningProgressLabel->setText(completedText);
    updateLearningProgressColor(completedText); // 【新增】更新颜色

    QString statusText = QString("已学习 - 周期:%1点, 波峰:%2, 波谷:%3, 上升斜率:%4, 下降斜率:%5")
            .arg(stats.avgPeriodLength)
            .arg(stats.avgPeakValue, 0, 'f', 0)
            .arg(stats.avgValleyValue, 0, 'f', 0)
            .arg(stats.avgRisingSlope)
            .arg(stats.avgFallingSlope);

    m_triangleStatusLabel->setText(statusText);
    updateTriangleStatusColor(statusText); //

    // 更新详细状态
    QString initText = "初始化: 完成 √";
    m_initializationStatusLabel->setText(initText);
    updateInitializationStatusColor(initText); // 【新增】更新颜色

    QString cycleText = QString("周期质量: %1个有效周期").arg(stats.totalCycles);
    m_cycleValidityLabel->setText(cycleText);
    updateCycleValidityColor(cycleText); // 【新增】更新颜色

    QString detectionText = "检测质量: 正常监测中";
    m_detectionQualityLabel->setText(detectionText);
    updateDetectionQualityColor(detectionText); // 【新增】更新颜色

    qDebug() << "三角波学习完成，开始实时异常检测";
}


QString USBVisualizerMainWindow::getAnomalyTypeString(TriangleAnomalyType type)
{
    switch (type) {
        case TriangleAnomalyType::RisingSlopeAnomaly:
            return "上升斜率异常";
        case TriangleAnomalyType::FallingSlopeAnomaly:
            return "下降斜率异常";
        case TriangleAnomalyType::PeakValueAnomaly:
            return "波峰异常";
        case TriangleAnomalyType::ValleyValueAnomaly:
            return "波谷异常";
        case TriangleAnomalyType::PeriodAnomaly:
            return "周期异常";
        case TriangleAnomalyType::DataJumpAnomaly:  // 新增
            return "数据跳变异常";
        default:
            return "未知异常";
    }
}

void USBVisualizerMainWindow::markAnomalyOnChart(const TriangleAnomalyResult& anomaly)
{
    if (!m_customPlot) return;

    // 在图表上添加异常标记
    QCPItemTracer* tracer = new QCPItemTracer(m_customPlot);
    tracer->setGraph(m_dataGraph);
    tracer->setGraphKey(m_sampleCounter); // 当前采样点
    tracer->setInterpolating(true);
    tracer->setStyle(QCPItemTracer::tsCircle);
    tracer->setPen(QPen(Qt::red, 3));
    tracer->setBrush(QBrush(Qt::red));
    tracer->setSize(8);

    // 添加文本标签
    QCPItemText* textLabel = new QCPItemText(m_customPlot);
    textLabel->setPositionAlignment(Qt::AlignTop | Qt::AlignHCenter);
    textLabel->position->setType(QCPItemPosition::ptPlotCoords);
    textLabel->position->setCoords(m_sampleCounter, anomaly.triggerValue + 200);
    textLabel->setText(QString("异常: %1").arg(getAnomalyTypeString(anomaly.type)));
    textLabel->setFont(QFont(font().family(), 8));
    textLabel->setPen(QPen(Qt::red));
    textLabel->setBrush(QBrush(QColor(255, 255, 255, 200)));

    m_customPlot->replot();
}

void USBVisualizerMainWindow::updateRecordingUI(bool isRecording)
{
    if (isRecording) {
           QString recordingText = "检测质量: 记录异常中...";
           m_detectionQualityLabel->setText(recordingText);
           updateDetectionQualityColor(recordingText); // 【新增】更新颜色
       } else {
           QString normalText = "检测质量: 正常监测中";
           m_detectionQualityLabel->setText(normalText);
           updateDetectionQualityColor(normalText); // 【新增】更新颜色
       }
}


/***********************************颜色处理****************************************/


// 设置标签颜色的通用函数
void USBVisualizerMainWindow::setLabelColor(QLabel* label, const QString& color, bool bold)
{
    QString style = QString("color: %1;").arg(color);
    if (bold) {
        style += " font-weight: bold;";
    }
    label->setStyleSheet(style);
}

// 学习进度颜色管理
void USBVisualizerMainWindow::updateLearningProgressColor(const QString& status)
{
    if (status.contains("等待数据") || status.contains("等待")) {
        setLabelColor(m_learningProgressLabel, "#888888"); // 灰色
    } else if (status.contains("学习进度") && !status.contains("完成")) {
        setLabelColor(m_learningProgressLabel, "#FFA500", true); // 橙色，加粗
    } else if (status.contains("完成")) {
        setLabelColor(m_learningProgressLabel, "#00AA00", true); // 绿色，加粗
    } else if (status.contains("失败") || status.contains("错误")) {
        setLabelColor(m_learningProgressLabel, "#FF4444", true); // 红色，加粗
    }
}

// 三角波状态颜色管理
void USBVisualizerMainWindow::updateTriangleStatusColor(const QString& status)
{
    if (status.isEmpty() || status.contains("未连接")) {
        setLabelColor(m_triangleStatusLabel, "#888888"); // 灰色
    } else if (status.contains("学习") || status.contains("已学习")) {
        setLabelColor(m_triangleStatusLabel, "#00AA00"); // 绿色
    } else if (status.contains("异常")) {
        setLabelColor(m_triangleStatusLabel, "#FF4444", true); // 红色，加粗
    } else {
        setLabelColor(m_triangleStatusLabel, "#FFFFFF"); // 默认白色
    }
}

// 初始化状态颜色管理
void USBVisualizerMainWindow::updateInitializationStatusColor(const QString& status)
{
    if (status.contains("等待稳定") || status.contains("等待")) {
        setLabelColor(m_initializationStatusLabel, "#888888"); // 灰色
    } else if (status.contains("初始化中") || status.contains("进行中")) {
        setLabelColor(m_initializationStatusLabel, "#FFA500"); // 橙色
    } else if (status.contains("完成")) {
        setLabelColor(m_initializationStatusLabel, "#00AA00"); // 绿色
    } else if (status.contains("失败") || status.contains("错误")) {
        setLabelColor(m_initializationStatusLabel, "#FF4444"); // 红色
    }
}

// 周期质量颜色管理
void USBVisualizerMainWindow::updateCycleValidityColor(const QString& status)
{
    if (status.contains("待评估") || status.contains("等待")) {
        setLabelColor(m_cycleValidityLabel, "#888888"); // 灰色
    } else if (status.contains("评估中") || status.contains("分析中")) {
        setLabelColor(m_cycleValidityLabel, "#FFA500"); // 橙色
    } else if (status.contains("质量") && (status.contains("好") || status.contains("有效") || status.contains("个有效周期"))) {
        setLabelColor(m_cycleValidityLabel, "#00AA00"); // 绿色
    } else if (status.contains("质量差") || status.contains("无效") || status.contains("异常")) {
        setLabelColor(m_cycleValidityLabel, "#FF4444"); // 红色
    }
}

// 检测质量颜色管理
void USBVisualizerMainWindow::updateDetectionQualityColor(const QString& status)
{
    if (status.contains("初始化中") || status.contains("等待")) {
        setLabelColor(m_detectionQualityLabel, "#888888"); // 灰色
    } else if (status.contains("正常监测") || status.contains("正常")) {
        setLabelColor(m_detectionQualityLabel, "#00AA00"); // 绿色
    } else if (status.contains("记录异常中") || status.contains("异常")) {
        setLabelColor(m_detectionQualityLabel, "#FF4444", true); // 红色，加粗
    } else if (status.contains("准备中") || status.contains("启动中")) {
        setLabelColor(m_detectionQualityLabel, "#FFA500"); // 橙色
    }
}

// 重置所有状态颜色
void USBVisualizerMainWindow::resetAllStatusColors()
{
    setLabelColor(m_learningProgressLabel, "#888888");
    setLabelColor(m_triangleStatusLabel, "#888888");
    setLabelColor(m_initializationStatusLabel, "#888888");
    setLabelColor(m_cycleValidityLabel, "#888888");
    setLabelColor(m_detectionQualityLabel, "#888888");
}


