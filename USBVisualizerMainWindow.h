// USBVisualizerMainWindow.h
#ifndef USBVISUALIZERMAINWINDOW_H
#define USBVISUALIZERMAINWINDOW_H

#include <QCheckBox>
#include <libusb-1.0/libusb.h>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QMutex>
#include <QPushButton>
#include <QQueue>
#include <QResizeEvent>
#include <QShowEvent>
#include <QSpinBox>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <atomic>
#include <deque>
#include <memory>
#include "AnomalyRecorderThread.h"
#include "USBDebugHelper.h"
#include "USBReaderThread.h"
#include "qcustomplot.h"
#include "OptimizedTriangleAnomalyDetector.h"  // 只保留三角波检测器

// 主窗口类
class USBVisualizerMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    USBVisualizerMainWindow(QWidget* parent = nullptr);
    ~USBVisualizerMainWindow();

protected:
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

private slots:
    //设备管理
    void refreshDevices();
    void connectDevice();
    void disconnectDevice();

    //数据管理
    void startDataCollection();
    void stopDataCollection();
    void clearData();

    void updatePlotSettings();
    void onDataReceived(const QByteArray& data);
    void onUSBError(const QString& error);
    void onStatisticsUpdated(quint64 bytesPerSecond, quint32 samplesPerSecond);
    void updatePlot();
    void updateStatusLabels();

private:
    void setupUI();
    void setupPlot();
    void updateDeviceCombo();
    void processDataBuffer();

    void setupTriangleDetector();        // 三角波检测器设置
    void createTriangleDebugMenu();      // 三角波调试菜单

    // 辅助函数
    void updateRecordingUI(bool isRecording);
    QString getAnomalyTypeString(TriangleAnomalyType type);  // 简化的异常类型
    void resetTriangleDetector();        // 重置检测器
    void updateInitializationStatus(bool stabilizationComplete, bool rangeEstimationComplete);  // 新增

private slots:
    void onTriangleAnomalyDetected(const TriangleAnomalyResult& anomaly);
    void onTriangleStatsUpdated(const TriangleStats& stats);
    void onTriangleLearningProgress(int progress, int total);
    void onTriangleLearningCompleted(const TriangleStats& learnedStats);
    void onRecordingStarted(const TriangleAnomalyResult& trigger);

    void onRecordingData(uint16_t value, qint64 timestamp);
    void onRecordingStopped(int totalDataPoints);

    void showTriangleDebugInfo();       // 调试信息

    // 异常记录线程相关槽函数
    void onRecorderThreadFinished(int totalPoints, const QString& filename);
    void onRecorderThreadError(const QString& error);

signals:
    //当满足一个周期数据时发送数据
    void periodicSignal();
// 抽取倍数新增代码
private:
    // 数据抽取相关
    QSpinBox* m_dataDecimationSpin;           // 数据抽取倍数输入框
    QPushButton* m_applyDecimationBtn;        // 应用抽取设置按钮
    int m_decimationFactor;                   // 当前抽取倍数
    QSpinBox *m_deltaValueSpin;               //点差值
    // 数据处理缓冲区
    int m_decimationCounter;                  // 抽取计数器
    uint64_t m_decimationSum;                 // 累加和

    //数据读取一次的大小
    QSpinBox *m_bufferReadSpin;

    QSpinBox *m_timerPaintEnginSpin;               //绘制定时器
    QPushButton *m_applytimerPaintEnginBtn;
    int m_timerPaintEnginValue;                   //绘制定时器间隔

    QPushButton *m_applyDeltaBtn;
    QPushButton *m_applyBufferReadBtn;

    int m_deltaValue;//点差值
    int m_bufferReadValue;//数据读取缓冲区大小

    // 方法声明
    void processDecimatedData(uint16_t averageValue);
    void updateDecimationSettings();

private:
    // 线程睡眠配置
    QSpinBox* m_sleepTimeSpin;               // 睡眠时间输入框
    QPushButton* m_applyThreadConfigBtn;     // 应用按钮

private slots:
    void applySleepTimeConfig();             // 应用睡眠时间配置
    void onDebugPrintToggled(bool enabled);

private:
    // 异常记录相关
    OptimizedTriangleAnomalyDetector* m_triangleDetector;        // 三角波检测器
    AnomalyRecorderThread* m_recorderThread;                     // 记录线程
    TriangleAnomalyResult m_currentAnomalyTrigger;              // 三角波异常结果
    qint64 m_recordingStartTime;                                 // 记录开始时间
    QString m_anomalyRecordFileName;                             // 异常记录文件名

    // UI组件
    QWidget* m_centralWidget;
    QVBoxLayout* m_mainLayout;

    // 设备控制区域
    QGroupBox* m_deviceGroup;
    QComboBox* m_deviceCombo;
    QPushButton* m_refreshBtn;
    QPushButton* m_connectBtn;
    QPushButton* m_disconnectBtn;

    // 数据控制区域
    QGroupBox* m_dataGroup;
    QPushButton* m_startBtn;
    QPushButton* m_stopBtn;
    QPushButton* m_clearBtn;

    // 配置区域
    QGroupBox* m_configGroup;
    QSpinBox* m_yMinSpin;
    QSpinBox* m_yMaxSpin;
    QCheckBox* m_autoScaleCheck;
    QSpinBox* m_bufferSizeSpin;

    // 状态显示
    QLabel* m_statusLabel;
    QLabel* m_dataRateLabel;
    QLabel* m_sampleCountLabel;
    QLabel* m_bufferUsageLabel;

    // 三角波状态 - 扩展版
    QLabel* m_triangleStatusLabel;
    QLabel* m_learningProgressLabel;
    QLabel* m_initializationStatusLabel;    // 新增：初始化状态
    QLabel* m_cycleValidityLabel;          // 新增：周期有效性
    QLabel* m_detectionQualityLabel;       // 新增：检测质量

    // 图表
    QGroupBox* m_plotGroup;
    QCustomPlot* m_customPlot;
    QCPGraph* m_dataGraph;

    // USB相关
    libusb_context* m_usbContext;
    QList<USBDeviceInfo> m_deviceList;
    USBReaderThread* m_readerThread;

    // 数据处理
    QMutex m_dataMutex;
    QQueue<uint16_t> m_dataBuffer;
    std::deque<double> m_plotDataX;
    std::deque<double> m_plotDataY;
    quint64 m_sampleCounter;
    int m_maxBufferSize;

    // 定时器
    QTimer* m_plotUpdateTimer;

    // 状态
    bool m_deviceConnected;
    bool m_dataCollection;
    QString m_currentDeviceInfo;

    // 性能统计
    quint64 m_currentDataRate;
    quint32 m_currentSampleRate; //每秒采样点数

    // 环形缓冲区相关
    int m_ringBufferWritePos{0};    // 环形缓冲区写入位置
    bool m_ringBufferFull{false};   // 缓冲区是否已满

    QCheckBox *m_debugCheckBox;
    bool m_isPrintEnabled{false};

    //波峰和波谷
    qint16 m_peakValue{0};
    qint16 m_valleyValue{0};
    uint16_t m_lastValue{0};
    uint16_t m_lastValue2{0};

    // 取固定的点做计算，算出波峰，波谷，和上升斜率和下降斜率，及周期
    QList<uint16_t> m_baseCalcPointsList;

};

#endif  // USBVISUALIZERMAINWINDOW_H
