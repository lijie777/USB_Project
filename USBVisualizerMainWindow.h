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
#include <QMenuBar>
#include <QAction>
#include "TriangleDetectorConfigDialog.h"
#include "TriangleStatsDisplayDialog.h"

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


private:
    // 【新增】状态标签颜色管理函数
    void setLabelColor(QLabel* label, const QString& color, bool bold = false);
    void updateLearningProgressColor(const QString& status);
    void updateTriangleStatusColor(const QString& status);
    void updateInitializationStatusColor(const QString& status);
    void updateCycleValidityColor(const QString& status);
    void updateDetectionQualityColor(const QString& status);
    void resetAllStatusColors();

public slots:
    // 【新增】菜单相关槽函数
    void showConfigDialog();
    void showStatsDialog();

private:
    void setupMenuBar();
    // 【新增】菜单相关成员
    QMenuBar* m_menuBar;
    QMenu* m_configMenu;
    QAction* m_configAction;
    QAction* m_statsAction;

    TriangleDetectorConfigDialog m_triangleDetectorConfigDialog;
    TriangleStatsDisplayDialog m_triangleStatsDisplayDialog;

    // 【新增】异常状态控制
    bool m_anomalyDetected{false};           // 是否检测到异常
    bool m_recordingInProgress{false};       // 是否正在记录

    // 【新增】配置参数存储
    double m_peakTolerance{10.0};            // 波峰容差百分比
    double m_valleyTolerance{10.0};          // 波谷容差百分比
    double m_slopeTolerance{10.0};           // 斜率容差百分比
    // 【修改】只要两个长度阈值
    int m_risingEdgeThreshold{100};           // 上升沿长度阈值
    int m_fallingEdgeThreshold{100};          // 下降沿长度阈值
    double m_jumpThreshold{20.0};            // 跳变阈值
    int m_learningDataCount{20000};           // 学习数据点数

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


private slots:
    // 新增三角波检测相关槽函数
    void onTriangleAnomalyDetected(const TriangleAnomalyResult& anomaly);
    void onTriangleLearningProgress(int progress, int total);
    void onTriangleLearningCompleted(const TriangleStats& learnedStats);

    //异常检测记录结果
    void onRecorderThreadError(const QString& error);
    void onRecorderThreadFinished(int totalPoints, const QString& filename);
private:
    // 新增辅助函数
    void setupTriangleDetector();
    QString getAnomalyTypeString(TriangleAnomalyType type);
    void markAnomalyOnChart(const TriangleAnomalyResult& anomaly);
    void updateRecordingUI(bool isRecording);

    // 新增成员变量
    OptimizedTriangleAnomalyDetector* m_triangleDetector{nullptr};
    TriangleAnomalyResult m_currentAnomalyTrigger;
    QString m_anomalyRecordFileName;
    qint64 m_recordingStartTime{0};
    AnomalyRecorderThread * m_recorderThread;

private:

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

    uint16_t m_lastValue{0};
    uint16_t m_lastValue2{0};
};

#endif  // USBVISUALIZERMAINWINDOW_H
