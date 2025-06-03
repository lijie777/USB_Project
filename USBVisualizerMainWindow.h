// USBVisualizerMainWindow.h
#ifndef USBVISUALIZERMAINWINDOW_H
#define USBVISUALIZERMAINWINDOW_H

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
#include "OptimizedSawtoothAnomalyDetector.h"
#include "USBDebugHelper.h"
#include "USBReaderThread.h"
#include "qcustomplot.h"

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


    void setupSawtoothDetector();

    void createSawtoothDebugMenu();

    // 辅助函数
    void updateRecordingUI(bool isRecording);
    QString getAnomalyTypeString(SawtoothAnomalyType type);

    void resetSawtoolDetector();
private slots:
    void onSawtoothAnomalyDetected(const SawtoothAnomalyResult& anomaly);
    void onSawtoothStatsUpdated(const SawtoothStats& stats);
    void onRecordingStarted(const SawtoothAnomalyResult& trigger);
    void onRecordingData(uint16_t value, qint64 timestamp);
    void onRecordingStopped(int totalDataPoints);

    void showSawtoothDebugInfo();
    void fineTuneSawtoothDetector();

    // 异常记录线程相关槽函数
    void onRecorderThreadFinished(int totalPoints, const QString& filename);
    void onRecorderThreadError(const QString& error);

private:
    // 异常记录相关
    OptimizedSawtoothAnomalyDetector* m_sawtoothDetector;
    AnomalyRecorderThread* m_recorderThread;              // 记录线程
    SawtoothAnomalyResult m_currentAnomalyTrigger;       // 当前触发的异常
    qint64 m_recordingStartTime;                          // 记录开始时间
    QString m_anomalyRecordFileName;                      // 异常记录文件名

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

    //锯齿波状态
    QLabel *m_statusLabelExp;

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
    quint32 m_currentSampleRate;
};

#endif  // USBVISUALIZERMAINWINDOW_H
