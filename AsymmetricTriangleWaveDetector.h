// AsymmetricTriangleWaveDetector.h
#ifndef ASYMMETRICTRIANGEWAVEDETECTOR_H
#define ASYMMETRICTRIANGEWAVEDETECTOR_H

#include <QObject>
#include <QTimer>
#include <QDateTime>
#include <QDebug>
#include <deque>
#include <QMap>
#include "MCLogger.h"

// 非对称三角波的异常类型
enum class TriangleAnomalyType {
    None = 0,
    FrequencyDrift,          // 频率漂移
    PeakAnomaly,            // 波峰异常
    ValleyAnomaly,          // 波谷异常
    RisingSlopeAnomaly,     // 上升斜率异常
    FallingSlopeAnomaly,    // 下降斜率异常
    AsymmetryRatioAnomaly,  // 非对称比例异常
    AmplitudeAnomaly,       // 幅度异常
    NoiseAnomaly,           // 噪声异常
    PhaseJump               // 相位跳变
};

// 三角波相位
enum class TrianglePhase {
    Rising,     // 上升阶段
    Falling,    // 下降阶段
    Unknown     // 未知阶段
};

// 异常检测结果
struct TriangleAnomalyResult {
    TriangleAnomalyType type;
    double severity;              // 严重程度 0.0-1.0
    uint16_t triggerValue;       // 触发值
    qint64 timestamp;            // 时间戳
    QString description;         // 描述
    TrianglePhase phaseWhenDetected;  // 检测时的相位

    TriangleAnomalyResult() : type(TriangleAnomalyType::None), severity(0.0),
                              triggerValue(0), timestamp(0),
                              phaseWhenDetected(TrianglePhase::Unknown) {}
};

// 三角波统计参数
struct TriangleWaveStats {
    // 频率相关
    double currentFrequency;      // 当前频率
    double averageFrequency;      // 平均频率
    double frequencyStability;    // 频率稳定性

    // 波峰波谷
    uint16_t currentPeak;         // 当前波峰
    uint16_t currentValley;       // 当前波谷
    uint16_t averagePeak;         // 平均波峰
    uint16_t averageValley;       // 平均波谷
    double amplitude;             // 幅度

    // 斜率相关
    double currentRisingSlope;    // 当前上升斜率
    double currentFallingSlope;   // 当前下降斜率
    double averageRisingSlope;    // 平均上升斜率
    double averageFallingSlope;   // 平均下降斜率

    // 非对称性
    double asymmetryRatio;        // 非对称比（上升时间/下降时间）
    double risingDuration;        // 上升持续时间
    double fallingDuration;       // 下降持续时间

    // 其他
    TrianglePhase currentPhase;   // 当前相位
    int cycleCount;              // 周期计数
    double noiseLevel;           // 噪声水平

    TriangleWaveStats() : currentFrequency(0), averageFrequency(0), frequencyStability(0),
                         currentPeak(0), currentValley(65535), averagePeak(0), averageValley(65535),
                         amplitude(0), currentRisingSlope(0), currentFallingSlope(0),
                         averageRisingSlope(0), averageFallingSlope(0), asymmetryRatio(1.0),
                         risingDuration(0), fallingDuration(0), currentPhase(TrianglePhase::Unknown),
                         cycleCount(0), noiseLevel(0) {}
};

class AsymmetricTriangleWaveDetector : public QObject
{
    Q_OBJECT

public:
    explicit AsymmetricTriangleWaveDetector(QObject *parent = nullptr);
    ~AsymmetricTriangleWaveDetector();

    // 主要接口
    void feedData(uint16_t value);
    void reset();

    // 配置接口
    void setNominalFrequency(double frequency);
    void setExpectedPeakValley(uint16_t peak, uint16_t valley);
    void setExpectedSlopes(double risingSlope, double fallingSlope);
    void setAsymmetryRatio(double ratio);  // 上升时间/下降时间
    void setSamplingRate(double sampleRate);
    void setRecordingDuration(int seconds);

    // 阈值设置
    void setDetectionThresholds(double freqTolerance = 0.05,
                               double peakTolerance = 0.1,
                               double slopeTolerance = 0.1,
                               double asymmetryTolerance = 0.2);

    // 状态查询
    bool isRecording() const { return m_isRecording; }
    TriangleWaveStats getCurrentStats() const { return m_currentStats; }
    TriangleAnomalyResult getLastAnomaly() const { return m_lastAnomaly; }

    // 高级配置
    void enableAnomalyType(TriangleAnomalyType type, bool enabled = true);
    void setAdaptiveMode(bool enabled = true);

signals:
    void anomalyDetected(const TriangleAnomalyResult& anomaly);
    void recordingStarted(const TriangleAnomalyResult& trigger);
    void recordingData(uint16_t value, qint64 timestamp);
    void recordingStopped(int totalDataPoints);
    void statsUpdated(const TriangleWaveStats& stats);

private slots:
    void onRecordingTimeout();

private:
    // 核心检测算法
    TriangleAnomalyResult analyzeTriangleWave();
    bool detectFrequencyDrift(double& severity);
    bool detectPeakAnomaly(double& severity);
    bool detectValleyAnomaly(double& severity);
    bool detectRisingSlopeAnomaly(double& severity);
    bool detectFallingSlopeAnomaly(double& severity);
    bool detectAsymmetryAnomaly(double& severity);
    bool detectAmplitudeAnomaly(double& severity);
    bool detectNoiseAnomaly(double& severity);
    bool detectPhaseJump(double& severity);

    // 波形分析
    void updateTriangleStats();
    void detectPhaseTransition();
    double calculateSlope(int startIdx, int endIdx);
    void updateSlopeHistory();
    void calculateAsymmetryRatio();

    // 自适应学习
    void updateBaseline();

private:
    // 数据缓冲区
    std::deque<uint16_t> m_dataBuffer;
    std::deque<qint64> m_timestampBuffer;

    // 历史记录
    std::deque<double> m_frequencyHistory;
    std::deque<uint16_t> m_peakHistory;
    std::deque<uint16_t> m_valleyHistory;
    std::deque<double> m_risingSlopeHistory;
    std::deque<double> m_fallingSlopeHistory;
    std::deque<double> m_asymmetryHistory;

    // 配置参数
    double m_nominalFrequency;
    uint16_t m_expectedPeak;
    uint16_t m_expectedValley;
    double m_expectedRisingSlope;
    double m_expectedFallingSlope;
    double m_expectedAsymmetryRatio;
    double m_samplingRate;
    int m_recordingDuration;

    // 检测阈值
    double m_frequencyTolerance;
    double m_peakTolerance;
    double m_slopeTolerance;
    double m_asymmetryTolerance;

    // 当前状态
    TriangleWaveStats m_currentStats;
    TriangleAnomalyResult m_lastAnomaly;

    // 相位检测
    TrianglePhase m_previousPhase;
    int m_phaseStartIndex;
    qint64 m_phaseStartTime;
    uint16_t m_lastPeakValue;
    uint16_t m_lastValleyValue;
    qint64 m_lastPeakTime;
    qint64 m_lastValleyTime;

    // 自适应学习
    bool m_adaptiveEnabled;
    double m_baselineFrequency;
    uint16_t m_baselinePeak;
    uint16_t m_baselineValley;
    double m_baselineRisingSlope;
    double m_baselineFallingSlope;
    double m_baselineAsymmetryRatio;
    int m_learningPeriod;
    int m_samplesProcessed;

    // 记录控制
    bool m_isRecording;
    QTimer* m_recordingTimer;
    int m_recordedDataPoints;

    // 异常类型开关
    QMap<TriangleAnomalyType, bool> m_enabledAnomalies;

    // 性能优化
    int m_analysisCounter;
    static const int ANALYSIS_INTERVAL = 10;  // 每10个数据点分析一次
};

#endif // ASYMMETRICTRIANGEWAVEDETECTOR_H
