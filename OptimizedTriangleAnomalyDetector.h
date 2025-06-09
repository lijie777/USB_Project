// OptimizedTriangleAnomalyDetector.h
// 针对非对称三角波的优化异常检测算法

#ifndef OPTIMIZEDTRIANGLEANOMALYDETECTOR_H
#define OPTIMIZEDTRIANGLEANOMALYDETECTOR_H

#include <QObject>
#include <QQueue>
#include <QTimer>
#include <QDateTime>
#include <QDebug>
#include <deque>
#include <functional>

// 三角波检测参数配置
#define NOMINAL_FREQUENCY 3.76      // 标称频率，初始设为0让程序自动学习
#define FREQUENCY_TOLERANCE 0.05    // 频率容差 5%
#define EXPECTED_MIN 0              // 期望最小值
#define EXPECTED_MAX 4095          // 期望最大值
#define AMPLITUDE_TOLERANCE 0.1     // 幅度容差 10%
#define SLOPE_TOLERANCE 0.15        // 斜率容差 15%
#define ASYMMETRY_TOLERANCE 0.2     // 非对称比例容差 20%
#define BASELINE_AMPLITUDE 0        // 基准幅度，自动学习
#define SAMPLING_RATE 10000.0      // 采样率 10kHz
#define LEARNING_PERIOD 2000       // 学习周期 2000个样本
#define LINEARITY_THRESHOLD 0.85   // 线性度阈值（三角波要求稍低）

// 三角波特定的异常类型
enum class TriangleAnomalyType {
    None = 0,
    FrequencyDrift,        // 频率漂移
    AmplitudeDrift,        // 幅度变化
    RisingSlopeError,      // 上升沿斜率异常
    FallingSlopeError,     // 下降沿斜率异常
    AsymmetryError,        // 非对称比例异常
    PeakValueError,        // 波峰值异常
    ValleyValueError,      // 波谷值异常
    PeriodDistortion,      // 周期失真
    LinearityError,        // 线性度误差
    NoiseSpike,            // 噪声尖峰
    PhaseJump,             // 相位突跳
    Saturation,            // 波形削顶
    Dropout                // 信号丢失
};

// 三角波状态
enum class TrianglePhase {
    Rising,                // 上升阶段
    Falling,               // 下降阶段
    AtPeak,                // 在波峰附近
    AtValley,              // 在波谷附近
    Unknown                // 未知阶段
};

// 异常检测结果
struct TriangleAnomalyResult {
    TriangleAnomalyType type;
    double severity;                    // 严重程度 0.0-1.0
    uint16_t triggerValue;
    qint64 timestamp;
    QString description;
    TrianglePhase phaseWhenDetected;    // 发生异常时的波形阶段

    TriangleAnomalyResult() : type(TriangleAnomalyType::None), severity(0.0),
                             triggerValue(0), timestamp(0), phaseWhenDetected(TrianglePhase::Unknown) {}
};

// 三角波统计参数
struct TriangleStats {
    double currentFrequency;            // 当前频率
    double averageFrequency;            // 平均频率
    double frequencyStability;          // 频率稳定性
    uint16_t currentMin;                // 当前周期最小值
    uint16_t currentMax;                // 当前周期最大值
    uint16_t averageAmplitude;          // 平均幅度
    double risingSlope;                 // 上升沿斜率
    double fallingSlope;                // 下降沿斜率
    double averageRisingSlope;          // 平均上升沿斜率
    double averageFallingSlope;         // 平均下降沿斜率
    double asymmetryRatio;              // 非对称比例 (上升时间/下降时间)
    double averageAsymmetryRatio;       // 平均非对称比例
    double risingLinearity;             // 上升沿线性度
    double fallingLinearity;            // 下降沿线性度
    TrianglePhase currentPhase;         // 当前阶段
    int cycleCount;                     // 周期计数
    double noiseLevel;                  // 噪声水平
    qint64 risingDuration;              // 上升持续时间(ms)
    qint64 fallingDuration;             // 下降持续时间(ms)
    qint64 totalPeriod;                 // 总周期时间(ms)

    TriangleStats() : currentFrequency(0), averageFrequency(0), frequencyStability(0),
                     currentMin(65535), currentMax(0), averageAmplitude(0),
                     risingSlope(0), fallingSlope(0), averageRisingSlope(0), averageFallingSlope(0),
                     asymmetryRatio(1.0), averageAsymmetryRatio(1.0),
                     risingLinearity(1.0), fallingLinearity(1.0),
                     currentPhase(TrianglePhase::Unknown), cycleCount(0), noiseLevel(0),
                     risingDuration(0), fallingDuration(0), totalPeriod(0) {}
};

// 内部数据结构：相位段信息
struct PhaseSegment {
    TrianglePhase phase;
    int startIndex;
    int endIndex;
    qint64 startTime;
    qint64 endTime;
    uint16_t startValue;
    uint16_t endValue;
    double slope;
    double linearity;

    PhaseSegment() : phase(TrianglePhase::Unknown), startIndex(0), endIndex(0),
                    startTime(0), endTime(0), startValue(0), endValue(0),
                    slope(0), linearity(1.0) {}
};

class OptimizedTriangleAnomalyDetector : public QObject
{
    Q_OBJECT

public:
    explicit OptimizedTriangleAnomalyDetector(QObject *parent = nullptr);
    ~OptimizedTriangleAnomalyDetector();

    // 主要接口
    void feedData(uint16_t value);
    void setOptimalParameters();

    // 参数配置
    void setNominalFrequency(double frequency);
    void setExpectedAmplitude(uint16_t minVal, uint16_t maxVal);
    void setDetectionThresholds(double freqTolerance = 0.05,      // 频率容差 5%
                               double ampTolerance = 0.1,         // 幅度容差 10%
                               double slopeTolerance = 0.15,      // 斜率容差 15%
                               double asymmetryTolerance = 0.2);  // 非对称容差 20%
    void setRecordingDuration(int seconds = 10);
    void reset();

    // 状态查询
    bool isRecording() const { return m_isRecording; }
    TriangleStats getCurrentStats() const { return m_currentStats; }
    TriangleAnomalyResult getLastAnomaly() const { return m_lastAnomaly; }
    bool isLearningComplete() const { return m_samplesProcessed >= m_learningPeriod; }

    // 高级配置
    void enableAnomalyType(TriangleAnomalyType type, bool enabled = true);
    void setSamplingRate(double sampleRate);
    void setAdaptiveThresholds(bool enabled = true);
    void setLearningPeriod(int samples) { m_learningPeriod = samples; }

signals:
    void anomalyDetected(const TriangleAnomalyResult& anomaly);
    void recordingStarted(const TriangleAnomalyResult& trigger);
    void recordingData(uint16_t value, qint64 timestamp);
    void recordingStopped(int totalDataPoints);
    void statsUpdated(const TriangleStats& stats);
    void learningProgressUpdated(int progress, int total);
    void learningCompleted(const TriangleStats& learnedStats);

private slots:
    void onRecordingTimeout();

private:
    // 核心检测算法
    TriangleAnomalyResult analyzeTriangle();
    bool detectFrequencyDrift(double& severity);
    bool detectAmplitudeDrift(double& severity);
    bool detectRisingSlopeError(double& severity);
    bool detectFallingSlopeError(double& severity);
    bool detectAsymmetryError(double& severity);
    bool detectPeakValueError(double& severity);
    bool detectValleyValueError(double& severity);
    bool detectPeriodDistortion(double& severity);
    bool detectLinearityError(double& severity);
    bool detectNoiseSpike(double& severity);
    bool detectPhaseJump(double& severity);
    bool detectSaturation(double& severity);
    bool detectDropout(double& severity);

    // 三角波分析
    void updateTriangleStats();
    void detectPhaseTransition();
    void analyzeCurrentPhase();
    void processCompleteCycle();
    double calculateCurrentFrequency();
    void findPeaksAndValleys();
    bool isValidTriangleCycle();

    // 相位段分析
    void analyzePhaseSegment(const PhaseSegment& segment);
    double calculateSegmentSlope(int startIdx, int endIdx);
    double calculateSegmentLinearity(int startIdx, int endIdx);
    bool isMonotonicSegment(int startIdx, int endIdx, bool shouldIncrease);

    // 自适应学习
    void updateBaseline();
    void finalizeBaseline();
    void adaptThresholds();

    // 辅助函数
    double calculateSlope(int startIdx, int endIdx);
    double calculateCorrelation(const std::vector<double>& x, const std::vector<double>& y);
    TrianglePhase determinePhase(int currentIdx);
    void smoothData(int windowSize = 5);

private:
    // 数据缓冲区
    std::deque<uint16_t> m_dataBuffer;          // 数据缓冲区
    std::deque<qint64> m_timestampBuffer;       // 时间戳缓冲区
    std::deque<double> m_frequencyHistory;      // 频率历史
    std::deque<uint16_t> m_amplitudeHistory;    // 幅度历史
    std::deque<double> m_risingSlopeHistory;    // 上升斜率历史
    std::deque<double> m_fallingSlopeHistory;   // 下降斜率历史
    std::deque<double> m_asymmetryHistory;      // 非对称比例历史

    // 三角波参数
    double m_nominalFrequency;                  // 标称频率
    uint16_t m_expectedMin;                     // 期望最小值
    uint16_t m_expectedMax;                     // 期望最大值
    double m_samplingRate;                      // 采样率

    // 检测阈值
    double m_frequencyTolerance;                // 频率容差
    double m_amplitudeTolerance;                // 幅度容差
    double m_slopeTolerance;                    // 斜率容差
    double m_asymmetryTolerance;                // 非对称容差
    double m_linearityThreshold;                // 线性度阈值

    // 当前状态
    TriangleStats m_currentStats;               // 三角波当前状态
    TriangleAnomalyResult m_lastAnomaly;        // 异常检测结果

    // 相位检测
    TrianglePhase m_previousPhase;
    int m_phaseStartIndex;                      // 当前阶段开始索引
    qint64 m_phaseStartTime;                    // 当前阶段开始时间
    uint16_t m_lastPeakValue;                   // 上个峰值
    uint16_t m_lastValleyValue;                 // 上个谷值
    qint64 m_lastPeakTime;                      // 上个峰值时间
    qint64 m_lastValleyTime;                    // 上个谷值时间

    // 相位段历史
    std::deque<PhaseSegment> m_recentSegments;  // 最近的相位段

    // 自适应学习
    bool m_adaptiveEnabled;
    double m_baselineFrequency;                 // 基准频率
    uint16_t m_baselineAmplitude;               // 基准幅度
    double m_baselineRisingSlope;               // 基准上升斜率
    double m_baselineFallingSlope;              // 基准下降斜率
    double m_baselineAsymmetryRatio;            // 基准非对称比例
    int m_learningPeriod;                       // 学习周期
    int m_samplesProcessed;                     // 已处理样本数
    bool m_learningCompleted;                   // 学习是否完成

    // 记录控制
    bool m_isRecording;
    int m_recordingDuration;
    QTimer* m_recordingTimer;
    int m_recordedDataPoints;

    // 异常类型开关
    QMap<TriangleAnomalyType, bool> m_enabledAnomalies;

    // 性能优化
    int m_analysisCounter;
    static const int ANALYSIS_INTERVAL = 10;    // 每10个数据点执行一次分析
};

#endif // OPTIMIZEDTRIANGLEANOMALYDETECTOR_H
