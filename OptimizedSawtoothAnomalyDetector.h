// OptimizedSawtoothAnomalyDetector.h
// 针对规整锯齿波/三角波的优化异常检测算法

#ifndef OPTIMIZEDSAWTOOTHANOMALYDETECTOR_H
#define OPTIMIZEDSAWTOOTHANOMALYDETECTOR_H

#include <QObject>
#include <QQueue>
#include <QTimer>
#include <QDateTime>
#include <QDebug>
#include <deque>
#include <functional>


#define NOMINALFREQUENCY 3.76   // 锯齿波频率，由于每次运行可能不同，应该设置为0，让程序自动学习
#define FREQUENCYTOLERANCE 0.05 // 频率容差，保持5%，对于规律信号这个容差足够

#define EXPECTEDMIN 0           // 锯齿波的期望值范围，0-4095（uint16_t后12位）需要根据实际数据调整（y轴的最小值）
#define EXPECTEDMAX 4095       // 需要根据实际数据调整（y轴的最大值）
#define AMPLITUDETOLERANCE 0.1  // 允许的幅度变化百分比，如果信号稳定，可以减小到5%；如果有噪声，保持10%
#define BASHLINEAMPLITUDE 0     // 应该设置为0，让程序自动学习

#define SAMPLINGRATE 10000.0    // 假设10kHz采样率，需要调整
#define LINEARTTHREASHOLD 0.9
#define LEARNINGPERIOD 1000     // 学习1000个样本


// 锯齿波特定的异常类型
enum class SawtoothAnomalyType {
    None = 0,
    FrequencyDrift,       // 频率漂移 (从3.76Hz偏离)
    AmplitudeDrift,       // 幅度变化
    LinearityError,       // 线性度误差 (不再是直线)
    PhaseJump,            // 相位突跳
    MissingEdge,          // 缺失上升/下降沿
    NoiseSpike,           // 噪声尖峰
    Saturation,           // 波形削顶
    Dropout               // 信号丢失
};

// 锯齿波状态
enum class SawtoothPhase {
    Rising,               // 上升阶段
    Falling,              // 下降阶段
    Unknown               // 未知阶段
};

// 异常检测结果
struct SawtoothAnomalyResult {
    SawtoothAnomalyType type;
    double severity;                    // 严重程度 0.0-1.0
    uint16_t triggerValue;
    qint64 timestamp;
    QString description;
    SawtoothPhase phaseWhenDetected;    // 发生异常时的波形阶段

    SawtoothAnomalyResult() : type(SawtoothAnomalyType::None), severity(0.0),
                             triggerValue(0), timestamp(0), phaseWhenDetected(SawtoothPhase::Unknown) {}
};

// 锯齿波统计参数
struct SawtoothStats {
    double currentFrequency;            // 当前频率
    double averageFrequency;            // 平均频率
    double frequencyStability;          // 频率稳定性
    uint16_t currentMin;                // 当前周期最小值
    uint16_t currentMax;                // 当前周期最大值
    uint16_t averageAmplitude;          // 平均幅度
    double linearity;                   // 线性度 (0-1, 1为完美直线)
    SawtoothPhase currentPhase;         // 当前阶段
    int cycleCount;                     // 周期计数
    double noiseLevel;                  // 噪声水平

    SawtoothStats() : currentFrequency(0), averageFrequency(0), frequencyStability(0),
                     currentMin(65535), currentMax(0), averageAmplitude(0),
                     linearity(1.0), currentPhase(SawtoothPhase::Unknown),
                     cycleCount(0), noiseLevel(0) {}
};

class OptimizedSawtoothAnomalyDetector : public QObject
{
    Q_OBJECT

public:
    explicit OptimizedSawtoothAnomalyDetector(QObject *parent = nullptr);
    ~OptimizedSawtoothAnomalyDetector();

    // 主要接口
    void feedData(uint16_t value);

    void setOptimalParameters();

    void setNominalFrequency(double frequency);          // 设置标称频率 (默认3.76Hz)
    void setExpectedAmplitude(uint16_t minVal, uint16_t maxVal);
    void setDetectionThresholds(double freqTolerance = 0.05,    // 频率容差 5%
                               double ampTolerance = 0.1,       // 幅度容差 10%
                               double linearityThreshold = 0.9); // 线性度阈值
    void setRecordingDuration(int seconds = 10);
    void reset();

    // 状态查询
    bool isRecording() const { return m_isRecording; }
    SawtoothStats getCurrentStats() const { return m_currentStats; }
    SawtoothAnomalyResult getLastAnomaly() const { return m_lastAnomaly; }

    // 高级配置
    void enableAnomalyType(SawtoothAnomalyType type, bool enabled = true);
    void setSamplingRate(double sampleRate);             // 设置采样率
    void setAdaptiveThresholds(bool enabled = true);     // 自适应阈值

signals:
    void anomalyDetected(const SawtoothAnomalyResult& anomaly);
    void recordingStarted(const SawtoothAnomalyResult& trigger);
    void recordingData(uint16_t value, qint64 timestamp);
    void recordingStopped(int totalDataPoints);
    void statsUpdated(const SawtoothStats& stats);

private slots:
    void onRecordingTimeout();

private:
    // 核心检测算法
    SawtoothAnomalyResult analyzeSawtooth();
    bool detectFrequencyDrift(double& severity);
    bool detectAmplitudeDrift(double& severity);
    bool detectLinearityError(double& severity);
    bool detectPhaseJump(double& severity);
    bool detectMissingEdge(double& severity);
    bool detectNoiseSpike(double& severity);
    bool detectSaturation(double& severity);
    bool detectDropout(double& severity);

    // 锯齿波分析
    void updateSawtoothStats();
    void detectPhaseTransition();
    double calculateCurrentFrequency();
    double calculateLinearity();
    void findCycleMinMax();
    bool isValidSawtoothCycle();

    // 自适应学习
    void updateBaseline();
    void adaptThresholds();

    // 辅助函数
    double calculateSlope(int startIdx, int endIdx);
    bool isMonotonic(int startIdx, int endIdx, bool shouldIncrease);
    double calculateCorrelation(const std::vector<double>& x, const std::vector<double>& y);

private:
    // 数据缓冲区
    std::deque<uint16_t> m_dataBuffer;      // 数据缓冲区
    std::deque<qint64> m_timestampBuffer;   // 时间戳缓冲区
    std::deque<double> m_frequencyHistory;  // 频率历史
    std::deque<uint16_t> m_amplitudeHistory;// 幅度历史

    // 锯齿波参数
    double m_nominalFrequency;              // 标称频率 (3.76Hz)
    uint16_t m_expectedMin;                 // 期望最小值
    uint16_t m_expectedMax;                 // 期望最大值
    double m_samplingRate;                  // 采样率

    // 检测阈值
    double m_frequencyTolerance;            // 频率容差
    double m_amplitudeTolerance;            // 幅度容差
    double m_linearityThreshold;            // 线性度阈值

    // 当前状态
    SawtoothStats m_currentStats;       // 锯齿波当前状态
    SawtoothAnomalyResult m_lastAnomaly;// 异常检测结果

    // 阶段检测
    SawtoothPhase m_previousPhase;
    int m_phaseStartIndex;                  // 当前阶段开始索引
    uint16_t m_lastPeakValue;               // 上个峰值
    uint16_t m_lastValleyValue;             // 上个谷值
    qint64 m_lastTransitionTime;            // 上次相位转换时间

    // 自适应学习
    bool m_adaptiveEnabled;
    double m_baselineFrequency;             // 基准频率
    uint16_t m_baselineAmplitude;           // 基准幅度
    int m_learningPeriod;                   // 学习周期
    int m_samplesProcessed;                 // 已处理样本数

    // 记录控制
    bool m_isRecording;
    int m_recordingDuration;
    QTimer* m_recordingTimer;
    int m_recordedDataPoints;

    // 异常类型开关
    QMap<SawtoothAnomalyType, bool> m_enabledAnomalies; // 控制检测哪些类型的异常，默认值：全部启用，可以根据实际需求禁用不关心的异常类型

    // 性能优化
    int m_analysisCounter;
    static const int ANALYSIS_INTERVAL = 20;    // 作用：每20个数据点执行一次分析,建议：高性能系统：10-20；低性能系统：50-100
};
#endif
