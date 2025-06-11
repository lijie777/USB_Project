// OptimizedTriangleAnomalyDetector.h
// 简化版三角波异常检测算法 - 只检测斜率、波峰波谷、周期异常

#ifndef OPTIMIZEDTRIANGLEANOMALYDETECTOR_H
#define OPTIMIZEDTRIANGLEANOMALYDETECTOR_H

#include <QObject>
#include <QQueue>
#include <QTimer>
#include <QDateTime>
#include <QDebug>
#include <deque>
#include <functional>

// =============================================================================
// 宏定义区域 - 所有参数都通过宏定义，便于调试和优化
// =============================================================================

// 4KB/s情况下 一个周期的数据采样大概是1000个，可以参考波形的一个周期的X轴长度

// 采样相关参数
#define TRIANGLE_SAMPLING_RATE              2048.0      // 采样率(Hz) - 根据USB传输速率计算得出  比如是4KB/s  那么就是 4 * 1024 / 2,因为是两个字节一组数据，所以是2048
#define TRIANGLE_ANALYSIS_INTERVAL          10           // 数据分析间隔(个数) - 每10个数据点执行一次分析，平衡性能和实时性
#define TRIANGLE_DATA_BUFFER_SECONDS        3           // 数据缓冲区时长(秒) - 保存3秒的历史数据用于分析

// 学习阶段参数
#define TRIANGLE_LEARNING_CYCLES            10          // 学习周期数 - 分析前10个完整周期来建立基准参数
#define TRIANGLE_MIN_SAMPLES_PER_CYCLE      1000          // 每周期最少样本数 - 确保有足够数据点分析一个周期
#define TRIANGLE_MAX_SAMPLES_PER_CYCLE      1200        // 每周期最多样本数 - 防止异常长周期导致缓冲区溢出

// 相位检测参数
#define TRIANGLE_PHASE_WINDOW_SIZE          30           // 相位判断窗口大小(个数) - 用30个连续点的趋势判断当前相位
#define TRIANGLE_SLOPE_THRESHOLD            10.0        // 斜率阈值 - 斜率绝对值小于10认为是峰值/谷值，大于20认为是上升/下降
#define TRIANGLE_STRONG_SLOPE_THRESHOLD     20.0        // 强斜率阈值 - 明确判断上升/下降阶段的斜率门限
#define TRIANGLE_PEAK_VALLEY_RATIO          0.9         // 峰值/谷值判断比例 - 超过最大值90%认为接近峰值，低于范围10%认为接近谷值

// 异常检测容差参数
#define TRIANGLE_RISING_SLOPE_TOLERANCE     0.20        // 上升斜率容差(20%) - 上升斜率偏差超过20%触发异常
#define TRIANGLE_FALLING_SLOPE_TOLERANCE    0.20        // 下降斜率容差(20%) - 下降斜率偏差超过20%触发异常
#define TRIANGLE_PEAK_VALUE_TOLERANCE       0.15        // 波峰值容差(15%) - 波峰值偏差超过15%触发异常
#define TRIANGLE_VALLEY_VALUE_TOLERANCE     0.15        // 波谷值容差(15%) - 波谷值偏差超过15%触发异常
#define TRIANGLE_PERIOD_TOLERANCE           0.25        // 周期容差(25%) - 周期时长偏差超过25%触发异常

// 异常记录参数
#define TRIANGLE_RECORDING_DURATION         10          // 异常记录时长(秒) - 检测到异常后记录10秒数据
#define TRIANGLE_MIN_ANOMALY_SEVERITY       0.3         // 最小异常严重度 - 严重度低于30%不触发记录
#define TRIANGLE_SEVERE_ANOMALY_THRESHOLD   0.7         // 严重异常阈值 - 严重度超过70%显示警告对话框

// 数据有效性参数
#define TRIANGLE_MIN_AMPLITUDE              100         // 最小有效幅度 - 小于100的幅度认为信号太弱
#define TRIANGLE_MAX_NOISE_LEVEL            50.0        // 最大噪声水平 - 噪声超过50认为信号质量差
#define TRIANGLE_MIN_CYCLE_DURATION_MS      10          // 最短周期时长(毫秒) - 短于10ms的周期认为无效
#define TRIANGLE_MAX_CYCLE_DURATION_MS      10000       // 最长周期时长(毫秒) - 长于10秒的周期认为无效

// 统计历史长度参数
#define TRIANGLE_SLOPE_HISTORY_SIZE         20          // 斜率历史记录数 - 保存最近20个周期的斜率用于计算基准
#define TRIANGLE_PEAK_VALLEY_HISTORY_SIZE   20          // 峰谷历史记录数 - 保存最近20个周期的峰谷值
#define TRIANGLE_PERIOD_HISTORY_SIZE        20          // 周期历史记录数 - 保存最近20个周期的时长

// 在头文件中添加新的宏定义
#define TRIANGLE_INITIAL_STABILIZATION_SAMPLES  1000     // 初始稳定期样本数 - 前200个样本用于稳定相位识别
#define TRIANGLE_MIN_VALID_CYCLES_FOR_LEARNING  10       // 学习阶段最少有效周期数 - 至少5个完整有效周期才开始计算基准
#define TRIANGLE_CYCLE_VALIDATION_STRICT        true    // 严格周期验证 - 学习阶段使用更严格的周期验证
#define TRIANGLE_PHASE_CONFIDENCE_THRESHOLD     30       // 相位识别置信度阈值 - 连续3次相同相位才确认
#define TRIANGLE_INITIAL_RANGE_DETECTION_WINDOW 1000     // 初始范围检测窗口 - 用前100个样本估算数据范围

// =============================================================================
// 枚举定义 - 简化的异常类型和相位状态
// =============================================================================

// 简化的三角波异常类型 - 只关注核心参数
enum class TriangleAnomalyType {
    None = 0,                   // 无异常
    RisingSlopeAnomaly,         // 上升斜率异常
    FallingSlopeAnomaly,        // 下降斜率异常
    PeakValueAnomaly,           // 波峰值异常
    ValleyValueAnomaly,         // 波谷值异常
    PeriodAnomaly               // 周期异常
};

// 三角波相位状态
enum class TrianglePhase {
    Rising,                     // 上升阶段
    Falling,                    // 下降阶段
    AtPeak,                     // 波峰附近
    AtValley,                   // 波谷附近
    Unknown                     // 未知阶段
};

// =============================================================================
// 数据结构定义
// =============================================================================

// 异常检测结果
struct TriangleAnomalyResult {
    TriangleAnomalyType type;           // 异常类型
    double severity;                    // 严重程度 0.0-1.0
    uint16_t triggerValue;              // 触发异常时的数值
    qint64 timestamp;                   // 异常发生时间戳
    QString description;                // 异常描述信息
    TrianglePhase phaseWhenDetected;    // 检测到异常时的相位

    TriangleAnomalyResult() : type(TriangleAnomalyType::None), severity(0.0),
                             triggerValue(0), timestamp(0), phaseWhenDetected(TrianglePhase::Unknown) {}
};

// 三角波统计参数
struct TriangleStats {
    // 基本统计
    double currentFrequency;            // 当前频率(Hz)
    double baselineFrequency;           // 基准频率(Hz)
    qint64 currentPeriodMs;             // 当前周期时长(毫秒)
    qint64 baselinePeriodMs;            // 基准周期时长(毫秒)

    // 斜率统计
    double currentRisingSlope;          // 当前上升斜率
    double currentFallingSlope;         // 当前下降斜率
    double baselineRisingSlope;         // 基准上升斜率
    double baselineFallingSlope;        // 基准下降斜率

    // 峰谷统计
    uint16_t currentPeakValue;          // 当前波峰值
    uint16_t currentValleyValue;        // 当前波谷值
    uint16_t baselinePeakValue;         // 基准波峰值
    uint16_t baselineValleyValue;       // 基准波谷值

    // 状态信息
    TrianglePhase currentPhase;         // 当前相位
    int completedCycles;                // 已完成周期数
    double noiseLevel;                  // 噪声水平
    bool isLearning;                    // 是否在学习阶段

    TriangleStats() : currentFrequency(0), baselineFrequency(0), currentPeriodMs(0), baselinePeriodMs(0),
                     currentRisingSlope(0), currentFallingSlope(0), baselineRisingSlope(0), baselineFallingSlope(0),
                     currentPeakValue(0), currentValleyValue(0), baselinePeakValue(0), baselineValleyValue(0),
                     currentPhase(TrianglePhase::Unknown), completedCycles(0), noiseLevel(0), isLearning(true) {}
};

struct CycleData {
    qint64 startTime;                   // 周期开始时间
    qint64 endTime;                     // 周期结束时间
    qint64 duration;                    // 周期持续时间(毫秒)
    qint64 peakTime;                    // 波峰时间 - 新增
    qint64 valleyTime;                  // 波谷时间 - 新增
    qint64 risingDuration;              // 上升持续时间 - 新增
    qint64 fallingDuration;             // 下降持续时间 - 新增
    double risingSlope;                 // 上升斜率
    double fallingSlope;                // 下降斜率
    uint16_t peakValue;                 // 波峰值
    uint16_t valleyValue;               // 波谷值
    bool isValid;                       // 周期是否有效

    CycleData() : startTime(0), endTime(0), duration(0), peakTime(0), valleyTime(0),
                 risingDuration(0), fallingDuration(0), risingSlope(0), fallingSlope(0),
                 peakValue(0), valleyValue(0), isValid(false) {}
};

// =============================================================================
// 主类定义
// =============================================================================

class OptimizedTriangleAnomalyDetector : public QObject
{
    Q_OBJECT

public:
    explicit OptimizedTriangleAnomalyDetector(QObject *parent = nullptr);
    ~OptimizedTriangleAnomalyDetector();

    // 主要接口
    void feedData(uint16_t value);
    void setOptimalParameters();
    void reset();

    // 状态查询
    bool isRecording() const { return m_isRecording; }
    bool isLearningComplete() const { return !m_currentStats.isLearning; }
    TriangleStats getCurrentStats() const { return m_currentStats; }
    TriangleAnomalyResult getLastAnomaly() const { return m_lastAnomaly; }

    // 参数配置
    void setSamplingRate(double sampleRate) { m_samplingRate = sampleRate; }
    void setRecordingDuration(int seconds) { m_recordingDuration = seconds; }

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
    void analyzeCurrentData();
    void detectPhaseTransition();
    void processCycleCompletion();
    TriangleAnomalyResult checkForAnomalies();

    // 异常检测函数
    bool checkRisingSlopeAnomaly(double& severity);
    bool checkFallingSlopeAnomaly(double& severity);
    bool checkPeakValueAnomaly(double& severity);
    bool checkValleyValueAnomaly(double& severity);
    bool checkPeriodAnomaly(double& severity);

    // 学习算法
    void updateLearningData();
    void completeLearningPhase();

    // 辅助函数
    TrianglePhase determineCurrentPhase();
    double calculateSlope(int startIdx, int endIdx);
    double calculateNoiseLevel();
    bool isValidCycle(const CycleData& cycle);
    void updateStatistics();

    void handleInitialStabilization();
    void estimateDataRange();

private:
    // 相位转换处理
    void processPhaseTransition(TrianglePhase newPhase);
    void analyzeCompletedPhaseSegment(TrianglePhase phase, int startIdx, int endIdx);

    // 周期管理
    void startNewCycle(qint64 currentTime);
    void recordPeakValue(uint16_t peakValue, qint64 peakTime);
    void recordValleyValue(uint16_t valleyValue, qint64 valleyTime);
    void completeCycle(qint64 currentTime);
    void updateCurrentStatistics();
private:
    // 数据缓冲区
    std::deque<uint16_t> m_dataBuffer;      // 原始数据缓冲区
    std::deque<qint64> m_timestampBuffer;   // 时间戳缓冲区

    // 学习阶段数据
    std::deque<CycleData> m_learningCycles; // 学习阶段的周期数据
    std::deque<double> m_risingSlopeHistory; // 上升斜率历史
    std::deque<double> m_fallingSlopeHistory; // 下降斜率历史
    std::deque<uint16_t> m_peakHistory;     // 波峰历史
    std::deque<uint16_t> m_valleyHistory;   // 波谷历史
    std::deque<qint64> m_periodHistory;     // 周期历史

    // 当前状态
    TriangleStats m_currentStats;           // 当前统计信息
    TriangleAnomalyResult m_lastAnomaly;    // 最后检测到的异常

    // 相位检测
    TrianglePhase m_previousPhase;          // 前一个相位
    int m_phaseStartIndex;                  // 当前相位开始索引
    qint64 m_phaseStartTime;                // 当前相位开始时间
    qint64 m_cycleStartTime;                // 当前周期开始时间

    // 系统参数
    double m_samplingRate;                  // 采样率
    int m_recordingDuration;                // 记录持续时间
    int m_analysisCounter;                  // 分析计数器
    int m_samplesProcessed;                 // 已处理样本数

    // 记录控制
    bool m_isRecording;                     // 是否正在记录
    QTimer* m_recordingTimer;               // 记录定时器
    int m_recordedDataPoints;               // 已记录数据点数

    // 当前周期数据
    CycleData m_currentCycle;               // 当前正在分析的周期


    // 在类中添加新的成员变量
private:
    // 初始化状态控制
    bool m_isInitialStabilizationComplete;      // 初始稳定期是否完成
    int m_stablePhaseCount;                     // 稳定相位计数
    TrianglePhase m_lastConfirmedPhase;         // 最后确认的相位

    // 数据范围估算
    uint16_t m_estimatedMin;                    // 估算的最小值
    uint16_t m_estimatedMax;                    // 估算的最大值
    bool m_rangeEstimationComplete;             // 范围估算是否完成

    // 周期有效性统计
    int m_validCyclesCount;                     // 有效周期计数
    int m_totalAttemptedCycles;                 // 尝试的总周期数
};

#endif // OPTIMIZEDTRIANGLEANOMALYDETECTOR_H

