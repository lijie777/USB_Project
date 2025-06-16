#ifndef OPTIMIZEDTRIANGLEANOMALYDETECTOR_H
#define OPTIMIZEDTRIANGLEANOMALYDETECTOR_H

#include <QMutex>
#include <QObject>
#include <QQueue>
#include <QTimer>
#include <QDebug>
#include <QDateTime>
#include <algorithm>
#include <cmath>

// 三角波相位枚举
enum class TrianglePhase {
    Unknown,
    Rising,      // 上升阶段
    Falling,     // 下降阶段
    AtPeak,      // 波峰
    AtValley     // 波谷
};

// 异常类型枚举
enum class TriangleAnomalyType {
    None,
    RisingSlopeAnomaly,    // 上升斜率异常
    FallingSlopeAnomaly,   // 下降斜率异常
    PeakValueAnomaly,      // 波峰值异常
    ValleyValueAnomaly,    // 波谷值异常
    PeriodAnomaly          // 周期异常
};

// 三角波统计信息
struct TriangleStats {
    double avgPeakValue{0};        // 平均波峰值
    double avgValleyValue{0};      // 平均波谷值
    double avgRisingSlope{0};      // 平均上升斜率
    double avgFallingSlope{0};     // 平均下降斜率
    int avgPeriodLength{0};        // 平均周期长度
    double peakValueStdDev{0};     // 波峰值标准差
    double valleyValueStdDev{0};   // 波谷值标准差
    double risingSlopeStdDev{0};   // 上升斜率标准差
    double fallingSlopeStdDev{0};  // 下降斜率标准差
    int totalCycles{0};            // 总周期数
    bool isValid{false};           // 统计信息是否有效
};

// 异常检测结果
struct TriangleAnomalyResult {
    TriangleAnomalyType type{TriangleAnomalyType::None};
    double severity{0.0};          // 异常严重程度 (0-1)
    uint16_t triggerValue{0};      // 触发异常的值
    qint64 timestamp{0};           // 时间戳
    TrianglePhase phaseWhenDetected{TrianglePhase::Unknown}; // 检测到异常时的相位
    QString description;           // 异常描述
    double expectedValue{0};       // 期望值
    double actualValue{0};         // 实际值
    double threshold{0};           // 阈值
};

class OptimizedTriangleAnomalyDetector : public QObject
{
    Q_OBJECT

public:
    explicit OptimizedTriangleAnomalyDetector(QObject *parent = nullptr);
    ~OptimizedTriangleAnomalyDetector();

    // 主要接口
    void addDataPoint(uint16_t value, qint64 timestamp = 0);
    void reset();
    void setLearningDataCount(int count) { m_learningDataCount = count; }

    // 配置参数
    void setAnomalyThresholds(double peakThreshold = 3.0,
                             double valleyThreshold = 3.0,
                             double slopeThreshold = 2.0,
                             double periodThreshold = 0.3);

    // 状态查询
    bool isLearningComplete() const { return m_learningComplete; }
    TriangleStats getLearnedStats() const { return m_learnedStats; }
    TrianglePhase getCurrentPhase() const { return m_currentPhase; }

    // 调试信息
    QString getDebugInfo() const;

signals:
    void anomalyDetected(const TriangleAnomalyResult& anomaly);
    void learningProgress(int current, int total);
    void learningCompleted(const TriangleStats& stats);
    void statsUpdated(const TriangleStats& stats);

private slots:
    void processLearningData();

private:
    // 数据结构
    struct DataPoint {
        uint16_t value;
        qint64 timestamp;
        int index;
    };

    struct PeakValleyPoint {
        uint16_t value;
        int index;
        bool isPeak;  // true为波峰，false为波谷
    };

    struct CycleInfo {
        int startIndex;
        int endIndex;
        uint16_t peakValue;
        uint16_t valleyValue;
        double risingSlope; //上升沿
        double fallingSlope;//下降沿
        int periodLength;
    };

    // 核心算法
    void findPeaksAndValleys();
    void analyzeCycles();
    void calculateStatistics();
    double calculateSlope(int startIdx, int endIdx) const;//斜率计算
    TrianglePhase determineCurrentPhase(uint16_t currentValue) const;

    // 异常检测
    void checkForAnomalies(uint16_t value, qint64 timestamp);
    TriangleAnomalyResult createAnomalyResult(TriangleAnomalyType type,
                                            double severity,
                                            uint16_t triggerValue,
                                            qint64 timestamp,
                                            const QString& description,
                                            double expectedValue,
                                            double actualValue,
                                            double threshold) const;

    // 数据管理
    QQueue<DataPoint> m_learningData;      // 学习阶段数据
    QQueue<DataPoint> m_recentData;        // 最近的数据点（用于实时分析）
    QList<PeakValleyPoint> m_peaksValleys; // 波峰波谷点
    QList<CycleInfo> m_cycles;             // 周期信息

    // 学习阶段参数
    int m_learningDataCount{15000};         // 学习数据点数
    bool m_learningComplete{false};        // 学习是否完成
    TriangleStats m_learnedStats;          // 学习到的统计信息

    // 实时检测参数
    TrianglePhase m_currentPhase{TrianglePhase::Unknown};
    int m_currentCycleStartIndex{-1};      // 当前周期开始索引
    uint16_t m_lastPeak{0};                // 最近的波峰值
    uint16_t m_lastValley{0};              // 最近的波谷值
    int m_recentDataSize{100};             // 保持最近数据的大小

    // 异常检测阈值 (标准差倍数)
    double m_peakAnomalyThreshold{3.0};    // 波峰异常阈值
    double m_valleyAnomalyThreshold{3.0};  // 波谷异常阈值
    double m_slopeAnomalyThreshold{2.0};   // 斜率异常阈值
    double m_periodAnomalyThreshold{0.3};  // 周期异常阈值 (比例)

    // 状态跟踪
    int m_totalDataPoints{0};              // 总数据点数
    qint64 m_lastAnomalyTime{0};           // 上次异常检测时间
    static constexpr int MIN_ANOMALY_INTERVAL = 1000; // 最小异常间隔(ms)

    // 调试用
    mutable QMutex m_mutex;
};

#endif // OPTIMIZEDTRIANGLEANOMALYDETECTOR_H
