#ifndef OPTIMIZEDTRIANGLEANOMALYDETECTOR_H
#define OPTIMIZEDTRIANGLEANOMALYDETECTOR_H

#include <QObject>
#include <QQueue>
#include <QTimer>
#include <QDebug>
#include <QDateTime>
#include <algorithm>
#include <cmath>
#include <QMutex>
#include <QPointF>
// 三角波相位枚举 - 保持不变
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
    PeriodAnomaly,         // 周期异常
    DataJumpAnomaly        // 【新增】数据跳变异常 - 检测相邻点的突变
};

// 三角波统计信息
struct TriangleStats {
    double avgPeakValue{0};        // 平均波峰值
    double avgValleyValue{0};      // 平均波谷值
    double avgRisingSlope{0};      // 平均上升斜率（线性拟合）
    double avgFallingSlope{0};     // 平均下降斜率（线性拟合）
    int avgPeriodLength{0};        // 平均周期长度
    double peakValueStdDev{0};     // 波峰值标准差
    double valleyValueStdDev{0};   // 波谷值标准差
    double risingSlopeStdDev{0};   // 上升斜率标准差
    double fallingSlopeStdDev{0};  // 下降斜率标准差
    int totalCycles{0};            // 总周期数
    bool isValid{false};           // 统计信息是否有效

    // 【新增】半周期相关 - 用于更精确的斜率计算和相位判断
    int avgRisingEdgeLength{0};    // 平均上升沿长度
    int avgFallingEdgeLength{0};   // 平均下降沿长度
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

    // 【新增】跳变异常相关 - 专门用于数据跳变检测
    uint16_t previousValue{0};     // 前一个值（用于跳变检测）
    double jumpMagnitude{0};       // 跳变幅度
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
                             double slopeThreshold = 20.0,
                             double periodThreshold = 0.3,
                             double jumpThreshold = 30.0,// 【新增参数】跳变阈值
                             int risePeriodThreshold = 100,
                             int fallPeriodThreshold = 100);

    // 状态查询
    bool isLearningComplete() const { return m_learningComplete; }
    TriangleStats getLearnedStats() const { return m_learnedStats; }
    TrianglePhase getCurrentPhase() const { return m_currentPhase; }

    // 调试信息(暂未使用)
    QString getDebugInfo() const;

signals:
    void anomalyDetected(const TriangleAnomalyResult& anomaly);
    void learningProgress(int current, int total);
    void learningCompleted(const TriangleStats& stats);

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

    // 【新增】半周期边沿信息结构 - 用于更精确的斜率计算
    struct EdgeInfo {
        int startIndex;
        int endIndex;
        uint16_t startValue;
        uint16_t endValue;
        double slope;          // 线性拟合斜率
        double correlation;    // 拟合相关系数
        bool isRising;         // true为上升沿，false为下降沿
    };

    // 【修改】周期信息结构 - 增加了详细的边沿信息
    struct CycleInfo {
        int startIndex;
        int endIndex;
        uint16_t peakValue;
        uint16_t valleyValue;
        EdgeInfo risingEdge;   // 【新增】上升沿信息
        EdgeInfo fallingEdge;  // 【新增】下降沿信息
        int periodLength;

        // 【删除但注释保留】原来简单的斜率计算
        // double risingSlope;    // 原来用两点计算的简单斜率
        // double fallingSlope;   // 原来用两点计算的简单斜率
        // 删除原因：用线性拟合的EdgeInfo替代，更准确
    };

    // 核心算法
    void findPeaksAndValleys();
    void analyzeCycles();
    void calculateStatistics();

    // 【新增】改进的斜率计算：使用线性拟合
    EdgeInfo calculateEdgeSlope(int startIdx, int endIdx, bool isRising) const;
    double linearRegression(const QVector<QPointF>& points, double& correlation) const;

    // 【删除但注释保留】原来简单的斜率计算
    // double calculateSlope(int startIdx, int endIdx) const;
    // 删除原因：用线性拟合方法替代两点斜率计算，提高准确性

    // 【新增】改进的相位判断
    TrianglePhase determineCurrentPhaseImproved(uint16_t currentValue) const;
    TrianglePhase analyzeRecentTrend(int windowSize = 20) const;

    // 【删除但注释保留】原来简单的相位判断
    // TrianglePhase determineCurrentPhase(uint16_t currentValue) const;
    // 删除原因：原方法只用3个点判断趋势，容易受噪声影响，用改进版替代

    // 异常检测
    void checkForAnomalies(uint16_t value, qint64 timestamp);

    // 【新增】专门的跳变异常检测
    void checkDataJumpAnomaly(uint16_t currentValue, uint16_t previousValue, qint64 timestamp);


    TriangleAnomalyResult createAnomalyResult(TriangleAnomalyType type,
                                            double severity,
                                            uint16_t triggerValue,
                                            qint64 timestamp,
                                            const QString& description,
                                            double expectedValue,
                                            double actualValue) const;

    // 数据管理
    QQueue<DataPoint> m_learningData;      // 学习阶段数据
    QList<PeakValleyPoint> m_peaksValleys; // 波峰波谷点
    QList<CycleInfo> m_cycles;             // 周期信息

    // 学习阶段参数
    int m_learningDataCount{20000};         // 学习数据点数
    bool m_learningComplete{false};        // 学习是否完成
    TriangleStats m_learnedStats;          // 学习到的统计信息

    // 实时检测参数
    TrianglePhase m_currentPhase{TrianglePhase::Unknown};

    // 【新增】改进的相位跟踪
    TrianglePhase m_lastConfidentPhase{TrianglePhase::Unknown}; // 最后确定的相位

    // 【新增】半周期检测相关 - 用于实时斜率监测
    QQueue<DataPoint> m_currentEdgeData;   // 当前半周期数据
    bool m_inRisingEdge{false};            // 是否在上升沿
    bool m_inFallingEdge{false};           // 是否在下降沿
    int m_edgeStartIndex{-1};              // 当前边沿开始索引


    QQueue<DataPoint> m_recentData;        // 最近的数据点（用于实时分析）
    int m_recentDataSize{50};              // 【修改】从100改为50，减少内存占用

    // 【极简】半周期跟踪
    enum class TrackingState {
        None,        // 不在跟踪
        FromPeak,    // 从波峰开始跟踪（收集下降沿数据）
        FromValley   // 从波谷开始跟踪（收集上升沿数据）
    };
    //半周期斜率计算
    void processHalfCycle(const DataPoint& point);
    void analyzeHalfCycleSlope(TrianglePhase edgeType);
    void analyzeAnomalyCycle(bool isRise);//检测半周期是否异常

    TrackingState m_trackingState{TrackingState::None};
    QQueue<DataPoint> m_halfCycleQueue;  // 当前半周期的数据点队列


    // 异常检测阈值
    double m_peakAnomalyThreshold{3.0};    // 波峰异常阈值
    double m_valleyAnomalyThreshold{3.0};  // 波谷异常阈值
    double m_slopeAnomalyThreshold{2.0};   // 斜率异常阈值
    double m_periodAnomalyThreshold{0.3};  // 周期异常阈值 (比例)
    double m_jumpAnomalyThreshold{10.0};   // 【新增】跳变异常阈值
    double m_risePeriodAnomalyThreshold{100.0};   // 上升周期点数异常阈值
    double m_fallPeriodAnomalyThreshold{100.0};   // 下降周期点数异常阈值


    // 状态跟踪
    int m_totalDataPoints{0};              // 总数据点数
    qint64 m_lastAnomalyTime{0};           // 上次异常检测时间
    uint16_t m_lastValue{0};               // 【新增】上一个数据值 - 用于跳变检测
    static constexpr int MIN_ANOMALY_INTERVAL = 20000000; // 【修改】在检测到一个异常后在接下来的MIN_ANOMALY_INTERVAL ms内不会报告新的异常

    // 调试用
    mutable QMutex m_mutex;
};

#endif // OPTIMIZEDTRIANGLEANOMALYDETECTOR_H

