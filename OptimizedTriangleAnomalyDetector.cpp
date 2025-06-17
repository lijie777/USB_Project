
#include "OptimizedTriangleAnomalyDetector.h"
#include <QMutexLocker>
#include <numeric>

OptimizedTriangleAnomalyDetector::OptimizedTriangleAnomalyDetector(QObject *parent)
    : QObject(parent)
{
    qDebug() << "三角波异常检测器初始化";
}

OptimizedTriangleAnomalyDetector::~OptimizedTriangleAnomalyDetector()
{
    qDebug() << "三角波异常检测器销毁";
}

void OptimizedTriangleAnomalyDetector::addDataPoint(uint16_t value, qint64 timestamp)
{
    QMutexLocker locker(&m_mutex);
    if (timestamp == 0) {
        timestamp = QDateTime::currentMSecsSinceEpoch();
    }

    DataPoint point{value, timestamp, m_totalDataPoints++};

    // 检测数据跳变异常（在学习阶段也需要检测）
    if (m_totalDataPoints > 1) {
        checkDataJumpAnomaly(value, m_lastValue, timestamp);
    }
    m_lastValue = value;

    if (!m_learningComplete) {
        // 学习阶段
        m_learningData.enqueue(point);
        emit learningProgress(m_learningData.size(), m_learningDataCount);

        if (m_learningData.size() >= m_learningDataCount) {
            processLearningData();
        }
    } else {
        // 实时检测阶段
        m_recentData.enqueue(point);

        // 保持最近数据队列大小
        while (m_recentData.size() > m_recentDataSize) {
            m_recentData.dequeue();
        }

        // 更新当前相位
        m_currentPhase = determineCurrentPhaseImproved(value);

        // 检测其他异常
        checkForAnomalies(value, timestamp);

        // 检测半周期斜率异常
        checkSlopeAnomalyInRealTime();
    }

}

void OptimizedTriangleAnomalyDetector::processLearningData()
{
    qDebug() << "开始学习三角波特征，数据点数：" << m_learningData.size();

    //波峰波谷计算
    findPeaksAndValleys();

    //周期分析
    analyzeCycles();

    //统计
    calculateStatistics();

    //至少要保证两个周期以上
    if (m_learnedStats.isValid && m_learnedStats.totalCycles >= 2) {
        m_learningComplete = true;
        qDebug() << "学习完成！检测到" << m_learnedStats.totalCycles << "个完整周期";
        qDebug() << QString("平均周期长度: %1, 波峰: %2±%3, 波谷: %4±%5, 上升斜率: %6，下降斜率: %7")
                    .arg(m_learnedStats.avgPeriodLength)
                    .arg(m_learnedStats.avgPeakValue, 0, 'f', 1)
                    .arg(m_learnedStats.peakValueStdDev, 0, 'f', 1)
                    .arg(m_learnedStats.avgValleyValue, 0, 'f', 1)
                    .arg(m_learnedStats.valleyValueStdDev, 0, 'f', 1)
                    .arg(m_learnedStats.avgRisingSlope, 0, 'f', 1)
                    .arg(m_learnedStats.avgFallingSlope, 0, 'f', 1);

        emit learningCompleted(m_learnedStats);
    } else {
        qDebug() << "学习失败，未能识别有效的三角波特征";
        // 重置并要求更多数据
        m_learningDataCount += 500;
        qDebug() << "增加学习数据量至：" << m_learningDataCount;
    }
}

void OptimizedTriangleAnomalyDetector::findPeaksAndValleys()
{
    m_peaksValleys.clear();

    if (m_learningData.size() < 10) return;

    // 转换为数组便于处理
    QVector<uint16_t> values;
    for (const auto& point : m_learningData) {
        values.append(point.value);
    }

    // 使用滑动窗口找波峰波谷
    int windowSize = qMax(5, m_learningData.size() / 200); // 自适应窗口大小

    for (int i = windowSize; i < values.size() - windowSize; i++) {
        uint16_t currentValue = values[i];
        bool isPeak = true;
        bool isValley = true;

        // 检查是否为局部最大值（波峰）
        for (int j = i - windowSize; j <= i + windowSize; j++) {
            if (j != i && values[j] >= currentValue) {
                isPeak = false;
                break;
            }
        }

        // 检查是否为局部最小值（波谷）
        for (int j = i - windowSize; j <= i + windowSize; j++) {
            if (j != i && values[j] <= currentValue) {
                isValley = false;
                break;
            }
        }

        if (isPeak || isValley) {
            PeakValleyPoint point{currentValue, i, isPeak};
            m_peaksValleys.append(point);
        }
    }

    qDebug() << "找到" << m_peaksValleys.size() << "个波峰波谷点";
}

void OptimizedTriangleAnomalyDetector::analyzeCycles()
{
    m_cycles.clear();

    if (m_peaksValleys.size() < 4) return;

    // 按索引排序
    std::sort(m_peaksValleys.begin(), m_peaksValleys.end(),
              [](const PeakValleyPoint& a, const PeakValleyPoint& b) {
                  return a.index < b.index;
              });

    // 识别完整周期并分析每个半周期的斜率
    for (int i = 0; i < m_peaksValleys.size() - 3; i++) {
        // 寻找完整的周期：波谷->波峰->波谷 或 波峰->波谷->波峰
        if ((m_peaksValleys[i].isPeak == false &&
             m_peaksValleys[i+1].isPeak == true &&
             m_peaksValleys[i+2].isPeak == false) ||
            (m_peaksValleys[i].isPeak == true &&
             m_peaksValleys[i+1].isPeak == false &&
             m_peaksValleys[i+2].isPeak == true)) {

            CycleInfo cycle;
            cycle.startIndex = m_peaksValleys[i].index;
            cycle.endIndex = m_peaksValleys[i+2].index;
            cycle.periodLength = cycle.endIndex - cycle.startIndex;

            // 识别波峰和波谷
            if (m_peaksValleys[i+1].isPeak) {
                cycle.peakValue = m_peaksValleys[i+1].value;
                cycle.valleyValue = qMin(m_peaksValleys[i].value, m_peaksValleys[i+2].value);

                // 上升沿：从波谷到波峰
                cycle.risingEdge = calculateEdgeSlope(m_peaksValleys[i].index,
                                                     m_peaksValleys[i+1].index, true);
                // 下降沿：从波峰到波谷
                cycle.fallingEdge = calculateEdgeSlope(m_peaksValleys[i+1].index,
                                                      m_peaksValleys[i+2].index, false);
            } else {
                cycle.valleyValue = m_peaksValleys[i+1].value;
                cycle.peakValue = qMax(m_peaksValleys[i].value, m_peaksValleys[i+2].value);

                // 下降沿：从波峰到波谷
                cycle.fallingEdge = calculateEdgeSlope(m_peaksValleys[i].index,
                                                      m_peaksValleys[i+1].index, false);
                // 上升沿：从波谷到波峰
                cycle.risingEdge = calculateEdgeSlope(m_peaksValleys[i+1].index,
                                                     m_peaksValleys[i+2].index, true);
            }

            m_cycles.append(cycle);
        }
    }

    qDebug() << "识别到" << m_cycles.size() << "个完整周期，包含详细的半周期斜率信息";

}

// 更新calculateStatistics方法
void OptimizedTriangleAnomalyDetector::calculateStatistics()
{
    if (m_cycles.isEmpty()) {
        m_learnedStats.isValid = false;
        return;
    }

    // 收集统计数据
    QVector<double> peakValues, valleyValues, risingSlopes, fallingSlopes, periodLengths;
    QVector<int> risingEdgeLengths, fallingEdgeLengths;

    for (const auto& cycle : m_cycles) {
        peakValues.append(cycle.peakValue);
        valleyValues.append(cycle.valleyValue);
        periodLengths.append(cycle.periodLength);

        // 只统计拟合质量好的斜率
        if (std::abs(cycle.risingEdge.correlation) > 0.6) {
            risingSlopes.append(cycle.risingEdge.slope);
            risingEdgeLengths.append(cycle.risingEdge.endIndex - cycle.risingEdge.startIndex);
        }

        if (std::abs(cycle.fallingEdge.correlation) > 0.6) {
            fallingSlopes.append(cycle.fallingEdge.slope);
            fallingEdgeLengths.append(cycle.fallingEdge.endIndex - cycle.fallingEdge.startIndex);
        }
    }

    // 计算平均值和标准差的辅助函数
    auto calcMeanAndStdDev = [](const QVector<double>& values) -> QPair<double, double> {
        if (values.isEmpty()) return {0.0, 0.0};

        double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        double sum = 0;
        for (double val : values) {
            sum += (val - mean) * (val - mean);
        }
        double stdDev = std::sqrt(sum / values.size());
        return {mean, stdDev};
    };

    auto calcIntMean = [](const QVector<int>& values) -> int {
        if (values.isEmpty()) return 0;
        return std::accumulate(values.begin(), values.end(), 0) / values.size();
    };

    // 计算统计信息
    auto peakStats = calcMeanAndStdDev(peakValues);
    auto valleyStats = calcMeanAndStdDev(valleyValues);
    auto risingStats = calcMeanAndStdDev(risingSlopes);
    auto fallingStats = calcMeanAndStdDev(fallingSlopes);
    auto periodStats = calcMeanAndStdDev(periodLengths);

    m_learnedStats.avgPeakValue = peakStats.first;
    m_learnedStats.peakValueStdDev = peakStats.second;
    m_learnedStats.avgValleyValue = valleyStats.first;
    m_learnedStats.valleyValueStdDev = valleyStats.second;
    m_learnedStats.avgRisingSlope = risingStats.first;
    m_learnedStats.risingSlopeStdDev = risingStats.second;
    m_learnedStats.avgFallingSlope = fallingStats.first;
    m_learnedStats.fallingSlopeStdDev = fallingStats.second;
    m_learnedStats.avgPeriodLength = static_cast<int>(periodStats.first);

    // 半周期长度统计
    m_learnedStats.avgRisingEdgeLength = calcIntMean(risingEdgeLengths);
    m_learnedStats.avgFallingEdgeLength = calcIntMean(fallingEdgeLengths);

    m_learnedStats.totalCycles = m_cycles.size();
    m_learnedStats.isValid = (m_cycles.size() >= 2 &&
                             risingSlopes.size() >= 2 &&
                             fallingSlopes.size() >= 2);

    qDebug() << QString("学习统计完成: 波峰=%1±%2, 波谷=%3±%4, 上升斜率=%5±%6, 下降斜率=%7±%8")
                .arg(m_learnedStats.avgPeakValue, 0, 'f', 1).arg(m_learnedStats.peakValueStdDev, 0, 'f', 1)
                .arg(m_learnedStats.avgValleyValue, 0, 'f', 1).arg(m_learnedStats.valleyValueStdDev, 0, 'f', 1)
                .arg(m_learnedStats.avgRisingSlope, 0, 'f', 3).arg(m_learnedStats.risingSlopeStdDev, 0, 'f', 3)
                .arg(m_learnedStats.avgFallingSlope, 0, 'f', 3).arg(m_learnedStats.fallingSlopeStdDev, 0, 'f', 3);
}



// 改进的线性拟合斜率计算
OptimizedTriangleAnomalyDetector::EdgeInfo
OptimizedTriangleAnomalyDetector::calculateEdgeSlope(int startIdx, int endIdx, bool isRising) const
{
    EdgeInfo edge;
    edge.startIndex = startIdx;
    edge.endIndex = endIdx;
    edge.isRising = isRising;

    if (startIdx >= endIdx || startIdx < 0 || endIdx >= m_learningData.size()) {
        edge.slope = 0.0;
        edge.correlation = 0.0;
        return edge;
    }

    // 提取数据点
    QVector<QPointF> points;
    for (int i = startIdx; i <= endIdx; i++) {
        auto iter = m_learningData.begin() + i;
        points.append(QPointF(i - startIdx, iter->value));  // 相对索引作为x坐标
    }

    // 线性拟合
    edge.slope = linearRegression(points, edge.correlation);
    edge.startValue = (m_learningData.begin() + startIdx)->value;
    edge.endValue = (m_learningData.begin() + endIdx)->value;

    return edge;
}

// 线性回归实现
double OptimizedTriangleAnomalyDetector::linearRegression(const QVector<QPointF>& points, double& correlation) const
{
    if (points.size() < 2) {
        correlation = 0.0;
        return 0.0;
    }

    int n = points.size();
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0, sumY2 = 0;

    foreach (QPointF point , points) {
        sumX += point.x();
        sumY += point.y();
        sumXY += point.x() * point.y();
        sumX2 += point.x() * point.x();
        sumY2 += point.y() * point.y();
    }

    double meanX = sumX / n;
    double meanY = sumY / n;

    // 计算斜率 (最小二乘法)
    double denominator = sumX2 - n * meanX * meanX;
    double slope = 0.0;
    if (std::abs(denominator) > 1e-10) {
        slope = (sumXY - n * meanX * meanY) / denominator;
    }

    // 计算相关系数
    double numerator = n * sumXY - sumX * sumY;
    double denomX = n * sumX2 - sumX * sumX;
    double denomY = n * sumY2 - sumY * sumY;

    if (denomX > 0 && denomY > 0) {
        correlation = numerator / std::sqrt(denomX * denomY);
    } else {
        correlation = 0.0;
    }

    return slope;
}

// 改进的相位判断
TrianglePhase OptimizedTriangleAnomalyDetector::determineCurrentPhaseImproved(uint16_t currentValue) const
{
    if (!m_learnedStats.isValid) return TrianglePhase::Unknown;

    // 首先基于阈值判断是否在波峰或波谷附近
    double peakThreshold = m_learnedStats.avgPeakValue - m_learnedStats.peakValueStdDev * 0.5;
    double valleyThreshold = m_learnedStats.avgValleyValue + m_learnedStats.valleyValueStdDev * 0.5;

    if (currentValue >= peakThreshold) {
        return TrianglePhase::AtPeak;
    } else if (currentValue <= valleyThreshold) {
        return TrianglePhase::AtValley;
    } else {
        // 在中间区域，通过趋势分析判断上升还是下降
        return analyzeRecentTrend(20); // 使用20个点的窗口分析趋势
    }
}


void OptimizedTriangleAnomalyDetector::checkForAnomalies(uint16_t value, qint64 timestamp)
{
    //如果学习未成功则返回
    if (!m_learnedStats.isValid) return;

    // 防止异常检测过于频繁
    if (timestamp - m_lastAnomalyTime < MIN_ANOMALY_INTERVAL) {
        return;
    }

    // 确定当前相位
    m_currentPhase = determineCurrentPhaseImproved(value);

    // 波峰异常检测
    if (m_currentPhase == TrianglePhase::AtPeak) {
        //偏差 = 当前值波峰值 - 平均波峰值
        double deviation = std::abs(value - m_learnedStats.avgPeakValue);
        //波峰偏差阈值 = 偏差阈值倍数 * 波峰标准差
        double threshold = m_peakAnomalyThreshold * m_learnedStats.peakValueStdDev;

        if (deviation > threshold) {
            double severity = deviation / (threshold + 1e-6);
            auto anomaly = createAnomalyResult(
                TriangleAnomalyType::PeakValueAnomaly,
                qMin(1.0, severity),
                value,
                timestamp,
                QString("波峰值异常：实际=%1，期望=%2±%3")
                    .arg(value)
                    .arg(m_learnedStats.avgPeakValue, 0, 'f', 1)
                    .arg(threshold, 0, 'f', 1),
                m_learnedStats.avgPeakValue,
                value
            );

            m_lastAnomalyTime = timestamp;
            emit anomalyDetected(anomaly);
        }
    }

    // 波谷异常检测
    if (m_currentPhase == TrianglePhase::AtValley) {
        double deviation = std::abs(value - m_learnedStats.avgValleyValue);
        double threshold = m_valleyAnomalyThreshold * m_learnedStats.valleyValueStdDev;

        if (deviation > threshold) {
            double severity = deviation / (threshold + 1e-6);
            auto anomaly = createAnomalyResult(
                TriangleAnomalyType::ValleyValueAnomaly,
                qMin(1.0, severity),
                value,
                timestamp,
                QString("波谷值异常：实际=%1，期望=%2±%3")
                    .arg(value)
                    .arg(m_learnedStats.avgValleyValue, 0, 'f', 1)
                    .arg(threshold, 0, 'f', 1),
                m_learnedStats.avgValleyValue,
                value
            );

            m_lastAnomalyTime = timestamp;
            emit anomalyDetected(anomaly);
        }
    }

    // 周期异常检测（可选实现）
    // 注：周期异常检测比较复杂，需要跟踪完整周期，这里暂时注释
    /*
    if (m_currentPhase == TrianglePhase::AtPeak || m_currentPhase == TrianglePhase::AtValley) {
        // 检测周期长度是否异常
        // 实现逻辑待完善...
    }
    */
}
// 趋势分析
TrianglePhase OptimizedTriangleAnomalyDetector::analyzeRecentTrend(int windowSize) const
{
    if (m_recentData.size() < windowSize) {
        return m_lastConfidentPhase; // 数据不足时返回上次确定的相位
    }

    // 提取最近的数据点进行线性拟合
    QVector<QPointF> recentPoints;
    int startPos = m_recentData.size() - windowSize;

    for (int i = startPos; i < m_recentData.size(); i++) {
        auto iter = m_recentData.begin() + i;
        recentPoints.append(QPointF(i - startPos, iter->value));
    }

    double correlation;
    double slope = linearRegression(recentPoints, correlation);

    // 只有当相关性足够高时才相信拟合结果
    if (std::abs(correlation) > 0.7) { // 相关系数阈值
        if (slope > 0.1) { // 正斜率阈值
            return TrianglePhase::Rising;
        } else if (slope < -0.1) { // 负斜率阈值
            return TrianglePhase::Falling;
        }
    }

    return m_lastConfidentPhase; // 趋势不明显时返回上次确定的相位
}

// 实时斜率异常检测
void OptimizedTriangleAnomalyDetector::checkSlopeAnomalyInRealTime()
{
    if (!m_learnedStats.isValid || m_recentData.size() < 20) return;

    TrianglePhase currentPhase = m_currentPhase;

    // 检测相位转换，当发生转换时分析刚完成的半周期
    static TrianglePhase lastPhase = TrianglePhase::Unknown;

    if (lastPhase != TrianglePhase::Unknown && lastPhase != currentPhase) {
        // 发生了相位转换，分析上一个半周期的斜率
        if ((lastPhase == TrianglePhase::Rising && currentPhase == TrianglePhase::AtPeak) ||
            (lastPhase == TrianglePhase::Falling && currentPhase == TrianglePhase::AtValley)) {

            // 找到半周期的数据
            // 应该改为 - 根据实际的相位选择
            int edgeLength = (lastPhase == TrianglePhase::Rising) ?
                             m_learnedStats.avgRisingEdgeLength :
                             m_learnedStats.avgFallingEdgeLength;
            edgeLength = qMin(edgeLength, m_recentData.size() / 2);

            if (edgeLength >= 10) { // 至少需要10个点

                QVector<QPointF> edgePoints;
                int startPos = m_recentData.size() - edgeLength;

                for (int i = startPos; i < m_recentData.size(); i++) {
                    auto iter = m_recentData.begin() + i;
                    edgePoints.append(QPointF(i - startPos, iter->value));
                }

                double correlation;
                double actualSlope = linearRegression(edgePoints, correlation);

                // 检查斜率异常
                if (std::abs(correlation) > 0.6) { // 拟合质量足够好
                    double expectedSlope = (lastPhase == TrianglePhase::Rising) ?
                                          m_learnedStats.avgRisingSlope : m_learnedStats.avgFallingSlope;
                    double slopeStdDev = (lastPhase == TrianglePhase::Rising) ?
                                        m_learnedStats.risingSlopeStdDev : m_learnedStats.fallingSlopeStdDev;

                    double deviation = std::abs(actualSlope - expectedSlope);
                    double threshold = m_slopeAnomalyThreshold * slopeStdDev;

                    if (deviation > threshold && threshold > 0.01) { // 避免除零
                        qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
                        if (currentTime - m_lastAnomalyTime >= MIN_ANOMALY_INTERVAL) {

                            double severity = qMin(1.0, deviation / (threshold + 1e-6));
                            TriangleAnomalyType anomalyType = (lastPhase == TrianglePhase::Rising) ?
                                                            TriangleAnomalyType::RisingSlopeAnomaly :
                                                            TriangleAnomalyType::FallingSlopeAnomaly;

                            auto anomaly = createAnomalyResult(
                                anomalyType,
                                severity,
                                m_recentData.last().value,
                                currentTime,
                                QString("%1斜率异常：实际=%2，期望=%3±%4，相关性=%5")
                                    .arg(lastPhase == TrianglePhase::Rising ? "上升沿" : "下降沿")
                                    .arg(actualSlope, 0, 'f', 3)
                                    .arg(expectedSlope, 0, 'f', 3)
                                    .arg(threshold, 0, 'f', 3)
                                    .arg(correlation, 0, 'f', 3),
                                expectedSlope,
                                actualSlope
                            );

                            m_lastAnomalyTime = currentTime;
                            emit anomalyDetected(anomaly);
                        }
                    }
                }
            }
        }
    }

    lastPhase = currentPhase;

    // 更新可信的相位
    if (currentPhase != TrianglePhase::Unknown) {
        m_lastConfidentPhase = currentPhase;
    }
}

// 数据跳变异常检测
void OptimizedTriangleAnomalyDetector::checkDataJumpAnomaly(uint16_t currentValue, uint16_t previousValue, qint64 timestamp)
{
    double jumpMagnitude = std::abs(static_cast<double>(currentValue) - static_cast<double>(previousValue));

    if (jumpMagnitude > m_jumpAnomalyThreshold) {
        // 防止异常检测过于频繁
        if (timestamp - m_lastAnomalyTime >= MIN_ANOMALY_INTERVAL) {

            double severity = qMin(1.0, jumpMagnitude / (m_jumpAnomalyThreshold * 2.0));

            TriangleAnomalyResult anomaly;
            anomaly.type = TriangleAnomalyType::DataJumpAnomaly;
            anomaly.severity = severity;
            anomaly.triggerValue = currentValue;
            anomaly.timestamp = timestamp;
            anomaly.phaseWhenDetected = m_currentPhase;
            anomaly.description = QString("数据跳变异常：从%1跳变到%2，幅度=%3")
                                 .arg(previousValue)
                                 .arg(currentValue)
                                 .arg(jumpMagnitude, 0, 'f', 1);
            anomaly.expectedValue = previousValue;
            anomaly.actualValue = currentValue;
            anomaly.threshold = m_jumpAnomalyThreshold;
            anomaly.previousValue = previousValue;
            anomaly.jumpMagnitude = jumpMagnitude;

            m_lastAnomalyTime = timestamp;
            emit anomalyDetected(anomaly);
        }
    }
}

TriangleAnomalyResult OptimizedTriangleAnomalyDetector::createAnomalyResult(TriangleAnomalyType type,double severity,
                                                                            uint16_t triggerValue, qint64 timestamp,
                                                                            const QString& description,
                                                                            double expectedValue, double actualValue) const
{
    TriangleAnomalyResult result;
    result.type = type;
    result.severity = severity;
    result.triggerValue = triggerValue;
    result.timestamp = timestamp;
    result.phaseWhenDetected = m_currentPhase;
    result.description = description;
    result.expectedValue = expectedValue;
    result.actualValue = actualValue;
    return result;
}

void OptimizedTriangleAnomalyDetector::reset()
{
    QMutexLocker locker(&m_mutex);

    m_learningData.clear();
    m_recentData.clear();
    m_peaksValleys.clear();
    m_cycles.clear();

    m_learningComplete = false;
    m_learnedStats = TriangleStats{};
    m_currentPhase = TrianglePhase::Unknown;
    m_totalDataPoints = 0;
    m_lastAnomalyTime = 0;

    qDebug() << "三角波检测器已重置";
}

void OptimizedTriangleAnomalyDetector::setAnomalyThresholds(double peakThreshold,
                                                          double valleyThreshold,
                                                          double slopeThreshold,
                                                          double periodThreshold,
                                                          double jumpThreshold)
{
    m_peakAnomalyThreshold = peakThreshold;
    m_valleyAnomalyThreshold = valleyThreshold;
    m_slopeAnomalyThreshold = slopeThreshold;
    m_periodAnomalyThreshold = periodThreshold;
    m_jumpAnomalyThreshold = jumpThreshold;

    qDebug() << QString("异常检测阈值已更新: 波峰=%1σ, 波谷=%2σ, 斜率=%3σ, 周期=%4, 跳变=%5")
                .arg(peakThreshold).arg(valleyThreshold)
                .arg(slopeThreshold).arg(periodThreshold).arg(jumpThreshold);
}

QString OptimizedTriangleAnomalyDetector::getDebugInfo() const
{
    QMutexLocker locker(&m_mutex);

    QString info;
    info += QString("学习状态: %1\n").arg(m_learningComplete ? "已完成" : "进行中");
    info += QString("总数据点: %1\n").arg(m_totalDataPoints);
    info += QString("当前相位: %1\n").arg(static_cast<int>(m_currentPhase));

    if (m_learnedStats.isValid) {
        info += QString("统计信息:\n");
        info += QString("  平均波峰: %1±%2\n").arg(m_learnedStats.avgPeakValue, 0, 'f', 1)
                                            .arg(m_learnedStats.peakValueStdDev, 0, 'f', 1);
        info += QString("  平均波谷: %1±%2\n").arg(m_learnedStats.avgValleyValue, 0, 'f', 1)
                                            .arg(m_learnedStats.valleyValueStdDev, 0, 'f', 1);
        info += QString("  平均周期: %1\n").arg(m_learnedStats.avgPeriodLength);
        info += QString("  总周期数: %1\n").arg(m_learnedStats.totalCycles);
    }

    return info;
}
