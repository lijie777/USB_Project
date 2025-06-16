
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

        // 检测异常
        checkForAnomalies(value, timestamp);
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
        qDebug() << QString("平均周期长度: %1, 波峰: %2±%3, 波谷: %4±%5")
                    .arg(m_learnedStats.avgPeriodLength)
                    .arg(m_learnedStats.avgPeakValue, 0, 'f', 1)
                    .arg(m_learnedStats.peakValueStdDev, 0, 'f', 1)
                    .arg(m_learnedStats.avgValleyValue, 0, 'f', 1)
                    .arg(m_learnedStats.valleyValueStdDev, 0, 'f', 1);

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

    if (m_peaksValleys.size() < 4) return; // 至少需要2个波峰2个波谷

    // 按索引排序
    std::sort(m_peaksValleys.begin(), m_peaksValleys.end(),
              [](const PeakValleyPoint& a, const PeakValleyPoint& b) {
                  return a.index < b.index;
              });

    // 识别完整周期（波峰到波峰，或波谷到波谷）
    for (int i = 0; i < m_peaksValleys.size() - 1; i++) {
        for (int j = i + 1; j < m_peaksValleys.size(); j++) {
            const auto& start = m_peaksValleys[i];
            const auto& end = m_peaksValleys[j];

            // 同类型的点构成一个周期
            if (start.isPeak == end.isPeak) {
                CycleInfo cycle;
                cycle.startIndex = start.index;
                cycle.endIndex = end.index;
                cycle.periodLength = end.index - start.index;

                // 在这个周期内找波峰和波谷
                uint16_t maxVal = 0, minVal = 65535;
                for (int k = i; k <= j; k++) {
                    if (m_peaksValleys[k].isPeak) {
                        maxVal = qMax(maxVal, m_peaksValleys[k].value);
                    } else {
                        minVal = qMin(minVal, m_peaksValleys[k].value);
                    }
                }

                cycle.peakValue = maxVal;
                cycle.valleyValue = minVal;

                // 计算斜率
                int midPoint = (start.index + end.index) / 2;
                cycle.risingSlope = calculateSlope(cycle.startIndex, midPoint);
                cycle.fallingSlope = calculateSlope(midPoint, cycle.endIndex);

                m_cycles.append(cycle);
                break; // 找到一个周期后，从下一个点开始
            }
        }
    }

    qDebug() << "识别到" << m_cycles.size() << "个完整周期";
}

void OptimizedTriangleAnomalyDetector::calculateStatistics()
{
    if (m_cycles.isEmpty()) {
        m_learnedStats.isValid = false;
        return;
    }

    // 计算各项平均值
    QVector<double> peakValues, valleyValues, risingSlopes, fallingSlopes, periodLengths;

    for (const auto& cycle : m_cycles) {
        peakValues.append(cycle.peakValue);
        valleyValues.append(cycle.valleyValue);
        risingSlopes.append(cycle.risingSlope);
        fallingSlopes.append(cycle.fallingSlope);
        periodLengths.append(cycle.periodLength);
    }

    // 计算平均值
    m_learnedStats.avgPeakValue = std::accumulate(peakValues.begin(), peakValues.end(), 0.0) / peakValues.size();
    m_learnedStats.avgValleyValue = std::accumulate(valleyValues.begin(), valleyValues.end(), 0.0) / valleyValues.size();
    m_learnedStats.avgRisingSlope = std::accumulate(risingSlopes.begin(), risingSlopes.end(), 0.0) / risingSlopes.size();
    m_learnedStats.avgFallingSlope = std::accumulate(fallingSlopes.begin(), fallingSlopes.end(), 0.0) / fallingSlopes.size();
    m_learnedStats.avgPeriodLength = std::accumulate(periodLengths.begin(), periodLengths.end(), 0.0) / periodLengths.size();

    // 计算标准差
    auto calcStdDev = [](const QVector<double>& values, double mean) {
        double sum = 0;
        for (double val : values) {
            sum += (val - mean) * (val - mean);
        }
        return std::sqrt(sum / values.size());
    };

    m_learnedStats.peakValueStdDev = calcStdDev(peakValues, m_learnedStats.avgPeakValue);
    m_learnedStats.valleyValueStdDev = calcStdDev(valleyValues, m_learnedStats.avgValleyValue);
    m_learnedStats.risingSlopeStdDev = calcStdDev(risingSlopes, m_learnedStats.avgRisingSlope);
    m_learnedStats.fallingSlopeStdDev = calcStdDev(fallingSlopes, m_learnedStats.avgFallingSlope);

    m_learnedStats.totalCycles = m_cycles.size();
    m_learnedStats.isValid = true;
}

double OptimizedTriangleAnomalyDetector::calculateSlope(int startIdx, int endIdx) const
{
    if (startIdx >= endIdx || startIdx < 0 || endIdx >= m_learningData.size()) {
        return 0.0;
    }

    auto startIter = m_learningData.begin() + startIdx;
    auto endIter = m_learningData.begin() + endIdx;

    double deltaY = endIter->value - startIter->value;
    double deltaX = endIdx - startIdx;

    return deltaX > 0 ? deltaY / deltaX : 0.0;
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
    m_currentPhase = determineCurrentPhase(value);

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
                value,
                threshold
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
                value,
                threshold
            );

            m_lastAnomalyTime = timestamp;
            emit anomalyDetected(anomaly);
        }
    }

    // 斜率异常检测（需要足够的最近数据）
    if (m_recentData.size() >= 10) {
        double currentSlope = calculateSlope(m_recentData.size() - 10, m_recentData.size() - 1);

        if (m_currentPhase == TrianglePhase::Rising) {
            double deviation = std::abs(currentSlope - m_learnedStats.avgRisingSlope);
            double threshold = m_slopeAnomalyThreshold * m_learnedStats.risingSlopeStdDev;

            if (deviation > threshold) {
                double severity = deviation / (threshold + 1e-6);
                auto anomaly = createAnomalyResult(
                    TriangleAnomalyType::RisingSlopeAnomaly,
                    qMin(1.0, severity),
                    value,
                    timestamp,
                    QString("上升斜率异常：实际=%1，期望=%2±%3")
                        .arg(currentSlope, 0, 'f', 3)
                        .arg(m_learnedStats.avgRisingSlope, 0, 'f', 3)
                        .arg(threshold, 0, 'f', 3),
                    m_learnedStats.avgRisingSlope,
                    currentSlope,
                    threshold
                );

                m_lastAnomalyTime = timestamp;
                emit anomalyDetected(anomaly);
            }
        } else if (m_currentPhase == TrianglePhase::Falling) {
            double deviation = std::abs(currentSlope - m_learnedStats.avgFallingSlope);
            double threshold = m_slopeAnomalyThreshold * m_learnedStats.fallingSlopeStdDev;

            if (deviation > threshold) {
                double severity = deviation / (threshold + 1e-6);
                auto anomaly = createAnomalyResult(
                    TriangleAnomalyType::FallingSlopeAnomaly,
                    qMin(1.0, severity),
                    value,
                    timestamp,
                    QString("下降斜率异常：实际=%1，期望=%2±%3")
                        .arg(currentSlope, 0, 'f', 3)
                        .arg(m_learnedStats.avgFallingSlope, 0, 'f', 3)
                        .arg(threshold, 0, 'f', 3),
                    m_learnedStats.avgFallingSlope,
                    currentSlope,
                    threshold
                );

                m_lastAnomalyTime = timestamp;
                emit anomalyDetected(anomaly);
            }
        }
    }
    //TODO:数据有可能发生跳变的处理

}

TrianglePhase OptimizedTriangleAnomalyDetector::determineCurrentPhase(uint16_t currentValue) const
{
    if (!m_learnedStats.isValid) return TrianglePhase::Unknown;

    double peakThreshold = m_learnedStats.avgPeakValue - m_learnedStats.peakValueStdDev;
    double valleyThreshold = m_learnedStats.avgValleyValue + m_learnedStats.valleyValueStdDev;

    if (currentValue >= peakThreshold) {
        return TrianglePhase::AtPeak;
    } else if (currentValue <= valleyThreshold) {
        return TrianglePhase::AtValley;
    } else {
        // 根据最近的趋势判断上升还是下降
        if (m_recentData.size() >= 3) {
            auto last = m_recentData.end() - 1;
            auto secondLast = m_recentData.end() - 2;

            if (last->value > secondLast->value) {
                return TrianglePhase::Rising;
            } else {
                return TrianglePhase::Falling;
            }
        }
    }

    return TrianglePhase::Unknown;
}

TriangleAnomalyResult OptimizedTriangleAnomalyDetector::createAnomalyResult(TriangleAnomalyType type,double severity,
                                                                            uint16_t triggerValue, qint64 timestamp,
                                                                            const QString& description,
                                                                            double expectedValue, double actualValue,
                                                                            double threshold) const
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
    result.threshold = threshold;
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
                                                          double periodThreshold)
{
    m_peakAnomalyThreshold = peakThreshold;
    m_valleyAnomalyThreshold = valleyThreshold;
    m_slopeAnomalyThreshold = slopeThreshold;
    m_periodAnomalyThreshold = periodThreshold;

    qDebug() << QString("异常检测阈值已更新: 波峰=%1σ, 波谷=%2σ, 斜率=%3σ, 周期=%4")
                .arg(peakThreshold).arg(valleyThreshold)
                .arg(slopeThreshold).arg(periodThreshold);
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
