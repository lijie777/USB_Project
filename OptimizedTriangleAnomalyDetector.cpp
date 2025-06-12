
// OptimizedTriangleAnomalyDetector.cpp
// 简化版三角波异常检测算法实现

#include "OptimizedTriangleAnomalyDetector.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include "MCLogger.h"

OptimizedTriangleAnomalyDetector::OptimizedTriangleAnomalyDetector(QObject *parent)
    : QObject(parent)
    , m_samplingRate(TRIANGLE_SAMPLING_RATE)
    , m_recordingDuration(TRIANGLE_RECORDING_DURATION)
    , m_analysisCounter(0)
    , m_samplesProcessed(0)
    , m_isRecording(false)
    , m_recordedDataPoints(0)
    , m_previousPhase(TrianglePhase::Unknown)
    , m_phaseStartIndex(0)
    , m_phaseStartTime(0)
    , m_cycleStartTime(0)
    , m_isInitialStabilizationComplete(false)  // 新增
    , m_stablePhaseCount(0)                    // 新增
    , m_lastConfirmedPhase(TrianglePhase::Unknown)  // 新增
    , m_estimatedMin(0)                        // 新增
    , m_estimatedMax(4095)                     // 新增
    , m_rangeEstimationComplete(false)         // 新增
    , m_validCyclesCount(0)                    // 新增
    , m_totalAttemptedCycles(0)                // 新增
{
    // 初始化记录定时器
    m_recordingTimer = new QTimer(this);
    m_recordingTimer->setSingleShot(true);
    connect(m_recordingTimer, &QTimer::timeout, this, &OptimizedTriangleAnomalyDetector::onRecordingTimeout);

    // 初始化统计信息
    m_currentStats.isLearning = true;

    LOG_INFO_CL("简化版三角波异常检测器初始化完成");
    LOG_INFO_CL("学习周期数: {}, 采样率: {} Hz, 记录时长: {} 秒",
                TRIANGLE_LEARNING_CYCLES, TRIANGLE_SAMPLING_RATE, TRIANGLE_RECORDING_DURATION);
}

OptimizedTriangleAnomalyDetector::~OptimizedTriangleAnomalyDetector()
{
    if (m_isRecording) {
        m_recordingTimer->stop();
        emit recordingStopped(m_recordedDataPoints);
    }
}

void OptimizedTriangleAnomalyDetector::feedData(uint16_t value)
{
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();

    // 添加数据到缓冲区
    m_dataBuffer.push_back(value);
    m_timestampBuffer.push_back(currentTime);

    // 维护缓冲区大小 - 保存TRIANGLE_DATA_BUFFER_SECONDS秒的数据
    int maxBufferSize = static_cast<int>(m_samplingRate * TRIANGLE_DATA_BUFFER_SECONDS);
    if (m_dataBuffer.size() > maxBufferSize) {
        m_dataBuffer.pop_front();
        m_timestampBuffer.pop_front();

        // 调整索引
        if (m_phaseStartIndex > 0) {
            m_phaseStartIndex--;
        }
    }

    m_samplesProcessed++;

    // 如果正在记录，发送数据
    if (m_isRecording) {
        emit recordingData(value, currentTime);
        m_recordedDataPoints++;
    }

    // 性能优化：每TRIANGLE_ANALYSIS_INTERVAL个数据点分析一次
    m_analysisCounter++;
    if (m_analysisCounter >= TRIANGLE_ANALYSIS_INTERVAL) {
        m_analysisCounter = 0;

        if (m_dataBuffer.size() >= TRIANGLE_MIN_SAMPLES_PER_CYCLE) {
            analyzeCurrentData();
        }
    }
}

void OptimizedTriangleAnomalyDetector::analyzeCurrentData()
{
    // 检测相位转换
    detectPhaseTransition();

    // 更新统计信息
    updateStatistics();

    // 检查是否完成了一个周期
    if (m_currentStats.currentPhase == TrianglePhase::AtValley &&
        m_previousPhase == TrianglePhase::Falling) {
        processCycleCompletion();
    }

    // 学习阶段处理
    if (m_currentStats.isLearning) {
        updateLearningData();

        // 发送学习进度（基于有效周期数）
        emit learningProgressUpdated(m_validCyclesCount, TRIANGLE_MIN_VALID_CYCLES_FOR_LEARNING);

        // 检查是否完成学习（使用有效周期数而不是总周期数）
        if (m_validCyclesCount >= TRIANGLE_MIN_VALID_CYCLES_FOR_LEARNING) {
            completeLearningPhase();
        }

        // 如果尝试了太多次仍然没有足够的有效周期，降低要求
        if (m_totalAttemptedCycles >= TRIANGLE_LEARNING_CYCLES * 3 &&
            m_validCyclesCount >= TRIANGLE_MIN_VALID_CYCLES_FOR_LEARNING - 2) {

            LOG_WARN_CL("降低学习要求：尝试 {} 次，获得 {} 个有效周期",
                       m_totalAttemptedCycles, m_validCyclesCount);
            completeLearningPhase();
        }
    }else {
        // 异常检测阶段
        TriangleAnomalyResult anomaly = checkForAnomalies();

        if (anomaly.type != TriangleAnomalyType::None &&
            anomaly.severity >= TRIANGLE_MIN_ANOMALY_SEVERITY) {

            m_lastAnomaly = anomaly;
            emit anomalyDetected(anomaly);

            // 开始记录（如果未在记录中且严重度足够）
            if (!m_isRecording) {
                m_isRecording = true;
                m_recordedDataPoints = 0;
                m_recordingTimer->start(m_recordingDuration * 1000);
                emit recordingStarted(anomaly);

                LOG_INFO_CL("检测到三角波异常: 类型={}, 严重程度={}%",
                           static_cast<int>(anomaly.type), int(anomaly.severity * 100));
            }
        }
    }

    // 定期发送统计更新
    if (m_samplesProcessed % (TRIANGLE_ANALYSIS_INTERVAL * 20) == 0) {
        emit statsUpdated(m_currentStats);
    }
}

void OptimizedTriangleAnomalyDetector::detectPhaseTransition()
{
    if (m_dataBuffer.size() < TRIANGLE_PHASE_WINDOW_SIZE + 1) return;

    // 初始稳定期处理
    if (!m_isInitialStabilizationComplete) {
        handleInitialStabilization();
        return;
    }

    TrianglePhase newPhase = determineCurrentPhase();

    // 相位置信度验证 - 防止噪声导致的误判
    if (newPhase == m_lastConfirmedPhase) {
        m_stablePhaseCount++;
    } else {
        m_stablePhaseCount = 1;
        m_lastConfirmedPhase = newPhase;
    }

    // 只有当相位稳定足够长时间才确认相位转换
    if (m_stablePhaseCount >= TRIANGLE_PHASE_CONFIDENCE_THRESHOLD &&
        newPhase != m_previousPhase &&
        newPhase != TrianglePhase::Unknown) {

        processPhaseTransition(newPhase);
    }

    m_currentStats.currentPhase = m_lastConfirmedPhase;
}

// 新增：初始稳定期处理
void OptimizedTriangleAnomalyDetector::handleInitialStabilization()
{
    // 估算数据范围
    if (!m_rangeEstimationComplete &&
        m_dataBuffer.size() >= TRIANGLE_INITIAL_RANGE_DETECTION_WINDOW) {

        estimateDataRange();
        m_rangeEstimationComplete = true;

        LOG_INFO_CL("数据范围估算完成: {} - {}", m_estimatedMin, m_estimatedMax);
    }

    // 检查是否完成初始稳定期
    if (m_samplesProcessed >= TRIANGLE_INITIAL_STABILIZATION_SAMPLES &&
        m_rangeEstimationComplete) {

        m_isInitialStabilizationComplete = true;

        LOG_INFO_CL("初始稳定期完成，开始正式相位检测");
        LOG_INFO_CL("估算的数据范围: {} - {} (幅度: {})",
                   m_estimatedMin, m_estimatedMax, m_estimatedMax - m_estimatedMin);
    }
}

// 新增：估算数据范围
void OptimizedTriangleAnomalyDetector::estimateDataRange()
{
    if (m_dataBuffer.size() < TRIANGLE_INITIAL_RANGE_DETECTION_WINDOW) return;

    // 使用最近的样本估算范围
    auto recentStart = m_dataBuffer.end() - TRIANGLE_INITIAL_RANGE_DETECTION_WINDOW;
    auto recentEnd = m_dataBuffer.end();

    auto minMax = std::minmax_element(recentStart, recentEnd);
    m_estimatedMin = *minMax.first;
    m_estimatedMax = *minMax.second;

    // 添加一些余量以应对变化
    uint16_t range = m_estimatedMax - m_estimatedMin;
    uint16_t margin = range * 0.1; // 10%余量

    m_estimatedMin = (m_estimatedMin > margin) ? m_estimatedMin - margin : 0;
    m_estimatedMax = (m_estimatedMax + margin < 4095) ? m_estimatedMax + margin : 4095;
}


TrianglePhase OptimizedTriangleAnomalyDetector::determineCurrentPhase()
{
    if (m_dataBuffer.size() < TRIANGLE_PHASE_WINDOW_SIZE || !m_rangeEstimationComplete) {
        return TrianglePhase::Unknown;
    }

    size_t currentIdx = m_dataBuffer.size() - 1;

    // 分析最近几个点的趋势
    int risingCount = 0, fallingCount = 0;

    for (int i = 1; i < TRIANGLE_PHASE_WINDOW_SIZE; i++) {
        size_t idx = currentIdx - TRIANGLE_PHASE_WINDOW_SIZE + i;
        if (idx > 0 && idx < m_dataBuffer.size()) {
            if (m_dataBuffer[idx] > m_dataBuffer[idx-1]) {
                risingCount++;
            } else if (m_dataBuffer[idx] < m_dataBuffer[idx-1]) {
                fallingCount++;
            }
        }
    }

    // 计算当前斜率
    double currentSlope = calculateSlope(currentIdx - TRIANGLE_PHASE_WINDOW_SIZE, currentIdx);
    uint16_t currentValue = m_dataBuffer[currentIdx];

    // 使用估算的范围进行相位判断
    uint16_t range = m_estimatedMax - m_estimatedMin;

    // 确保有足够的幅度进行判断
    if (range < TRIANGLE_MIN_AMPLITUDE) {
        return TrianglePhase::Unknown;
    }

    // 判断是否接近峰值或谷值
    bool nearPeak = (currentValue > m_estimatedMin + range * TRIANGLE_PEAK_VALLEY_RATIO);
    bool nearValley = (currentValue < m_estimatedMin + range * (1.0 - TRIANGLE_PEAK_VALLEY_RATIO));

    // 相位判断逻辑（使用估算的范围）
    if (nearPeak && std::abs(currentSlope) < TRIANGLE_SLOPE_THRESHOLD) {
        return TrianglePhase::AtPeak;
    } else if (nearValley && std::abs(currentSlope) < TRIANGLE_SLOPE_THRESHOLD) {
        return TrianglePhase::AtValley;
    } else if (risingCount > fallingCount && currentSlope > TRIANGLE_STRONG_SLOPE_THRESHOLD) {
        return TrianglePhase::Rising;
    } else if (fallingCount > risingCount && currentSlope < -TRIANGLE_STRONG_SLOPE_THRESHOLD) {
        return TrianglePhase::Falling;
    }

    return TrianglePhase::Unknown;
}

void OptimizedTriangleAnomalyDetector::processCycleCompletion()
{
    qint64 currentTime = m_timestampBuffer.back();

    // 完成当前周期数据
    m_currentCycle.endTime = currentTime;
    m_currentCycle.duration = currentTime - m_currentCycle.startTime;
    m_currentCycle.peakValue = m_dataBuffer.back(); // 假设刚刚经过峰值

    // 计算斜率（简化计算）
    if (m_phaseStartIndex > TRIANGLE_MIN_SAMPLES_PER_CYCLE) {
        int risingStart = std::max(0, m_phaseStartIndex - static_cast<int>(m_dataBuffer.size()) / 2);
        int risingEnd = m_phaseStartIndex;
        int fallingStart = m_phaseStartIndex;
        int fallingEnd = m_dataBuffer.size() - 1;

        m_currentCycle.risingSlope = calculateSlope(risingStart, risingEnd);
        m_currentCycle.fallingSlope = calculateSlope(fallingStart, fallingEnd);
    }

    // 验证周期有效性
    m_currentCycle.isValid = isValidCycle(m_currentCycle);

    if (m_currentCycle.isValid) {
        m_currentStats.completedCycles++;

        // 更新当前统计
        m_currentStats.currentPeriodMs = m_currentCycle.duration;
        m_currentStats.currentRisingSlope = m_currentCycle.risingSlope;
        m_currentStats.currentFallingSlope = std::abs(m_currentCycle.fallingSlope);
        m_currentStats.currentPeakValue = m_currentCycle.peakValue;
        m_currentStats.currentValleyValue = m_currentCycle.valleyValue;

        if (m_currentCycle.duration > 0) {
            m_currentStats.currentFrequency = 1000.0 / m_currentCycle.duration;
        }

        LOG_DEBUG_CL("完成周期 #{}: 时长={}ms, 频率={:.3f}Hz, 上升斜率={:.2f}, 下降斜率={:.2f}",
                    m_currentStats.completedCycles, m_currentCycle.duration,
                    m_currentStats.currentFrequency, m_currentCycle.risingSlope, m_currentCycle.fallingSlope);
    }
}

bool OptimizedTriangleAnomalyDetector::isValidCycle(const CycleData& cycle)
{
      // 基本验证
      if (cycle.duration < TRIANGLE_MIN_CYCLE_DURATION_MS ||
        cycle.duration > TRIANGLE_MAX_CYCLE_DURATION_MS) {
        return false;
    }

    // 幅度验证（使用估算范围）
    if (!m_rangeEstimationComplete) return false;

    uint16_t expectedRange = m_estimatedMax - m_estimatedMin;
    uint16_t actualRange = cycle.peakValue - cycle.valleyValue;

    // 实际幅度应该在估算范围的70%-130%之间
    if (actualRange < expectedRange * 0.7 || actualRange > expectedRange * 1.3) {
        LOG_DEBUG_CL("周期幅度验证失败: 实际={}, 期望={}", actualRange, expectedRange);
        return false;
    }

    // 峰谷值合理性验证
    if (cycle.peakValue < m_estimatedMin + expectedRange * 0.6 ||
        cycle.valleyValue > m_estimatedMin + expectedRange * 0.4) {
        LOG_DEBUG_CL("周期峰谷值验证失败: 峰值={}, 谷值={}", cycle.peakValue, cycle.valleyValue);
        return false;
    }

    // 斜率合理性验证
    if (std::abs(cycle.risingSlope) < 1.0 || std::abs(cycle.fallingSlope) < 1.0) {
        return false;
    }

    // 学习阶段使用更严格的验证
    if (m_currentStats.isLearning && TRIANGLE_CYCLE_VALIDATION_STRICT) {
        // 检查斜率符号
        if (cycle.risingSlope <= 0 || cycle.fallingSlope >= 0) {
            LOG_DEBUG_CL("严格模式：斜率符号验证失败");
            return false;
        }
    }

    return true;
}

void OptimizedTriangleAnomalyDetector::updateLearningData()
{
    if (!m_currentCycle.isValid) {
        m_totalAttemptedCycles++;
//        LOG_DEBUG_CL("无效周期 #{}, 总尝试次数: {}", m_totalAttemptedCycles, m_totalAttemptedCycles);
        return;
    }

    // 添加到学习数据
    m_learningCycles.push_back(m_currentCycle);
    m_validCyclesCount++;
    m_totalAttemptedCycles++;

    LOG_DEBUG_CL("有效周期 #{}, 有效率: {:.1f}%",
                m_validCyclesCount,
                (m_validCyclesCount * 100.0) / m_totalAttemptedCycles);

    // 维护学习数据大小
    if (m_learningCycles.size() > TRIANGLE_LEARNING_CYCLES * 2) {
        m_learningCycles.pop_front();
    }
}

void OptimizedTriangleAnomalyDetector::completeLearningPhase()
{
    if (m_learningCycles.empty()) return;

    LOG_INFO_CL("=== 开始完成学习阶段 ===");

    // 计算基准参数
    std::vector<double> risingSlopes, fallingSlopes;
    std::vector<uint16_t> peakValues, valleyValues;
    std::vector<qint64> periods;

    for (const auto& cycle : m_learningCycles) {
        if (cycle.isValid) {
            risingSlopes.push_back(cycle.risingSlope);
            fallingSlopes.push_back(std::abs(cycle.fallingSlope));
            peakValues.push_back(cycle.peakValue);
            valleyValues.push_back(cycle.valleyValue);
            periods.push_back(cycle.duration);
        }
    }

    // 计算中位数作为基准（更稳定）
    if (!risingSlopes.empty()) {
        std::sort(risingSlopes.begin(), risingSlopes.end());
        std::sort(fallingSlopes.begin(), fallingSlopes.end());
        std::sort(peakValues.begin(), peakValues.end());
        std::sort(valleyValues.begin(), valleyValues.end());
        std::sort(periods.begin(), periods.end());

        size_t mid = risingSlopes.size() / 2;

        m_currentStats.baselineRisingSlope = risingSlopes[mid];
        m_currentStats.baselineFallingSlope = fallingSlopes[mid];
        m_currentStats.baselinePeakValue = peakValues[mid];
        m_currentStats.baselineValleyValue = valleyValues[mid];
        m_currentStats.baselinePeriodMs = periods[mid];

        if (periods[mid] > 0) {
            m_currentStats.baselineFrequency = 1000.0 / periods[mid];
        }
    }

    // 完成学习
    m_currentStats.isLearning = false;

    LOG_INFO_CL("=== 学习阶段完成 ===");
    LOG_INFO_CL("基准频率: {:.3f} Hz", m_currentStats.baselineFrequency);
    LOG_INFO_CL("基准周期: {} ms", m_currentStats.baselinePeriodMs);
    LOG_INFO_CL("基准上升斜率: {:.2f}", m_currentStats.baselineRisingSlope);
    LOG_INFO_CL("基准下降斜率: {:.2f}", m_currentStats.baselineFallingSlope);
    LOG_INFO_CL("基准波峰值: {}", m_currentStats.baselinePeakValue);
    LOG_INFO_CL("基准波谷值: {}", m_currentStats.baselineValleyValue);
    LOG_INFO_CL("========================");

    emit learningCompleted(m_currentStats);
}

TriangleAnomalyResult OptimizedTriangleAnomalyDetector::checkForAnomalies()
{
    TriangleAnomalyResult result;
    double maxSeverity = 0.0;
    double severity;

    // 检查上升斜率异常
    if (checkRisingSlopeAnomaly(severity) && severity > maxSeverity) {
        result.type = TriangleAnomalyType::RisingSlopeAnomaly;
        result.severity = severity;
        result.description = QString("上升斜率异常: 当前值=%.2f, 基准值=%.2f, 偏差=%.1f%%")
                           .arg(m_currentStats.currentRisingSlope)
                           .arg(m_currentStats.baselineRisingSlope)
                           .arg(severity * 100);
        maxSeverity = severity;
    }

    // 检查下降斜率异常
    if (checkFallingSlopeAnomaly(severity) && severity > maxSeverity) {
        result.type = TriangleAnomalyType::FallingSlopeAnomaly;
        result.severity = severity;
        result.description = QString("下降斜率异常: 当前值=%.2f, 基准值=%.2f, 偏差=%.1f%%")
                           .arg(m_currentStats.currentFallingSlope)
                           .arg(m_currentStats.baselineFallingSlope)
                           .arg(severity * 100);
        maxSeverity = severity;
    }

    // 检查波峰值异常
    if (checkPeakValueAnomaly(severity) && severity > maxSeverity) {
        result.type = TriangleAnomalyType::PeakValueAnomaly;
        result.severity = severity;
        result.description = QString("波峰值异常: 当前值=%1, 基准值=%2, 偏差=%.1f%%")
                           .arg(m_currentStats.currentPeakValue)
                           .arg(m_currentStats.baselinePeakValue)
                           .arg(severity * 100);
        maxSeverity = severity;
    }

    // 检查波谷值异常
    if (checkValleyValueAnomaly(severity) && severity > maxSeverity) {
        result.type = TriangleAnomalyType::ValleyValueAnomaly;
        result.severity = severity;
        result.description = QString("波谷值异常: 当前值=%1, 基准值=%2, 偏差=%.1f%%")
                           .arg(m_currentStats.currentValleyValue)
                           .arg(m_currentStats.baselineValleyValue)
                           .arg(severity * 100);
        maxSeverity = severity;
    }

    // 检查周期异常
    if (checkPeriodAnomaly(severity) && severity > maxSeverity) {
        result.type = TriangleAnomalyType::PeriodAnomaly;
        result.severity = severity;
        result.description = QString("周期异常: 当前值=%1ms, 基准值=%2ms, 偏差=%.1f%%")
                           .arg(m_currentStats.currentPeriodMs)
                           .arg(m_currentStats.baselinePeriodMs)
                           .arg(severity * 100);
        maxSeverity = severity;
    }

    // 填充结果信息
    if (result.type != TriangleAnomalyType::None) {
        result.triggerValue = m_dataBuffer.back();
        result.timestamp = QDateTime::currentMSecsSinceEpoch();
        result.phaseWhenDetected = m_currentStats.currentPhase;
    }

    return result;
}

// =============================================================================
// 具体的异常检测函数实现
// =============================================================================

bool OptimizedTriangleAnomalyDetector::checkRisingSlopeAnomaly(double& severity)
{
    if (m_currentStats.baselineRisingSlope <= 0) return false;

    double deviation = std::abs(m_currentStats.currentRisingSlope - m_currentStats.baselineRisingSlope) /
                      m_currentStats.baselineRisingSlope;

    if (deviation > TRIANGLE_RISING_SLOPE_TOLERANCE) {
        severity = std::min(deviation / TRIANGLE_RISING_SLOPE_TOLERANCE, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::checkFallingSlopeAnomaly(double& severity)
{
    if (m_currentStats.baselineFallingSlope <= 0) return false;

    double deviation = std::abs(m_currentStats.currentFallingSlope - m_currentStats.baselineFallingSlope) /
                      m_currentStats.baselineFallingSlope;

    if (deviation > TRIANGLE_FALLING_SLOPE_TOLERANCE) {
        severity = std::min(deviation / TRIANGLE_FALLING_SLOPE_TOLERANCE, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::checkPeakValueAnomaly(double& severity)
{
    if (m_currentStats.baselinePeakValue <= 0) return false;

    double deviation = std::abs(static_cast<double>(m_currentStats.currentPeakValue) - m_currentStats.baselinePeakValue) /
                      m_currentStats.baselinePeakValue;

    if (deviation > TRIANGLE_PEAK_VALUE_TOLERANCE) {
        severity = std::min(deviation / TRIANGLE_PEAK_VALUE_TOLERANCE, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::checkValleyValueAnomaly(double& severity)
{
    if (m_currentStats.baselineValleyValue == 0) return false;

    double baseline = std::max(1.0, static_cast<double>(m_currentStats.baselineValleyValue));
    double deviation = std::abs(static_cast<double>(m_currentStats.currentValleyValue) - baseline) / baseline;

    if (deviation > TRIANGLE_VALLEY_VALUE_TOLERANCE) {
        severity = std::min(deviation / TRIANGLE_VALLEY_VALUE_TOLERANCE, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::checkPeriodAnomaly(double& severity)
{
    if (m_currentStats.baselinePeriodMs <= 0) return false;

    double deviation = std::abs(static_cast<double>(m_currentStats.currentPeriodMs) - m_currentStats.baselinePeriodMs) /
                      m_currentStats.baselinePeriodMs;

    if (deviation > TRIANGLE_PERIOD_TOLERANCE) {
        severity = std::min(deviation / TRIANGLE_PERIOD_TOLERANCE, 1.0);
        return true;
    }
    return false;
}

// =============================================================================
// 辅助函数实现
// =============================================================================

double OptimizedTriangleAnomalyDetector::calculateSlope(int startIdx, int endIdx)
{
    if (startIdx >= endIdx || endIdx >= static_cast<int>(m_dataBuffer.size()) || startIdx < 0) {
        return 0.0;
    }

    double deltaY = static_cast<double>(m_dataBuffer[endIdx]) - m_dataBuffer[startIdx];
    double deltaX = endIdx - startIdx;

    return deltaX > 0 ? deltaY / deltaX : 0.0;
}

double OptimizedTriangleAnomalyDetector::calculateNoiseLevel()
{
    if (m_dataBuffer.size() < 10) return 0.0;

    double totalVariation = 0.0;
    for (size_t i = m_dataBuffer.size() - 10; i < m_dataBuffer.size() - 1; i++) {
        totalVariation += std::abs(static_cast<double>(m_dataBuffer[i+1]) - m_dataBuffer[i]);
    }

    return totalVariation / 9.0;
}


void OptimizedTriangleAnomalyDetector::setOptimalParameters()
{
    LOG_INFO_CL("使用预设的最优参数配置");
    LOG_INFO_CL("学习周期: {} 个", TRIANGLE_LEARNING_CYCLES);
    LOG_INFO_CL("采样率: {} Hz", TRIANGLE_SAMPLING_RATE);
    LOG_INFO_CL("上升斜率容差: {}%", TRIANGLE_RISING_SLOPE_TOLERANCE * 100);
    LOG_INFO_CL("下降斜率容差: {}%", TRIANGLE_FALLING_SLOPE_TOLERANCE * 100);
    LOG_INFO_CL("波峰值容差: {}%", TRIANGLE_PEAK_VALUE_TOLERANCE * 100);
    LOG_INFO_CL("波谷值容差: {}%", TRIANGLE_VALLEY_VALUE_TOLERANCE * 100);
    LOG_INFO_CL("周期容差: {}%", TRIANGLE_PERIOD_TOLERANCE * 100);
}

void OptimizedTriangleAnomalyDetector::reset()
{
    // 停止记录
    if (m_isRecording) {
        m_recordingTimer->stop();
        m_isRecording = false;
        emit recordingStopped(m_recordedDataPoints);
    }

    // 清空所有缓冲区
    m_dataBuffer.clear();
    m_timestampBuffer.clear();
    m_learningCycles.clear();
    m_risingSlopeHistory.clear();
    m_fallingSlopeHistory.clear();
    m_peakHistory.clear();
    m_valleyHistory.clear();
    m_periodHistory.clear();

    // 重置状态
    m_currentStats = TriangleStats();
    m_currentStats.isLearning = true;
    m_lastAnomaly = TriangleAnomalyResult();
    m_previousPhase = TrianglePhase::Unknown;
    m_phaseStartIndex = 0;
    m_phaseStartTime = 0;
    m_cycleStartTime = 0;
    m_analysisCounter = 0;
    m_samplesProcessed = 0;
    m_recordedDataPoints = 0;
    m_currentCycle = CycleData();

    LOG_INFO_CL("三角波异常检测器已重置，重新开始学习阶段");
}

void OptimizedTriangleAnomalyDetector::onRecordingTimeout()
{
    if (m_isRecording) {
        m_isRecording = false;
        emit recordingStopped(m_recordedDataPoints);
        LOG_INFO_CL("异常记录结束，共记录 {} 个数据点", m_recordedDataPoints);
    }
}

// 新增：处理相位转换
void OptimizedTriangleAnomalyDetector::processPhaseTransition(TrianglePhase newPhase)
{
    qint64 currentTime = m_timestampBuffer.back();

    LOG_DEBUG_CL("相位转换: {} -> {}",
                static_cast<int>(m_previousPhase), static_cast<int>(newPhase));

    // 如果有有效的前一相位，分析刚结束的相位段
    if (m_previousPhase != TrianglePhase::Unknown && m_phaseStartIndex < static_cast<int>(m_dataBuffer.size()) - 1) {
        analyzeCompletedPhaseSegment(m_previousPhase, m_phaseStartIndex, m_dataBuffer.size() - 1);
    }

    // 更新相位信息
    m_previousPhase = newPhase;
    m_phaseStartIndex = m_dataBuffer.size() - 1;
    m_phaseStartTime = currentTime;

    // 处理特殊的相位转换
    switch (newPhase) {
        case TrianglePhase::Rising:
            if (m_previousPhase == TrianglePhase::AtValley) {
                // 新周期开始（从谷值转向上升）
                startNewCycle(currentTime);
            }
            break;

        case TrianglePhase::AtPeak:
            if (m_previousPhase == TrianglePhase::Rising) {
                // 记录波峰信息
                recordPeakValue(m_dataBuffer.back(), currentTime);
            }
            break;

        case TrianglePhase::Falling:
            // 继续当前周期
            break;

        case TrianglePhase::AtValley:
            if (m_previousPhase == TrianglePhase::Falling) {
                // 周期结束，记录波谷并完成周期
                recordValleyValue(m_dataBuffer.back(), currentTime);
                completeCycle(currentTime);
            }
            break;

        default:
            break;
    }
}

// 新增：分析已完成的相位段
void OptimizedTriangleAnomalyDetector::analyzeCompletedPhaseSegment(TrianglePhase phase, int startIdx, int endIdx)
{
    if (startIdx >= endIdx || endIdx >= static_cast<int>(m_dataBuffer.size())) return;

    double segmentSlope = calculateSlope(startIdx, endIdx);
    qint64 duration = m_timestampBuffer[endIdx] - m_timestampBuffer[startIdx];

    switch (phase) {
        case TrianglePhase::Rising:
            m_currentCycle.risingSlope = segmentSlope;
            m_currentCycle.risingDuration = duration;

            // 添加到上升斜率历史
            m_risingSlopeHistory.push_back(segmentSlope);
            if (m_risingSlopeHistory.size() > TRIANGLE_SLOPE_HISTORY_SIZE) {
                m_risingSlopeHistory.pop_front();
            }

            LOG_DEBUG_CL("上升段分析: 斜率={:.2f}, 持续时间={}ms", segmentSlope, duration);
            break;

        case TrianglePhase::Falling:
            m_currentCycle.fallingSlope = segmentSlope; // 保持原始符号（负数）
            m_currentCycle.fallingDuration = duration;

            // 添加到下降斜率历史（取绝对值）
            m_fallingSlopeHistory.push_back(std::abs(segmentSlope));
            if (m_fallingSlopeHistory.size() > TRIANGLE_SLOPE_HISTORY_SIZE) {
                m_fallingSlopeHistory.pop_front();
            }

            LOG_DEBUG_CL("下降段分析: 斜率={:.2f}, 持续时间={}ms", segmentSlope, duration);
            break;

        default:
            // 峰值和谷值段通常很短，不需要特别分析
            break;
    }
}

// 新增：开始新周期
void OptimizedTriangleAnomalyDetector::startNewCycle(qint64 currentTime)
{
    // 完成上一个周期（如果存在）
    if (m_currentCycle.startTime > 0) {
        completeCycle(currentTime);
    }

    // 初始化新周期
    m_currentCycle = CycleData();
    m_currentCycle.startTime = currentTime;
    m_currentCycle.valleyValue = m_dataBuffer.back();
    m_cycleStartTime = currentTime;

    LOG_DEBUG_CL("开始新周期: 谷值={}, 时间={}", m_currentCycle.valleyValue, currentTime);
}

// 新增：记录波峰值
void OptimizedTriangleAnomalyDetector::recordPeakValue(uint16_t peakValue, qint64 peakTime)
{
    m_currentCycle.peakValue = peakValue;
    m_currentCycle.peakTime = peakTime;

    // 添加到波峰历史
    m_peakHistory.push_back(peakValue);
    if (m_peakHistory.size() > TRIANGLE_PEAK_VALLEY_HISTORY_SIZE) {
        m_peakHistory.pop_front();
    }

    LOG_DEBUG_CL("记录波峰: 值={}, 时间={}", peakValue, peakTime);
}

// 新增：记录波谷值
void OptimizedTriangleAnomalyDetector::recordValleyValue(uint16_t valleyValue, qint64 valleyTime)
{
    m_currentCycle.valleyValue = valleyValue;
    m_currentCycle.valleyTime = valleyTime;

    // 添加到波谷历史
    m_valleyHistory.push_back(valleyValue);
    if (m_valleyHistory.size() > TRIANGLE_PEAK_VALLEY_HISTORY_SIZE) {
        m_valleyHistory.pop_front();
    }

    LOG_DEBUG_CL("记录波谷: 值={}, 时间={}", valleyValue, valleyTime);
}

// 新增：完成周期
void OptimizedTriangleAnomalyDetector::completeCycle(qint64 currentTime)
{
    // 完成当前周期数据
    m_currentCycle.endTime = currentTime;
    m_currentCycle.duration = currentTime - m_currentCycle.startTime;

    // 添加到周期历史
    m_periodHistory.push_back(m_currentCycle.duration);
    if (m_periodHistory.size() > TRIANGLE_PERIOD_HISTORY_SIZE) {
        m_periodHistory.pop_front();
    }

    // 验证周期有效性
    m_currentCycle.isValid = isValidCycle(m_currentCycle);

    if (m_currentCycle.isValid) {
        m_currentStats.completedCycles++;

        // 更新当前统计
        updateCurrentStatistics();

        LOG_DEBUG_CL("完成有效周期 #{}: 时长={}ms, 频率={:.3f}Hz, 上升斜率={:.2f}, 下降斜率={:.2f}, 峰值={}, 谷值={}",
                    m_currentStats.completedCycles,
                    m_currentCycle.duration,
                    m_currentStats.currentFrequency,
                    m_currentCycle.risingSlope,
                    m_currentCycle.fallingSlope,
                    m_currentCycle.peakValue,
                    m_currentCycle.valleyValue);
    } else {
        LOG_DEBUG_CL("完成无效周期: 时长={}ms, 原因=验证失败", m_currentCycle.duration);
    }
}

// 新增：更新当前统计信息
void OptimizedTriangleAnomalyDetector::updateCurrentStatistics()
{
    // 更新周期和频率统计
    m_currentStats.currentPeriodMs = m_currentCycle.duration;
    if (m_currentCycle.duration > 0) {
        m_currentStats.currentFrequency = 1000.0 / m_currentCycle.duration;
    }

    // 更新斜率统计
    m_currentStats.currentRisingSlope = m_currentCycle.risingSlope;
    m_currentStats.currentFallingSlope = std::abs(m_currentCycle.fallingSlope);

    // 更新峰谷统计
    m_currentStats.currentPeakValue = m_currentCycle.peakValue;
    m_currentStats.currentValleyValue = m_currentCycle.valleyValue;
}

// 补充：更新统计信息
void OptimizedTriangleAnomalyDetector::updateStatistics()
{
    // 更新噪声水平
    m_currentStats.noiseLevel = calculateNoiseLevel();

    // 如果有足够的历史数据，计算平均值
    if (!m_currentStats.isLearning) {
        // 计算基准统计信息的当前偏差（用于监控）
        if (!m_risingSlopeHistory.empty()) {
            double avgRisingSlope = std::accumulate(m_risingSlopeHistory.begin(),
                                                  m_risingSlopeHistory.end(), 0.0) / m_risingSlopeHistory.size();
            // 可以用这个信息进行实时监控
        }

        if (!m_fallingSlopeHistory.empty()) {
            double avgFallingSlope = std::accumulate(m_fallingSlopeHistory.begin(),
                                                   m_fallingSlopeHistory.end(), 0.0) / m_fallingSlopeHistory.size();
            // 可以用这个信息进行实时监控
        }

        if (!m_periodHistory.empty()) {
            double avgPeriod = std::accumulate(m_periodHistory.begin(),
                                             m_periodHistory.end(), 0.0) / m_periodHistory.size();
            // 可以用这个信息进行实时监控
        }
    }
}
