
// OptimizedTriangleAnomalyDetector.cpp
#include "OptimizedTriangleAnomalyDetector.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include "MCLogger.h"

OptimizedTriangleAnomalyDetector::OptimizedTriangleAnomalyDetector(QObject *parent)
    : QObject(parent)
    , m_nominalFrequency(NOMINAL_FREQUENCY)
    , m_expectedMin(EXPECTED_MIN)
    , m_expectedMax(EXPECTED_MAX)
    , m_samplingRate(SAMPLING_RATE)
    , m_frequencyTolerance(FREQUENCY_TOLERANCE)
    , m_amplitudeTolerance(AMPLITUDE_TOLERANCE)
    , m_slopeTolerance(SLOPE_TOLERANCE)
    , m_asymmetryTolerance(ASYMMETRY_TOLERANCE)
    , m_linearityThreshold(LINEARITY_THRESHOLD)
    , m_previousPhase(TrianglePhase::Unknown)
    , m_phaseStartIndex(0)
    , m_phaseStartTime(0)
    , m_lastPeakValue(0)
    , m_lastValleyValue(65535)
    , m_lastPeakTime(0)
    , m_lastValleyTime(0)
    , m_adaptiveEnabled(true)
    , m_baselineFrequency(NOMINAL_FREQUENCY)
    , m_baselineAmplitude(BASELINE_AMPLITUDE)
    , m_baselineRisingSlope(0)
    , m_baselineFallingSlope(0)
    , m_baselineAsymmetryRatio(1.0)
    , m_learningPeriod(LEARNING_PERIOD)
    , m_samplesProcessed(0)
    , m_learningCompleted(false)
    , m_isRecording(false)
    , m_recordingDuration(10)
    , m_recordedDataPoints(0)
    , m_analysisCounter(0)
{
    // 初始化定时器
    m_recordingTimer = new QTimer(this);
    m_recordingTimer->setSingleShot(true);
    connect(m_recordingTimer, &QTimer::timeout, this, &OptimizedTriangleAnomalyDetector::onRecordingTimeout);

    // 默认启用所有异常类型
    m_enabledAnomalies[TriangleAnomalyType::FrequencyDrift] = true;
    m_enabledAnomalies[TriangleAnomalyType::AmplitudeDrift] = true;
    m_enabledAnomalies[TriangleAnomalyType::RisingSlopeError] = true;
    m_enabledAnomalies[TriangleAnomalyType::FallingSlopeError] = true;
    m_enabledAnomalies[TriangleAnomalyType::AsymmetryError] = true;
    m_enabledAnomalies[TriangleAnomalyType::PeakValueError] = true;
    m_enabledAnomalies[TriangleAnomalyType::ValleyValueError] = true;
    m_enabledAnomalies[TriangleAnomalyType::PeriodDistortion] = true;
    m_enabledAnomalies[TriangleAnomalyType::LinearityError] = true;
    m_enabledAnomalies[TriangleAnomalyType::NoiseSpike] = true;
    m_enabledAnomalies[TriangleAnomalyType::PhaseJump] = true;
    m_enabledAnomalies[TriangleAnomalyType::Saturation] = true;
    m_enabledAnomalies[TriangleAnomalyType::Dropout] = true;

    LOG_INFO("三角波异常检测器初始化完成");
    LOG_INFO("标称频率: {} Hz, 学习周期: {} 样本", m_nominalFrequency, m_learningPeriod);
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

    // 维护缓冲区大小
    int maxBufferSize = static_cast<int>(m_samplingRate * 3);  // 3秒的数据
    if (m_dataBuffer.size() > maxBufferSize) {
        m_dataBuffer.pop_front();
        m_timestampBuffer.pop_front();
    }

    m_samplesProcessed++;

    // 如果要记录，发送数据
    if (m_isRecording) {
        emit recordingData(value, currentTime);
        m_recordedDataPoints++;
    }

    // 性能优化：不是每个点都进行分析
    m_analysisCounter++;
    if (m_analysisCounter >= ANALYSIS_INTERVAL) {
        m_analysisCounter = 0;

        if (m_dataBuffer.size() >= 50) {  // 至少50个点才开始分析
            // 检测相位转换
            detectPhaseTransition();

            // 更新统计信息
            updateTriangleStats();

            // 学习阶段
            if (!m_learningCompleted) {
                updateBaseline();

                // 发送学习进度
                emit learningProgressUpdated(m_samplesProcessed, m_learningPeriod);

                // 检查学习是否完成
                if (m_samplesProcessed >= m_learningPeriod) {
                    finalizeBaseline();
                    m_learningCompleted = true;
                    emit learningCompleted(m_currentStats);
                    LOG_INFO("三角波参数学习完成，开始异常检测");
                }
            } else {
                // 执行异常检测
                TriangleAnomalyResult anomaly = analyzeTriangle();

                if (anomaly.type != TriangleAnomalyType::None) {
                    m_lastAnomaly = anomaly;
                    emit anomalyDetected(anomaly);

                    // 开始记录（如果未在记录中）
                    if (!m_isRecording) {
                        m_isRecording = true;
                        m_recordedDataPoints = 0;
                        m_recordingTimer->start(m_recordingDuration * 1000);
                        emit recordingStarted(anomaly);

                        LOG_INFO("检测到三角波异常: {}, 严重程度: {}%",
                                static_cast<int>(anomaly.type), int(anomaly.severity * 100));
                    }
                }
            }

            // 定期发送统计更新
            if (m_samplesProcessed % 100 == 0) {
                emit statsUpdated(m_currentStats);
            }
        }
    }
}

void OptimizedTriangleAnomalyDetector::setOptimalParameters()
{
    // 根据传输速率计算采样率
    double dataRate = 5.0; // 5KB/s
    double samplingRate = (dataRate * 1024) / 2; // 每2字节一个样本 = 2560 Hz
    setSamplingRate(samplingRate);

    // 设置自适应学习
    setAdaptiveThresholds(true);

    // 频率参数（让程序自动学习）
    setNominalFrequency(0);  // 0 表示自动学习

    // 幅度参数（全范围，自动缩小）
    setExpectedAmplitude(0, 4095);

    // 检测阈值
    setDetectionThresholds(
        0.05,   // 频率容差 5%
        0.10,   // 幅度容差 10%
        0.15,   // 斜率容差 15%
        0.20    // 非对称容差 20%
    );

    // 学习周期
    setLearningPeriod(2000);

    // 记录参数
    setRecordingDuration(10);

    // 启用所有异常检测
    for (int i = 1; i <= 13; i++) {
        enableAnomalyType(static_cast<TriangleAnomalyType>(i), true);
    }

    LOG_INFO("三角波检测器优化配置完成");
    LOG_INFO("采样率: {} Hz, 学习周期: {} 样本", samplingRate, m_learningPeriod);
}

void OptimizedTriangleAnomalyDetector::detectPhaseTransition()
{
    if (m_dataBuffer.size() < 10) return;

    size_t currentIdx = m_dataBuffer.size() - 1;
    TrianglePhase newPhase = determinePhase(currentIdx);

    // 检测相位变化
    if (newPhase != m_previousPhase && newPhase != TrianglePhase::Unknown) {
        qint64 currentTime = m_timestampBuffer.back();

        // 如果有有效的前一相位，创建相位段
        if (m_previousPhase != TrianglePhase::Unknown && m_phaseStartIndex < currentIdx) {
            PhaseSegment segment;
            segment.phase = m_previousPhase;
            segment.startIndex = m_phaseStartIndex;
            segment.endIndex = currentIdx - 1;
            segment.startTime = m_phaseStartTime;
            segment.endTime = currentTime;
            segment.startValue = m_dataBuffer[m_phaseStartIndex];
            segment.endValue = m_dataBuffer[currentIdx - 1];

            // 计算斜率和线性度
            segment.slope = calculateSegmentSlope(segment.startIndex, segment.endIndex);
            segment.linearity = calculateSegmentLinearity(segment.startIndex, segment.endIndex);

            // 分析相位段
            analyzePhaseSegment(segment);

            // 保存到最近段历史
            m_recentSegments.push_back(segment);
            if (m_recentSegments.size() > 10) {
                m_recentSegments.pop_front();
            }
        }

        // 更新相位信息
        m_previousPhase = newPhase;
        m_phaseStartIndex = currentIdx;
        m_phaseStartTime = currentTime;

        // 记录峰值和谷值
        if (newPhase == TrianglePhase::AtPeak ||
           (m_previousPhase == TrianglePhase::Rising && newPhase == TrianglePhase::Falling)) {
            m_lastPeakValue = m_dataBuffer.back();
            m_lastPeakTime = currentTime;
        } else if (newPhase == TrianglePhase::AtValley ||
                  (m_previousPhase == TrianglePhase::Falling && newPhase == TrianglePhase::Rising)) {
            m_lastValleyValue = m_dataBuffer.back();
            m_lastValleyTime = currentTime;
            m_currentStats.cycleCount++;

            // 完成一个周期，处理周期统计
            if (m_lastPeakTime > 0 && m_lastValleyTime > m_lastPeakTime) {
                processCompleteCycle();
            }
        }
    }

    m_currentStats.currentPhase = newPhase;
}

TrianglePhase OptimizedTriangleAnomalyDetector::determinePhase(int currentIdx)
{
    if (currentIdx < 10) return TrianglePhase::Unknown;

    // 分析最近几个点的趋势
    const int windowSize = 5;
    int risingCount = 0, fallingCount = 0;

    for (int i = currentIdx - windowSize; i < currentIdx; i++) {
        if (i > 0) {
            if (m_dataBuffer[i] > m_dataBuffer[i-1]) risingCount++;
            else if (m_dataBuffer[i] < m_dataBuffer[i-1]) fallingCount++;
        }
    }

    // 计算最近的斜率
    double recentSlope = calculateSlope(currentIdx - windowSize, currentIdx);
    uint16_t currentValue = m_dataBuffer[currentIdx];

    // 判断是否接近峰值或谷值
    bool nearPeak = (currentValue > m_expectedMax * 0.9);
    bool nearValley = (currentValue < m_expectedMin + (m_expectedMax - m_expectedMin) * 0.1);

    if (nearPeak && std::abs(recentSlope) < 10) {
        return TrianglePhase::AtPeak;
    } else if (nearValley && std::abs(recentSlope) < 10) {
        return TrianglePhase::AtValley;
    } else if (risingCount > fallingCount && recentSlope > 20) {
        return TrianglePhase::Rising;
    } else if (fallingCount > risingCount && recentSlope < -20) {
        return TrianglePhase::Falling;
    }

    return TrianglePhase::Unknown;
}

void OptimizedTriangleAnomalyDetector::analyzePhaseSegment(const PhaseSegment& segment)
{
    if (segment.phase == TrianglePhase::Rising) {
        m_risingSlopeHistory.push_back(segment.slope);
        if (m_risingSlopeHistory.size() > 20) {
            m_risingSlopeHistory.pop_front();
        }
        m_currentStats.risingSlope = segment.slope;
        m_currentStats.risingLinearity = segment.linearity;
        m_currentStats.risingDuration = segment.endTime - segment.startTime;
    } else if (segment.phase == TrianglePhase::Falling) {
        m_fallingSlopeHistory.push_back(std::abs(segment.slope)); // 取绝对值
        if (m_fallingSlopeHistory.size() > 20) {
            m_fallingSlopeHistory.pop_front();
        }
        m_currentStats.fallingSlope = std::abs(segment.slope);
        m_currentStats.fallingLinearity = segment.linearity;
        m_currentStats.fallingDuration = segment.endTime - segment.startTime;
    }
}

void OptimizedTriangleAnomalyDetector::processCompleteCycle()
{
    // 计算周期时间和频率
    qint64 cycleDuration = m_lastValleyTime - m_lastPeakTime + (m_lastPeakTime - m_lastValleyTime);
    m_currentStats.totalPeriod = cycleDuration;

    if (cycleDuration > 0) {
        double cycleFreq = 1000.0 / cycleDuration;  // Hz
        m_frequencyHistory.push_back(cycleFreq);
        if (m_frequencyHistory.size() > 20) {
            m_frequencyHistory.pop_front();
        }
    }

    // 计算非对称比例
    if (m_currentStats.fallingDuration > 0) {
        double asymmetry = static_cast<double>(m_currentStats.risingDuration) / m_currentStats.fallingDuration;
        m_asymmetryHistory.push_back(asymmetry);
        if (m_asymmetryHistory.size() > 20) {
            m_asymmetryHistory.pop_front();
        }
        m_currentStats.asymmetryRatio = asymmetry;
    }

    // 计算幅度
    if (m_lastPeakValue > m_lastValleyValue) {
        uint16_t amplitude = m_lastPeakValue - m_lastValleyValue;
        m_amplitudeHistory.push_back(amplitude);
        if (m_amplitudeHistory.size() > 20) {
            m_amplitudeHistory.pop_front();
        }
        m_currentStats.currentMin = m_lastValleyValue;
        m_currentStats.currentMax = m_lastPeakValue;
    }
}

TriangleAnomalyResult OptimizedTriangleAnomalyDetector::analyzeTriangle()
{
    TriangleAnomalyResult result;
    double maxSeverity = 0.0;
    double severity;

    // 频率漂移检测
    if (m_enabledAnomalies[TriangleAnomalyType::FrequencyDrift] && detectFrequencyDrift(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::FrequencyDrift;
            result.severity = severity;
            result.description = QString("频率漂移: 当前频率 %1 Hz, 基准频率 %2 Hz")
                               .arg(m_currentStats.currentFrequency, 0, 'f', 4)
                               .arg(m_baselineFrequency, 0, 'f', 4);
            maxSeverity = severity;
        }
    }

    // 上升沿斜率异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::RisingSlopeError] && detectRisingSlopeError(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::RisingSlopeError;
            result.severity = severity;
            result.description = QString("上升沿斜率异常: 当前斜率 %1, 基准斜率 %2")
                               .arg(m_currentStats.risingSlope, 0, 'f', 2)
                               .arg(m_baselineRisingSlope, 0, 'f', 2);
            maxSeverity = severity;
        }
    }

    // 下降沿斜率异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::FallingSlopeError] && detectFallingSlopeError(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::FallingSlopeError;
            result.severity = severity;
            result.description = QString("下降沿斜率异常: 当前斜率 %1, 基准斜率 %2")
                               .arg(m_currentStats.fallingSlope, 0, 'f', 2)
                               .arg(m_baselineFallingSlope, 0, 'f', 2);
            maxSeverity = severity;
        }
    }

    // 非对称比例异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::AsymmetryError] && detectAsymmetryError(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::AsymmetryError;
            result.severity = severity;
            result.description = QString("非对称比例异常: 当前比例 %1, 基准比例 %2")
                               .arg(m_currentStats.asymmetryRatio, 0, 'f', 3)
                               .arg(m_baselineAsymmetryRatio, 0, 'f', 3);
            maxSeverity = severity;
        }
    }

    // 幅度漂移检测
    if (m_enabledAnomalies[TriangleAnomalyType::AmplitudeDrift] && detectAmplitudeDrift(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::AmplitudeDrift;
            result.severity = severity;
            result.description = QString("幅度异常: 当前幅度 %1, 基准幅度 %2")
                               .arg(m_currentStats.averageAmplitude)
                               .arg(m_baselineAmplitude);
            maxSeverity = severity;
        }
    }

    // 线性度误差检测
    if (m_enabledAnomalies[TriangleAnomalyType::LinearityError] && detectLinearityError(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::LinearityError;
            result.severity = severity;
            result.description = QString("线性度异常: 上升线性度 %1%, 下降线性度 %2%")
                               .arg(int(m_currentStats.risingLinearity * 100))
                               .arg(int(m_currentStats.fallingLinearity * 100));
            maxSeverity = severity;
        }
    }

    // 噪声尖峰检测
    if (m_enabledAnomalies[TriangleAnomalyType::NoiseSpike] && detectNoiseSpike(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::NoiseSpike;
            result.severity = severity;
            result.description = QString("噪声尖峰: 噪声水平 %1")
                               .arg(m_currentStats.noiseLevel, 0, 'f', 1);
            maxSeverity = severity;
        }
    }

    // 填充结果信息
    if (result.type != TriangleAnomalyType::None) {
        result.triggerValue = m_dataBuffer.back();
        result.timestamp = QDateTime::currentMSecsSinceEpoch();
        result.phaseWhenDetected = m_currentStats.currentPhase;
    }

    return result;
}

// 异常检测函数实现
bool OptimizedTriangleAnomalyDetector::detectFrequencyDrift(double& severity)
{
    if (m_frequencyHistory.empty() || m_baselineFrequency <= 0) return false;

    double currentFreq = m_currentStats.currentFrequency;
    double deviation = std::abs(currentFreq - m_baselineFrequency) / m_baselineFrequency;

    if (deviation > m_frequencyTolerance) {
        severity = std::min(deviation / m_frequencyTolerance, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectRisingSlopeError(double& severity)
{
    if (m_risingSlopeHistory.empty() || m_baselineRisingSlope <= 0) return false;

    double currentSlope = m_currentStats.risingSlope;
    double deviation = std::abs(currentSlope - m_baselineRisingSlope) / m_baselineRisingSlope;

    if (deviation > m_slopeTolerance) {
        severity = std::min(deviation / m_slopeTolerance, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectFallingSlopeError(double& severity)
{
    if (m_fallingSlopeHistory.empty() || m_baselineFallingSlope <= 0) return false;

    double currentSlope = m_currentStats.fallingSlope;
    double deviation = std::abs(currentSlope - m_baselineFallingSlope) / m_baselineFallingSlope;

    if (deviation > m_slopeTolerance) {
        severity = std::min(deviation / m_slopeTolerance, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectAsymmetryError(double& severity)
{
    if (m_asymmetryHistory.empty() || m_baselineAsymmetryRatio <= 0) return false;

    double currentAsymmetry = m_currentStats.asymmetryRatio;
    double deviation = std::abs(currentAsymmetry - m_baselineAsymmetryRatio) / m_baselineAsymmetryRatio;

    if (deviation > m_asymmetryTolerance) {
        severity = std::min(deviation / m_asymmetryTolerance, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectAmplitudeDrift(double& severity)
{
    if (m_baselineAmplitude <= 0) return false;

    double currentAmp = m_currentStats.averageAmplitude;
    double deviation = std::abs(static_cast<double>(currentAmp) - m_baselineAmplitude) / m_baselineAmplitude;

    if (deviation > m_amplitudeTolerance) {
        severity = std::min(deviation / m_amplitudeTolerance, 1.0);
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectLinearityError(double& severity)
{
    double minLinearity = std::min(m_currentStats.risingLinearity, m_currentStats.fallingLinearity);

    if (minLinearity < m_linearityThreshold) {
        severity = (m_linearityThreshold - minLinearity) / m_linearityThreshold;
        return severity > 0.1;  // 至少10%的线性度降低才报告
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectNoiseSpike(double& severity)
{
    if (m_dataBuffer.size() < 10) return false;

    // 计算最近10个点的噪声水平
    double noise = 0.0;
    for (size_t i = m_dataBuffer.size() - 10; i < m_dataBuffer.size() - 1; i++) {
        noise += std::abs(static_cast<double>(m_dataBuffer[i+1]) - m_dataBuffer[i]);
    }
    noise /= 9;

    m_currentStats.noiseLevel = noise;

    // 基于当前幅度的相对噪声阈值
    double noiseThreshold = m_currentStats.averageAmplitude * 0.05;  // 5% of amplitude

    if (noise > noiseThreshold) {
        severity = std::min(noise / noiseThreshold, 1.0);
        return true;
    }
    return false;
}

// 其他检测函数的简单实现
bool OptimizedTriangleAnomalyDetector::detectPeakValueError(double& severity)
{
    uint16_t expectedPeak = m_expectedMax;
    if (expectedPeak > 0 && m_lastPeakValue > 0) {
        double deviation = std::abs(static_cast<double>(m_lastPeakValue) - expectedPeak) / expectedPeak;
        if (deviation > 0.1) {
            severity = std::min(deviation / 0.1, 1.0);
            return true;
        }
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectValleyValueError(double& severity)
{
    uint16_t expectedValley = m_expectedMin;
    if (m_lastValleyValue < 65535) {
        double deviation = std::abs(static_cast<double>(m_lastValleyValue) - expectedValley);
        if (deviation > 100) {  // 允许100的偏差
            severity = std::min(deviation / 200, 1.0);
            return true;
        }
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectPeriodDistortion(double& severity)
{
    if (m_currentStats.totalPeriod > 0 && m_baselineFrequency > 0) {
        qint64 expectedPeriod = 1000 / m_baselineFrequency;  // ms
        double deviation = std::abs(static_cast<double>(m_currentStats.totalPeriod) - expectedPeriod) / expectedPeriod;
        if (deviation > 0.2) {  // 20%的周期偏差
            severity = std::min(deviation / 0.2, 1.0);
            return true;
        }
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectPhaseJump(double& severity)
{
    // 简单实现：检测相位转换时间的异常
    if (m_frequencyHistory.size() < 3) return false;

    double lastFreq = m_frequencyHistory.back();
    double avgFreq = m_currentStats.averageFrequency;

    if (avgFreq > 0) {
        double jump = std::abs(lastFreq - avgFreq) / avgFreq;
        if (jump > 0.3) {  // 30%的跳变
            severity = std::min(jump / 0.3, 1.0);
            return true;
        }
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectSaturation(double& severity)
{
    uint16_t currentValue = m_dataBuffer.back();

    // 检测饱和（接近极值）
    if (currentValue <= 50 || currentValue >= 4045) {
        severity = 0.9;
        return true;
    }
    return false;
}

bool OptimizedTriangleAnomalyDetector::detectDropout(double& severity)
{
    if (m_dataBuffer.size() < 20) return false;

    // 检测连续相同值（数据丢失）
    uint16_t currentValue = m_dataBuffer.back();
    int consecutiveCount = 1;

    for (int i = m_dataBuffer.size() - 2; i >= std::max(0, (int)m_dataBuffer.size() - 20); i--) {
        if (m_dataBuffer[i] == currentValue) {
            consecutiveCount++;
        } else {
            break;
        }
    }

    if (consecutiveCount > 15) {  // 连续15个相同值
        severity = std::min(consecutiveCount / 30.0, 1.0);
        return true;
    }
    return false;
}

void OptimizedTriangleAnomalyDetector::updateTriangleStats()
{
    // 更新频率统计
    m_currentStats.currentFrequency = calculateCurrentFrequency();

    if (!m_frequencyHistory.empty()) {
        m_currentStats.averageFrequency = std::accumulate(m_frequencyHistory.begin(),
                                                         m_frequencyHistory.end(), 0.0) / m_frequencyHistory.size();

        // 计算频率稳定性
        double variance = 0.0;
        for (double freq : m_frequencyHistory) {
            variance += std::pow(freq - m_currentStats.averageFrequency, 2);
        }
        variance /= m_frequencyHistory.size();
        m_currentStats.frequencyStability = variance > 0 ? 1.0 / std::sqrt(variance) : 1.0;
    }

    // 更新斜率统计
    if (!m_risingSlopeHistory.empty()) {
        m_currentStats.averageRisingSlope = std::accumulate(m_risingSlopeHistory.begin(),
                                                          m_risingSlopeHistory.end(), 0.0) / m_risingSlopeHistory.size();
    }

    if (!m_fallingSlopeHistory.empty()) {
        m_currentStats.averageFallingSlope = std::accumulate(m_fallingSlopeHistory.begin(),
                                                           m_fallingSlopeHistory.end(), 0.0) / m_fallingSlopeHistory.size();
    }

    // 更新非对称比例统计
    if (!m_asymmetryHistory.empty()) {
        m_currentStats.averageAsymmetryRatio = std::accumulate(m_asymmetryHistory.begin(),
                                                             m_asymmetryHistory.end(), 0.0) / m_asymmetryHistory.size();
    }

    // 更新幅度统计
    if (!m_amplitudeHistory.empty()) {
        m_currentStats.averageAmplitude = std::accumulate(m_amplitudeHistory.begin(),
                                                        m_amplitudeHistory.end(), 0) / m_amplitudeHistory.size();
    }
}

double OptimizedTriangleAnomalyDetector::calculateCurrentFrequency()
{
    if (m_frequencyHistory.empty()) return 0.0;

    // 返回最近几个频率测量的平均值
    int recentCount = std::min(5, static_cast<int>(m_frequencyHistory.size()));
    double sum = 0.0;

    for (int i = m_frequencyHistory.size() - recentCount; i < static_cast<int>(m_frequencyHistory.size()); i++) {
        sum += m_frequencyHistory[i];
    }

    return sum / recentCount;
}

void OptimizedTriangleAnomalyDetector::updateBaseline()
{
    // 需要足够样本才开始学习
    if (m_samplesProcessed < 200) return;

    // 学习频率
    if (!m_frequencyHistory.empty()) {
        std::vector<double> sortedFreq(m_frequencyHistory.begin(), m_frequencyHistory.end());
        std::sort(sortedFreq.begin(), sortedFreq.end());
        double medianFreq = sortedFreq[sortedFreq.size() / 2];

        if (m_nominalFrequency == 0 || m_adaptiveEnabled) {
            m_baselineFrequency = medianFreq;
            if (m_nominalFrequency == 0) {
                m_nominalFrequency = medianFreq;
            }
        }
    }

    // 学习斜率
    if (!m_risingSlopeHistory.empty()) {
        m_baselineRisingSlope = std::accumulate(m_risingSlopeHistory.begin(),
                                              m_risingSlopeHistory.end(), 0.0) / m_risingSlopeHistory.size();
    }

    if (!m_fallingSlopeHistory.empty()) {
        m_baselineFallingSlope = std::accumulate(m_fallingSlopeHistory.begin(),
                                               m_fallingSlopeHistory.end(), 0.0) / m_fallingSlopeHistory.size();
    }

    // 学习非对称比例
    if (!m_asymmetryHistory.empty()) {
        m_baselineAsymmetryRatio = std::accumulate(m_asymmetryHistory.begin(),
                                                 m_asymmetryHistory.end(), 0.0) / m_asymmetryHistory.size();
    }

    // 学习幅度
    if (!m_amplitudeHistory.empty()) {
        m_baselineAmplitude = std::accumulate(m_amplitudeHistory.begin(),
                                            m_amplitudeHistory.end(), 0) / m_amplitudeHistory.size();
    }
}

void OptimizedTriangleAnomalyDetector::finalizeBaseline()
{
    LOG_INFO("=== 三角波参数学习完成 ===");
    LOG_INFO("学习到的基准频率: {} Hz", m_baselineFrequency);
    LOG_INFO("学习到的基准幅度: {}", m_baselineAmplitude);
    LOG_INFO("学习到的上升斜率: {}", m_baselineRisingSlope);
    LOG_INFO("学习到的下降斜率: {}", m_baselineFallingSlope);
    LOG_INFO("学习到的非对称比例: {}", m_baselineAsymmetryRatio);
    LOG_INFO("值范围: {} - {}", m_currentStats.currentMin, m_currentStats.currentMax);
    LOG_INFO("频率稳定性指数: {}", m_currentStats.frequencyStability);
    LOG_INFO("===============================");
}

// 辅助函数实现
double OptimizedTriangleAnomalyDetector::calculateSlope(int startIdx, int endIdx)
{
    if (startIdx >= endIdx || endIdx >= static_cast<int>(m_dataBuffer.size())) return 0.0;

    double deltaY = m_dataBuffer[endIdx] - m_dataBuffer[startIdx];
    double deltaX = endIdx - startIdx;

    return deltaX > 0 ? deltaY / deltaX : 0.0;
}

double OptimizedTriangleAnomalyDetector::calculateSegmentSlope(int startIdx, int endIdx)
{
    return calculateSlope(startIdx, endIdx);
}

double OptimizedTriangleAnomalyDetector::calculateSegmentLinearity(int startIdx, int endIdx)
{
    if (endIdx - startIdx < 10) return 1.0;

    // 创建理想直线
    std::vector<double> x, y;
    for (int i = startIdx; i <= endIdx; i++) {
        x.push_back(i - startIdx);
        y.push_back(m_dataBuffer[i]);
    }

    // 计算与理想直线的相关系数
    return calculateCorrelation(x, y);
}

double OptimizedTriangleAnomalyDetector::calculateCorrelation(const std::vector<double>& x, const std::vector<double>& y)
{
    if (x.size() != y.size() || x.empty()) return 0.0;

    double meanX = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
    double meanY = std::accumulate(y.begin(), y.end(), 0.0) / y.size();

    double numerator = 0.0, denomX = 0.0, denomY = 0.0;

    for (size_t i = 0; i < x.size(); i++) {
        double dx = x[i] - meanX;
        double dy = y[i] - meanY;
        numerator += dx * dy;
        denomX += dx * dx;
        denomY += dy * dy;
    }

    double denom = std::sqrt(denomX * denomY);
    return denom > 0 ? std::abs(numerator / denom) : 0.0;
}

void OptimizedTriangleAnomalyDetector::onRecordingTimeout()
{
    if (m_isRecording) {
        m_isRecording = false;
        emit recordingStopped(m_recordedDataPoints);
        LOG_INFO("三角波异常记录结束，共记录 {} 个数据点", m_recordedDataPoints);
    }
}

// 配置函数实现
void OptimizedTriangleAnomalyDetector::setNominalFrequency(double frequency)
{
    m_nominalFrequency = frequency;
    m_baselineFrequency = frequency;
    LOG_INFO("设置标称频率: {} Hz", frequency);
}

void OptimizedTriangleAnomalyDetector::setExpectedAmplitude(uint16_t minVal, uint16_t maxVal)
{
    m_expectedMin = minVal;
    m_expectedMax = maxVal;
    m_baselineAmplitude = maxVal - minVal;
    LOG_INFO("设置期望幅度范围: {} - {} (幅度: {})", minVal, maxVal, m_baselineAmplitude);
}

void OptimizedTriangleAnomalyDetector::setDetectionThresholds(double freqTolerance, double ampTolerance,
                                                             double slopeTolerance, double asymmetryTolerance)
{
    m_frequencyTolerance = freqTolerance;
    m_amplitudeTolerance = ampTolerance;
    m_slopeTolerance = slopeTolerance;
    m_asymmetryTolerance = asymmetryTolerance;

    LOG_INFO("设置检测阈值: 频率容差={}%, 幅度容差={}%, 斜率容差={}%, 非对称容差={}%",
             freqTolerance * 100, ampTolerance * 100, slopeTolerance * 100, asymmetryTolerance * 100);
}

void OptimizedTriangleAnomalyDetector::setSamplingRate(double sampleRate)
{
    m_samplingRate = sampleRate;
    LOG_INFO("设置采样率: {} Hz", sampleRate);
}

void OptimizedTriangleAnomalyDetector::setRecordingDuration(int seconds)
{
    m_recordingDuration = seconds;
    LOG_INFO("设置记录持续时间: {} 秒", seconds);
}

void OptimizedTriangleAnomalyDetector::enableAnomalyType(TriangleAnomalyType type, bool enabled)
{
    m_enabledAnomalies[type] = enabled;
    LOG_INFO("异常类型 {}: {}", static_cast<int>(type), enabled ? "启用" : "禁用");
}

void OptimizedTriangleAnomalyDetector::setAdaptiveThresholds(bool enabled)
{
    m_adaptiveEnabled = enabled;
    LOG_INFO("自适应阈值: {}", enabled ? "启用" : "禁用");
}

void OptimizedTriangleAnomalyDetector::reset()
{
    // 停止记录
    if (m_isRecording) {
        m_recordingTimer->stop();
        m_isRecording = false;
        emit recordingStopped(m_recordedDataPoints);
    }

    // 清空缓冲区
    m_dataBuffer.clear();
    m_timestampBuffer.clear();
    m_frequencyHistory.clear();
    m_amplitudeHistory.clear();
    m_risingSlopeHistory.clear();
    m_fallingSlopeHistory.clear();
    m_asymmetryHistory.clear();
    m_recentSegments.clear();

    // 重置状态
    m_currentStats = TriangleStats();
    m_lastAnomaly = TriangleAnomalyResult();
    m_previousPhase = TrianglePhase::Unknown;
    m_phaseStartIndex = 0;
    m_samplesProcessed = 0;
    m_analysisCounter = 0;
    m_recordedDataPoints = 0;
    m_learningCompleted = false;

    LOG_INFO("三角波异常检测器已重置");
}
