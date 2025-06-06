// AsymmetricTriangleWaveDetector.cpp
#include "AsymmetricTriangleWaveDetector.h"
#include <algorithm>
#include <numeric>
#include <cmath>

AsymmetricTriangleWaveDetector::AsymmetricTriangleWaveDetector(QObject *parent)
    : QObject(parent)
    , m_nominalFrequency(0)
    , m_expectedPeak(4095)
    , m_expectedValley(0)
    , m_expectedRisingSlope(0)
    , m_expectedFallingSlope(0)
    , m_expectedAsymmetryRatio(1.0)
    , m_samplingRate(10000.0)
    , m_recordingDuration(12)  // 12秒
    , m_frequencyTolerance(0.05)
    , m_peakTolerance(0.1)
    , m_slopeTolerance(0.1)
    , m_asymmetryTolerance(0.2)
    , m_previousPhase(TrianglePhase::Unknown)
    , m_phaseStartIndex(0)
    , m_phaseStartTime(0)
    , m_lastPeakValue(0)
    , m_lastValleyValue(65535)
    , m_lastPeakTime(0)
    , m_lastValleyTime(0)
    , m_adaptiveEnabled(true)
    , m_baselineFrequency(0)
    , m_baselinePeak(0)
    , m_baselineValley(65535)
    , m_baselineRisingSlope(0)
    , m_baselineFallingSlope(0)
    , m_baselineAsymmetryRatio(1.0)
    , m_learningPeriod(5000)  // 学习5000个样本
    , m_samplesProcessed(0)
    , m_isRecording(false)
    , m_recordedDataPoints(0)
    , m_analysisCounter(0)
{
    // 初始化定时器
    m_recordingTimer = new QTimer(this);
    m_recordingTimer->setSingleShot(true);
    connect(m_recordingTimer, &QTimer::timeout, this, &AsymmetricTriangleWaveDetector::onRecordingTimeout);

    // 默认启用所有异常类型
    m_enabledAnomalies[TriangleAnomalyType::FrequencyDrift] = true;
    m_enabledAnomalies[TriangleAnomalyType::PeakAnomaly] = true;
    m_enabledAnomalies[TriangleAnomalyType::ValleyAnomaly] = true;
    m_enabledAnomalies[TriangleAnomalyType::RisingSlopeAnomaly] = true;
    m_enabledAnomalies[TriangleAnomalyType::FallingSlopeAnomaly] = true;
    m_enabledAnomalies[TriangleAnomalyType::AsymmetryRatioAnomaly] = true;
    m_enabledAnomalies[TriangleAnomalyType::AmplitudeAnomaly] = true;
    m_enabledAnomalies[TriangleAnomalyType::NoiseAnomaly] = true;
    m_enabledAnomalies[TriangleAnomalyType::PhaseJump] = true;

    LOG_INFO("AsymmetricTriangleWaveDetector 初始化完成");
}

AsymmetricTriangleWaveDetector::~AsymmetricTriangleWaveDetector()
{
    if (m_isRecording) {
        m_recordingTimer->stop();
        emit recordingStopped(m_recordedDataPoints);
    }
}

void AsymmetricTriangleWaveDetector::feedData(uint16_t value)
{
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();

    // 添加数据到缓冲区
    m_dataBuffer.push_back(value);
    m_timestampBuffer.push_back(currentTime);

    // 维护缓冲区大小 (保留足够的数据用于分析)
    int maxBufferSize = static_cast<int>(m_samplingRate * 2);  // 2秒的数据
    if (m_dataBuffer.size() > maxBufferSize) {
        m_dataBuffer.pop_front();
        m_timestampBuffer.pop_front();
    }

    m_samplesProcessed++;

    // 如果正在记录，发送数据
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

            // 自适应学习阶段
            if (m_adaptiveEnabled && m_samplesProcessed < m_learningPeriod) {
                updateBaseline();
            } else {
                // 执行异常检测
                TriangleAnomalyResult anomaly = analyzeTriangleWave();

                if (anomaly.type != TriangleAnomalyType::None) {
                    m_lastAnomaly = anomaly;
                    emit anomalyDetected(anomaly);

                    // 开始记录（如果未在记录中）
                    if (!m_isRecording) {
                        m_isRecording = true;
                        m_recordedDataPoints = 0;
                        m_recordingTimer->start(m_recordingDuration * 1000);
                        emit recordingStarted(anomaly);

                        LOG_INFO("检测到非对称三角波异常: {}, 严重程度: {}",
                                static_cast<int>(anomaly.type), anomaly.severity);
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

void AsymmetricTriangleWaveDetector::detectPhaseTransition()
{
    if (m_dataBuffer.size() < 10) return;

    size_t currentIdx = m_dataBuffer.size() - 1;
    uint16_t currentValue = m_dataBuffer[currentIdx];

    // 使用移动平均来平滑噪声
    const int windowSize = 5;
    if (currentIdx < windowSize * 2) return;

    // 计算当前和之前的斜率
    double currentSlope = calculateSlope(currentIdx - windowSize, currentIdx);
    double prevSlope = calculateSlope(currentIdx - windowSize * 2, currentIdx - windowSize);

    TrianglePhase newPhase = m_currentStats.currentPhase;

    // 检测转折点
    bool isPeak = false;
    bool isValley = false;

    if (prevSlope > 0 && currentSlope <= 0) {
        // 可能是波峰
        isPeak = true;
        newPhase = TrianglePhase::Falling;
    } else if (prevSlope < 0 && currentSlope >= 0) {
        // 可能是波谷
        isValley = true;
        newPhase = TrianglePhase::Rising;
    } else if (currentSlope > 0) {
        newPhase = TrianglePhase::Rising;
    } else if (currentSlope < 0) {
        newPhase = TrianglePhase::Falling;
    }

    // 处理相位变化
    if (newPhase != m_previousPhase && newPhase != TrianglePhase::Unknown) {
        qint64 currentTime = m_timestampBuffer[currentIdx];

        if (m_previousPhase != TrianglePhase::Unknown) {
            // 计算相位持续时间
            double phaseDuration = (currentTime - m_phaseStartTime) / 1000.0;  // 转换为秒

            if (m_previousPhase == TrianglePhase::Rising) {
                m_currentStats.risingDuration = phaseDuration;
                updateSlopeHistory();
            } else if (m_previousPhase == TrianglePhase::Falling) {
                m_currentStats.fallingDuration = phaseDuration;
                updateSlopeHistory();

                // 完成一个周期
                if (m_currentStats.risingDuration > 0 && m_currentStats.fallingDuration > 0) {
                    double period = m_currentStats.risingDuration + m_currentStats.fallingDuration;
                    double frequency = 1.0 / period;
                    m_frequencyHistory.push_back(frequency);
                    if (m_frequencyHistory.size() > 20) {
                        m_frequencyHistory.pop_front();
                    }

                    calculateAsymmetryRatio();
                    m_currentStats.cycleCount++;
                }
            }
        }

        m_phaseStartTime = currentTime;
        m_phaseStartIndex = currentIdx;
        m_previousPhase = newPhase;

        // 记录峰值和谷值
        if (isPeak) {
            m_lastPeakValue = currentValue;
            m_lastPeakTime = currentTime;
            m_peakHistory.push_back(currentValue);
            if (m_peakHistory.size() > 10) {
                m_peakHistory.pop_front();
            }
        } else if (isValley) {
            m_lastValleyValue = currentValue;
            m_lastValleyTime = currentTime;
            m_valleyHistory.push_back(currentValue);
            if (m_valleyHistory.size() > 10) {
                m_valleyHistory.pop_front();
            }
        }
    }

    m_currentStats.currentPhase = newPhase;
}

double AsymmetricTriangleWaveDetector::calculateSlope(int startIdx, int endIdx)
{
    if (startIdx >= endIdx || endIdx >= static_cast<int>(m_dataBuffer.size())) {
        return 0.0;
    }

    // 使用最小二乘法计算斜率
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    int n = endIdx - startIdx + 1;

    for (int i = startIdx; i <= endIdx; i++) {
        double x = i - startIdx;
        double y = m_dataBuffer[i];
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

    // 转换为每秒的变化率
    return slope * m_samplingRate;
}

void AsymmetricTriangleWaveDetector::updateSlopeHistory()
{
    if (m_phaseStartIndex >= m_dataBuffer.size() - 5) return;

    double avgSlope = calculateSlope(m_phaseStartIndex, m_dataBuffer.size() - 5);

    if (m_previousPhase == TrianglePhase::Rising) {
        m_currentStats.currentRisingSlope = avgSlope;
        m_risingSlopeHistory.push_back(avgSlope);
        if (m_risingSlopeHistory.size() > 10) {
            m_risingSlopeHistory.pop_front();
        }
    } else if (m_previousPhase == TrianglePhase::Falling) {
        m_currentStats.currentFallingSlope = avgSlope;
        m_fallingSlopeHistory.push_back(std::abs(avgSlope));  // 存储绝对值
        if (m_fallingSlopeHistory.size() > 10) {
            m_fallingSlopeHistory.pop_front();
        }
    }
}

void AsymmetricTriangleWaveDetector::calculateAsymmetryRatio()
{
    if (m_currentStats.fallingDuration > 0) {
        m_currentStats.asymmetryRatio = m_currentStats.risingDuration / m_currentStats.fallingDuration;
        m_asymmetryHistory.push_back(m_currentStats.asymmetryRatio);
        if (m_asymmetryHistory.size() > 10) {
            m_asymmetryHistory.pop_front();
        }
    }
}

void AsymmetricTriangleWaveDetector::updateTriangleStats()
{
    // 更新频率
    if (!m_frequencyHistory.empty()) {
        m_currentStats.currentFrequency = m_frequencyHistory.back();
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

    // 更新峰值和谷值
    if (!m_peakHistory.empty()) {
        m_currentStats.currentPeak = m_lastPeakValue;
        uint32_t sumPeak = std::accumulate(m_peakHistory.begin(), m_peakHistory.end(), 0);
        m_currentStats.averagePeak = sumPeak / m_peakHistory.size();
    }

    if (!m_valleyHistory.empty()) {
        m_currentStats.currentValley = m_lastValleyValue;
        uint32_t sumValley = std::accumulate(m_valleyHistory.begin(), m_valleyHistory.end(), 0);
        m_currentStats.averageValley = sumValley / m_valleyHistory.size();
    }

    // 更新幅度
    if (m_currentStats.averagePeak > m_currentStats.averageValley) {
        m_currentStats.amplitude = m_currentStats.averagePeak - m_currentStats.averageValley;
    }

    // 更新斜率
    if (!m_risingSlopeHistory.empty()) {
        m_currentStats.averageRisingSlope = std::accumulate(m_risingSlopeHistory.begin(),
                                                           m_risingSlopeHistory.end(), 0.0) / m_risingSlopeHistory.size();
    }

    if (!m_fallingSlopeHistory.empty()) {
        m_currentStats.averageFallingSlope = std::accumulate(m_fallingSlopeHistory.begin(),
                                                            m_fallingSlopeHistory.end(), 0.0) / m_fallingSlopeHistory.size();
    }

    // 更新噪声水平
    if (m_dataBuffer.size() >= 10) {
        double noise = 0.0;
        for (size_t i = m_dataBuffer.size() - 10; i < m_dataBuffer.size() - 1; i++) {
            noise += std::abs(static_cast<double>(m_dataBuffer[i+1]) - m_dataBuffer[i]);
        }
        m_currentStats.noiseLevel = noise / 9.0;
    }
}

TriangleAnomalyResult AsymmetricTriangleWaveDetector::analyzeTriangleWave()
{
    TriangleAnomalyResult result;
    double maxSeverity = 0.0;
    double severity;

    // 频率漂移检测
    if (m_enabledAnomalies[TriangleAnomalyType::FrequencyDrift] && detectFrequencyDrift(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::FrequencyDrift;
            result.severity = severity;
            result.description = QString("频率漂移: 当前 %1 Hz, 基准 %2 Hz")
                               .arg(m_currentStats.currentFrequency, 0, 'f', 3)
                               .arg(m_baselineFrequency, 0, 'f', 3);
            maxSeverity = severity;
        }
    }

    // 波峰异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::PeakAnomaly] && detectPeakAnomaly(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::PeakAnomaly;
            result.severity = severity;
            result.description = QString("波峰异常: 当前 %1, 基准 %2")
                               .arg(m_currentStats.currentPeak)
                               .arg(m_baselinePeak);
            maxSeverity = severity;
        }
    }

    // 波谷异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::ValleyAnomaly] && detectValleyAnomaly(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::ValleyAnomaly;
            result.severity = severity;
            result.description = QString("波谷异常: 当前 %1, 基准 %2")
                               .arg(m_currentStats.currentValley)
                               .arg(m_baselineValley);
            maxSeverity = severity;
        }
    }

    // 上升斜率异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::RisingSlopeAnomaly] && detectRisingSlopeAnomaly(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::RisingSlopeAnomaly;
            result.severity = severity;
            result.description = QString("上升斜率异常: 当前 %1, 基准 %2")
                               .arg(m_currentStats.currentRisingSlope, 0, 'f', 1)
                               .arg(m_baselineRisingSlope, 0, 'f', 1);
            maxSeverity = severity;
        }
    }

    // 下降斜率异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::FallingSlopeAnomaly] && detectFallingSlopeAnomaly(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::FallingSlopeAnomaly;
            result.severity = severity;
            result.description = QString("下降斜率异常: 当前 %1, 基准 %2")
                               .arg(m_currentStats.currentFallingSlope, 0, 'f', 1)
                               .arg(m_baselineFallingSlope, 0, 'f', 1);
            maxSeverity = severity;
        }
    }

    // 非对称性异常检测
    if (m_enabledAnomalies[TriangleAnomalyType::AsymmetryRatioAnomaly] && detectAsymmetryAnomaly(severity)) {
        if (severity > maxSeverity) {
            result.type = TriangleAnomalyType::AsymmetryRatioAnomaly;
            result.severity = severity;
            result.description = QString("非对称比例异常: 当前 %1, 基准 %2")
                               .arg(m_currentStats.asymmetryRatio, 0, 'f', 2)
                               .arg(m_baselineAsymmetryRatio, 0, 'f', 2);
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

// 各种异常检测函数的实现
bool AsymmetricTriangleWaveDetector::detectFrequencyDrift(double& severity)
{
    if (m_frequencyHistory.empty() || m_baselineFrequency <= 0) return false;

    double deviation = std::abs(m_currentStats.currentFrequency - m_baselineFrequency) / m_baselineFrequency;

    if (deviation > m_frequencyTolerance) {
        severity = std::min(deviation / m_frequencyTolerance, 1.0);
        return true;
    }

    return false;
}

bool AsymmetricTriangleWaveDetector::detectPeakAnomaly(double& severity)
{
    if (m_peakHistory.empty() || m_baselinePeak <= 0) return false;

    double deviation = std::abs(static_cast<double>(m_currentStats.currentPeak) - m_baselinePeak) / m_baselinePeak;

    if (deviation > m_peakTolerance) {
        severity = std::min(deviation / m_peakTolerance, 1.0);
        return true;
    }

    return false;
}

bool AsymmetricTriangleWaveDetector::detectValleyAnomaly(double& severity)
{
    if (m_valleyHistory.empty()) return false;

    double range = m_baselinePeak - m_baselineValley;
    if (range <= 0) return false;

    double deviation = std::abs(static_cast<double>(m_currentStats.currentValley) - m_baselineValley) / range;

    if (deviation > m_peakTolerance) {
        severity = std::min(deviation / m_peakTolerance, 1.0);
        return true;
    }

    return false;
}

bool AsymmetricTriangleWaveDetector::detectRisingSlopeAnomaly(double& severity)
{
    if (m_risingSlopeHistory.empty() || m_baselineRisingSlope <= 0) return false;

    double deviation = std::abs(m_currentStats.currentRisingSlope - m_baselineRisingSlope) / m_baselineRisingSlope;

    if (deviation > m_slopeTolerance) {
        severity = std::min(deviation / m_slopeTolerance, 1.0);
        return true;
    }

    return false;
}

bool AsymmetricTriangleWaveDetector::detectFallingSlopeAnomaly(double& severity)
{
    if (m_fallingSlopeHistory.empty() || m_baselineFallingSlope <= 0) return false;

    double deviation = std::abs(m_currentStats.currentFallingSlope - m_baselineFallingSlope) / m_baselineFallingSlope;

    if (deviation > m_slopeTolerance) {
        severity = std::min(deviation / m_slopeTolerance, 1.0);
        return true;
    }

    return false;
}

bool AsymmetricTriangleWaveDetector::detectAsymmetryAnomaly(double& severity)
{
    if (m_asymmetryHistory.empty() || m_baselineAsymmetryRatio <= 0) return false;

    double deviation = std::abs(m_currentStats.asymmetryRatio - m_baselineAsymmetryRatio) / m_baselineAsymmetryRatio;

    if (deviation > m_asymmetryTolerance) {
        severity = std::min(deviation / m_asymmetryTolerance, 1.0);
        return true;
    }

    return false;
}

bool AsymmetricTriangleWaveDetector::detectAmplitudeAnomaly(double& severity)
{
    // 实现幅度异常检测
    return false;
}

bool AsymmetricTriangleWaveDetector::detectNoiseAnomaly(double& severity)
{
    // 基于当前幅度的相对噪声阈值
    double noiseThreshold = m_currentStats.amplitude * 0.05;  // 5% of amplitude

    if (m_currentStats.noiseLevel > noiseThreshold) {
        severity = std::min(m_currentStats.noiseLevel / noiseThreshold, 1.0);
        return true;
    }

    return false;
}

bool AsymmetricTriangleWaveDetector::detectPhaseJump(double& severity)
{
    // 实现相位跳变检测
    return false;
}

void AsymmetricTriangleWaveDetector::updateBaseline()
{
    if (m_samplesProcessed < 100) return;

    // 更新基准频率
    if (!m_frequencyHistory.empty()) {
        m_baselineFrequency = m_currentStats.averageFrequency;
    }

    // 更新基准峰值和谷值
    if (!m_peakHistory.empty()) {
        m_baselinePeak = m_currentStats.averagePeak;
    }
    if (!m_valleyHistory.empty()) {
        m_baselineValley = m_currentStats.averageValley;
    }

    // 更新基准斜率
    if (!m_risingSlopeHistory.empty()) {
        m_baselineRisingSlope = m_currentStats.averageRisingSlope;
    }
    if (!m_fallingSlopeHistory.empty()) {
        m_baselineFallingSlope = m_currentStats.averageFallingSlope;
    }

    // 更新基准非对称比
    if (!m_asymmetryHistory.empty()) {
        double sum = std::accumulate(m_asymmetryHistory.begin(), m_asymmetryHistory.end(), 0.0);
        m_baselineAsymmetryRatio = sum / m_asymmetryHistory.size();
    }

    // 学习完成后输出
    if (m_samplesProcessed == m_learningPeriod) {
        LOG_INFO_CL("=== 非对称三角波学习完成 ===");
        LOG_INFO_CL("基准频率: {} Hz", m_baselineFrequency);
        LOG_INFO_CL("基准峰值: {}, 基准谷值: {}", m_baselinePeak, m_baselineValley);
        LOG_INFO_CL("基准上升斜率: {}, 基准下降斜率: {}", m_baselineRisingSlope, m_baselineFallingSlope);
        LOG_INFO_CL("基准非对称比: {}", m_baselineAsymmetryRatio);
        LOG_INFO_CL("开始异常检测...");
    }
}

void AsymmetricTriangleWaveDetector::onRecordingTimeout()
{
    if (m_isRecording) {
        m_isRecording = false;
        emit recordingStopped(m_recordedDataPoints);
        LOG_INFO("三角波异常记录结束，共记录 {} 个数据点", m_recordedDataPoints);
    }
}

// 配置函数实现
void AsymmetricTriangleWaveDetector::setNominalFrequency(double frequency)
{
    m_nominalFrequency = frequency;
    m_baselineFrequency = frequency;
    LOG_INFO("设置标称频率: {} Hz", frequency);
}

void AsymmetricTriangleWaveDetector::setExpectedPeakValley(uint16_t peak, uint16_t valley)
{
    m_expectedPeak = peak;
    m_expectedValley = valley;
    m_baselinePeak = peak;
    m_baselineValley = valley;
    LOG_INFO("设置期望峰谷值: 峰值={}, 谷值={}", peak, valley);
}

void AsymmetricTriangleWaveDetector::setExpectedSlopes(double risingSlope, double fallingSlope)
{
    m_expectedRisingSlope = risingSlope;
    m_expectedFallingSlope = fallingSlope;
    m_baselineRisingSlope = risingSlope;
    m_baselineFallingSlope = fallingSlope;
    LOG_INFO("设置期望斜率: 上升={}, 下降={}", risingSlope, fallingSlope);
}

void AsymmetricTriangleWaveDetector::setAsymmetryRatio(double ratio)
{
    m_expectedAsymmetryRatio = ratio;
    m_baselineAsymmetryRatio = ratio;
    LOG_INFO("设置非对称比: {}", ratio);
}

void AsymmetricTriangleWaveDetector::setSamplingRate(double sampleRate)
{
    m_samplingRate = sampleRate;
    m_learningPeriod = static_cast<int>(sampleRate * 5);  // 学习5秒
    LOG_INFO("设置采样率: {} Hz, 学习周期: {} 样本", sampleRate, m_learningPeriod);
}

void AsymmetricTriangleWaveDetector::setRecordingDuration(int seconds)
{
    m_recordingDuration = seconds;
    LOG_INFO("设置记录持续时间: {} 秒", seconds);
}

void AsymmetricTriangleWaveDetector::setDetectionThresholds(double freqTolerance, double peakTolerance,
                                                           double slopeTolerance, double asymmetryTolerance)
{
    m_frequencyTolerance = freqTolerance;
    m_peakTolerance = peakTolerance;
    m_slopeTolerance = slopeTolerance;
    m_asymmetryTolerance = asymmetryTolerance;

    LOG_INFO("设置检测阈值: 频率容差={}%, 峰值容差={}%, 斜率容差={}%, 非对称容差={}%",
            freqTolerance * 100, peakTolerance * 100, slopeTolerance * 100, asymmetryTolerance * 100);
}

void AsymmetricTriangleWaveDetector::enableAnomalyType(TriangleAnomalyType type, bool enabled)
{
    m_enabledAnomalies[type] = enabled;
    LOG_INFO("异常类型 {}: {}", static_cast<int>(type), enabled ? "启用" : "禁用");
}

void AsymmetricTriangleWaveDetector::setAdaptiveMode(bool enabled)
{
    m_adaptiveEnabled = enabled;
    LOG_INFO("自适应模式: {}", enabled ? "启用" : "禁用");
}

void AsymmetricTriangleWaveDetector::reset()
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
    m_peakHistory.clear();
    m_valleyHistory.clear();
    m_risingSlopeHistory.clear();
    m_fallingSlopeHistory.clear();
    m_asymmetryHistory.clear();

    // 重置状态
    m_currentStats = TriangleWaveStats();
    m_lastAnomaly = TriangleAnomalyResult();
    m_previousPhase = TrianglePhase::Unknown;
    m_phaseStartIndex = 0;
    m_phaseStartTime = 0;
    m_samplesProcessed = 0;
    m_analysisCounter = 0;
    m_recordedDataPoints = 0;

    LOG_INFO("非对称三角波检测器已重置");
}
