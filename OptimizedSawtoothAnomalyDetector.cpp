// 实现文件
#include "OptimizedSawtoothAnomalyDetector.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include "MCLogger.h"


OptimizedSawtoothAnomalyDetector::OptimizedSawtoothAnomalyDetector(QObject *parent)
    : QObject(parent)
    , m_nominalFrequency(NOMINALFREQUENCY)         // 用户设定的期望频率,设为 0，让程序自动学习
    , m_expectedMin(EXPECTEDMIN)                   // 保持全范围，让程序自动缩小
    , m_expectedMax(EXPECTEDMAX)                   // 保持全范围，让程序自动缩小
    , m_samplingRate(SAMPLINGRATE)                 // 数据采样频率，必须根据实际USB数据率设置，实际采样率 = USB数据率(bps) / 16
    , m_frequencyTolerance(FREQUENCYTOLERANCE)     // 允许的频率偏差百分比，保持 5%，可根据信号稳定性动态调整
    , m_amplitudeTolerance(AMPLITUDETOLERANCE)     // 允许的幅度变化百分比，初始 10%，可根据噪声水平调整
    , m_linearityThreshold(LINEARTTHREASHOLD)      // 锯齿波直线部分的线性度要求（0-1），默认值：0.9 (90%)，建议：低噪声环境：0.95；一般环境：0.90；高噪声环境：0.85
    , m_previousPhase(SawtoothPhase::Unknown)      // 记录前一个检测到的相位,保持默认Unknown
    , m_phaseStartIndex(0)                         // 当前相位开始的数据索引,保持默认0
    , m_lastPeakValue(0)                           // 记录最近的峰值和谷值,默认值：0
    , m_lastValleyValue(65535)                     // 记录最近的峰值和谷值，默认值：65535
    , m_lastTransitionTime(0)                      // 作用：记录相位转换时间戳,默认值：0
    , m_adaptiveEnabled(true)                      // 控制是否自动学习和调整参数,保持 true，以适应参数变化
    , m_baselineFrequency(NOMINALFREQUENCY)        // 学习期间建立的实际基准频率，初始设为 0，通过学习获得
    , m_baselineAmplitude(BASHLINEAMPLITUDE)       // 估算值，需要调整
    , m_learningPeriod(LEARNINGPERIOD)             // 学习周期, 学习1000个样本,设为 `采样率 × 5`（学习5秒）
    , m_samplesProcessed(0)                        // 已处理样本数,保持默认，计数器，跟踪处理进度,自动管理
    , m_isRecording(false)                         // 作用：标记是否正在记录异常数据,默认值：false
    , m_recordingDuration(10)                      // 作用：检测到异常后记录的时长,默认值：10 秒，建议5-30秒，根据需要调整
    , m_recordedDataPoints(0)                      // 统计记录的数据点数，默认值：0
    , m_analysisCounter(0)                         // 作用：控制分析频率的计数器
{
    // 初始化定时器
    m_recordingTimer = new QTimer(this);
    m_recordingTimer->setSingleShot(true);
    connect(m_recordingTimer, &QTimer::timeout, this, &OptimizedSawtoothAnomalyDetector::onRecordingTimeout);

    // 默认启用所有异常类型
    m_enabledAnomalies[SawtoothAnomalyType::FrequencyDrift] = true;
    m_enabledAnomalies[SawtoothAnomalyType::AmplitudeDrift] = true;
    m_enabledAnomalies[SawtoothAnomalyType::LinearityError] = true;
    m_enabledAnomalies[SawtoothAnomalyType::PhaseJump] = true;
    m_enabledAnomalies[SawtoothAnomalyType::MissingEdge] = true;
    m_enabledAnomalies[SawtoothAnomalyType::NoiseSpike] = true;
    m_enabledAnomalies[SawtoothAnomalyType::Saturation] = true;
    m_enabledAnomalies[SawtoothAnomalyType::Dropout] = true;

    qDebug() << "OptimizedSawtoothAnomalyDetector 初始化完成";
    qDebug() << QString("标称频率: %1 Hz").arg(m_nominalFrequency);
}

OptimizedSawtoothAnomalyDetector::~OptimizedSawtoothAnomalyDetector()
{
    if (m_isRecording) {
        m_recordingTimer->stop();
        emit recordingStopped(m_recordedDataPoints);
    }
}

void OptimizedSawtoothAnomalyDetector::feedData(uint16_t value)
{
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();

    // 添加数据到缓冲区
    m_dataBuffer.push_back(value);
    m_timestampBuffer.push_back(currentTime);

    // 维护缓冲区大小 (保留足够的数据用于频率分析)
    int maxBufferSize = static_cast<int>(m_samplingRate * 5);  // 5秒的数据
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

        if (m_dataBuffer.size() >= 100) {  // 至少100个点才开始分析
            // 检测相位转换
            detectPhaseTransition();

            // 更新统计信息
            updateSawtoothStats();

            // 自适应学习阶段
            if (m_adaptiveEnabled && m_samplesProcessed < m_learningPeriod) {
                updateBaseline();
            } else {
                // 执行异常检测
                SawtoothAnomalyResult anomaly = analyzeSawtooth();

                if (anomaly.type != SawtoothAnomalyType::None) {
                    m_lastAnomaly = anomaly;
                    emit anomalyDetected(anomaly);

                    // 开始记录（如果未在记录中）
                    if (!m_isRecording) {
                        m_isRecording = true;
                        m_recordedDataPoints = 0;
                        m_recordingTimer->start(m_recordingDuration * 1000);
                        emit recordingStarted(anomaly);

                        qDebug() << QString("检测到锯齿波异常: %1, 严重程度: %2")
                                   .arg(static_cast<int>(anomaly.type))
                                   .arg(anomaly.severity);
                    }
                }
            }

            // 定期发送统计更新
            if (m_samplesProcessed % 200 == 0) {
                emit statsUpdated(m_currentStats);
            }
        }
    }
}

void OptimizedSawtoothAnomalyDetector::setOptimalParameters()
{
    // 1. 计算实际采样率
    double usbDataRate = 10.0e6;  // 10 Mbps (根据实际情况修改)
    double samplingRate = usbDataRate / 16;  // 16位per sample
      setSamplingRate(samplingRate);

      // 2. 设置自适应学习
      setAdaptiveThresholds(true);

      // 3. 频率参数（让程序自动学习）
      setNominalFrequency(0);  // 0 表示自动学习
      m_baselineFrequency = 0;

      // 4. 幅度参数（全范围，自动缩小）
      setExpectedAmplitude(0, 65535);
      m_baselineAmplitude = 0;

      // 5. 检测阈值（适中设置）
      setDetectionThresholds(
          0.05,   // 频率容差 5%
          0.10,   // 幅度容差 10%
          0.90    // 线性度 90%
      );

      // 6. 学习周期（至少20个锯齿波周期）
      // 假设锯齿波频率在1-10Hz范围
      m_learningPeriod = static_cast<int>(samplingRate * 5);  // 学习5秒

      // 7. 记录参数
      setRecordingDuration(10);  // 异常后记录10秒

      // 8. 性能优化
      // 根据采样率调整分析间隔
      if (samplingRate > 100000) {
          // 高采样率，增加间隔
          // 注意：ANALYSIS_INTERVAL 是常量，需要改为变量才能动态调整
      }

      // 9. 启用所有异常检测（可根据需求调整）
      for (int i = 1; i <= 8; i++) {
          enableAnomalyType(static_cast<SawtoothAnomalyType>(i), true);
      }

      LOG_INFO("锯齿波检测器优化配置完成");
      LOG_INFO("采样率: {} Hz", samplingRate);
      LOG_INFO("学习周期: {} 样本 (约{}秒)", m_learningPeriod, m_learningPeriod/samplingRate);
}

void OptimizedSawtoothAnomalyDetector::detectPhaseTransition()
{
    if (m_dataBuffer.size() < 10) return;

    // 简单的相位检测：基于斜率变化
    size_t currentIdx = m_dataBuffer.size() - 1;
    size_t prevIdx = currentIdx - 5;

    if (prevIdx < 5) return;

    // 计算最近的斜率
    double recentSlope = calculateSlope(prevIdx, currentIdx);
    double prevSlope = calculateSlope(prevIdx - 5, prevIdx);

    SawtoothPhase newPhase = SawtoothPhase::Unknown;

    // 判断当前阶段
    if (recentSlope > 100) {        // 正斜率，上升阶段
        newPhase = SawtoothPhase::Rising;
    } else if (recentSlope < -100) { // 负斜率，下降阶段
        newPhase = SawtoothPhase::Falling;
    }

    // 检测相位变化
    if (newPhase != m_previousPhase && newPhase != SawtoothPhase::Unknown) {
        qint64 currentTime = m_timestampBuffer.back();

        if (m_previousPhase != SawtoothPhase::Unknown) {
            // 记录相位转换时间，用于频率计算
            if (m_lastTransitionTime > 0) {
                qint64 cycleTime = currentTime - m_lastTransitionTime;
                if (cycleTime > 0) {
                    // 半周期时间，完整周期是两倍
                    double halfCycleFreq = 500.0 / cycleTime;  // Hz
                    m_frequencyHistory.push_back(halfCycleFreq);

                    if (m_frequencyHistory.size() > 20) {
                        m_frequencyHistory.pop_front();
                    }
                }
            }
        }

        m_lastTransitionTime = currentTime;
        m_previousPhase = newPhase;
        m_phaseStartIndex = currentIdx;

        // 记录峰值和谷值
        if (newPhase == SawtoothPhase::Falling) {
            m_lastPeakValue = m_dataBuffer.back();
        } else if (newPhase == SawtoothPhase::Rising) {
            m_lastValleyValue = m_dataBuffer.back();
            m_currentStats.cycleCount++;
        }
    }

    m_currentStats.currentPhase = newPhase;
}

SawtoothAnomalyResult OptimizedSawtoothAnomalyDetector::analyzeSawtooth()
{
    SawtoothAnomalyResult result;
    double maxSeverity = 0.0;

    double severity;

    // 频率漂移检测
    if (m_enabledAnomalies[SawtoothAnomalyType::FrequencyDrift] && detectFrequencyDrift(severity)) {
        if (severity > maxSeverity) {
            result.type = SawtoothAnomalyType::FrequencyDrift;
            result.severity = severity;
            result.description = QString("频率漂移: 当前频率 %1 Hz, 标称频率 %2 Hz")
                               .arg(m_currentStats.currentFrequency, 0, 'f', 3)
                               .arg(m_nominalFrequency, 0, 'f', 3);
            maxSeverity = severity;
        }
    }

    // 幅度漂移检测
    if (m_enabledAnomalies[SawtoothAnomalyType::AmplitudeDrift] && detectAmplitudeDrift(severity)) {
        if (severity > maxSeverity) {
            result.type = SawtoothAnomalyType::AmplitudeDrift;
            result.severity = severity;
            result.description = QString("幅度异常: 当前幅度 %1, 基准幅度 %2")
                               .arg(m_currentStats.averageAmplitude)
                               .arg(m_baselineAmplitude);
            maxSeverity = severity;
        }
    }

    // 线性度误差检测
    if (m_enabledAnomalies[SawtoothAnomalyType::LinearityError] && detectLinearityError(severity)) {
        if (severity > maxSeverity) {
            result.type = SawtoothAnomalyType::LinearityError;
            result.severity = severity;
            result.description = QString("线性度异常: 当前线性度 %1, 阈值 %2")
                               .arg(m_currentStats.linearity, 0, 'f', 3)
                               .arg(m_linearityThreshold, 0, 'f', 3);
            maxSeverity = severity;
        }
    }

    // 相位跳跃检测
    if (m_enabledAnomalies[SawtoothAnomalyType::PhaseJump] && detectPhaseJump(severity)) {
        if (severity > maxSeverity) {
            result.type = SawtoothAnomalyType::PhaseJump;
            result.severity = severity;
            result.description = "相位突跳: 检测到异常的相位变化";
            maxSeverity = severity;
        }
    }

    // 噪声尖峰检测
    if (m_enabledAnomalies[SawtoothAnomalyType::NoiseSpike] && detectNoiseSpike(severity)) {
        if (severity > maxSeverity) {
            result.type = SawtoothAnomalyType::NoiseSpike;
            result.severity = severity;
            result.description = QString("噪声尖峰: 噪声水平 %1")
                               .arg(m_currentStats.noiseLevel, 0, 'f', 1);
            maxSeverity = severity;
        }
    }

    // 填充结果信息
    if (result.type != SawtoothAnomalyType::None) {
        result.triggerValue = m_dataBuffer.back();
        result.timestamp = QDateTime::currentMSecsSinceEpoch();
        result.phaseWhenDetected = m_currentStats.currentPhase;
    }

    return result;
}

bool OptimizedSawtoothAnomalyDetector::detectFrequencyDrift(double& severity)
{
    if (m_frequencyHistory.empty()) return false;

    double currentFreq = m_currentStats.currentFrequency;
    double expectedFreq = m_baselineFrequency;

    if (expectedFreq <= 0) return false;

    double deviation = std::abs(currentFreq - expectedFreq) / expectedFreq;

    if (deviation > m_frequencyTolerance) {
        severity = std::min(deviation / m_frequencyTolerance, 1.0);
        return true;
    }

    return false;
}

bool OptimizedSawtoothAnomalyDetector::detectLinearityError(double& severity)
{
    double currentLinearity = calculateLinearity();
    m_currentStats.linearity = currentLinearity;

    if (currentLinearity < m_linearityThreshold) {
        severity = (m_linearityThreshold - currentLinearity) / m_linearityThreshold;
        return severity > 0.1;  // 至少10%的线性度降低才报告
    }

    return false;
}

double OptimizedSawtoothAnomalyDetector::calculateLinearity()
{
    if (m_dataBuffer.size() < 50) return 1.0;

    // 分析最近的单调段的线性度
    int segmentLength = std::min(100, static_cast<int>(m_dataBuffer.size()) / 2);
    int startIdx = m_dataBuffer.size() - segmentLength;

    // 创建理想直线
    std::vector<double> x(segmentLength), y(segmentLength);
    for (int i = 0; i < segmentLength; i++) {
        x[i] = i;
        y[i] = m_dataBuffer[startIdx + i];
    }

    // 计算与理想直线的相关系数
    return calculateCorrelation(x, y);
}

double OptimizedSawtoothAnomalyDetector::calculateSlope(int startIdx, int endIdx)
{
    if (startIdx >= endIdx || endIdx >= static_cast<int>(m_dataBuffer.size())) return 0.0;

    double deltaY = m_dataBuffer[endIdx] - m_dataBuffer[startIdx];
    double deltaX = endIdx - startIdx;

    return deltaX > 0 ? deltaY / deltaX : 0.0;
}

double OptimizedSawtoothAnomalyDetector::calculateCorrelation(const std::vector<double>& x, const std::vector<double>& y)
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

void OptimizedSawtoothAnomalyDetector::updateSawtoothStats()
{
    // 更新频率统计
    m_currentStats.currentFrequency = calculateCurrentFrequency();

    if (!m_frequencyHistory.empty()) {
        m_currentStats.averageFrequency = std::accumulate(m_frequencyHistory.begin(),
                                                         m_frequencyHistory.end(), 0.0) / m_frequencyHistory.size();

        // 计算频率稳定性（标准差的倒数）
        double variance = 0.0;
        for (double freq : m_frequencyHistory) {
            variance += std::pow(freq - m_currentStats.averageFrequency, 2);
        }
        variance /= m_frequencyHistory.size();
        m_currentStats.frequencyStability = variance > 0 ? 1.0 / std::sqrt(variance) : 1.0;
    }

    // 更新幅度统计
    if (m_lastPeakValue > m_lastValleyValue) {
        m_currentStats.currentMin = m_lastValleyValue;
        m_currentStats.currentMax = m_lastPeakValue;
        uint16_t currentAmplitude = m_lastPeakValue - m_lastValleyValue;

        m_amplitudeHistory.push_back(currentAmplitude);
        if (m_amplitudeHistory.size() > 10) {
            m_amplitudeHistory.pop_front();
        }

        if (!m_amplitudeHistory.empty()) {
            m_currentStats.averageAmplitude = std::accumulate(m_amplitudeHistory.begin(),
                                                            m_amplitudeHistory.end(), 0) / m_amplitudeHistory.size();
        }
    }
}

double OptimizedSawtoothAnomalyDetector::calculateCurrentFrequency()
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

// 其他检测函数的基本实现
bool OptimizedSawtoothAnomalyDetector::detectAmplitudeDrift(double& severity)
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

bool OptimizedSawtoothAnomalyDetector::detectPhaseJump(double& severity)
{
    // 简单实现：检测相位转换时间的异常
    if (m_frequencyHistory.size() < 3) return false;

    double lastFreq = m_frequencyHistory.back();
    double avgFreq = m_currentStats.averageFrequency;

    if (avgFreq > 0) {
        double jump = std::abs(lastFreq - avgFreq) / avgFreq;
        if (jump > 0.2) {  // 20%的跳变
            severity = std::min(jump / 0.2, 1.0);
            return true;
        }
    }

    return false;
}

bool OptimizedSawtoothAnomalyDetector::detectNoiseSpike(double& severity)
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

bool OptimizedSawtoothAnomalyDetector::detectMissingEdge(double& severity)
{
    // 检测是否缺失上升或下降沿
    if (m_currentStats.currentPhase == SawtoothPhase::Unknown) {
        severity = 0.8;
        return true;
    }

    // 检查相位持续时间是否异常
    if (m_phaseStartIndex > 0) {
        int phaseDuration = m_dataBuffer.size() - m_phaseStartIndex;
        int expectedDuration = static_cast<int>(m_samplingRate / (2 * m_nominalFrequency));

        if (phaseDuration > expectedDuration * 2) {  // 持续时间超过期望的2倍
            severity = std::min(static_cast<double>(phaseDuration) / expectedDuration - 1.0, 1.0);
            return true;
        }
    }

    return false;
}

bool OptimizedSawtoothAnomalyDetector::detectSaturation(double& severity)
{
    uint16_t currentValue = m_dataBuffer.back();

    // 检测饱和（接近极值）
    if (currentValue <= 100 || currentValue >= 65435) {
        severity = 0.9;
        return true;
    }

    // 检测削顶（连续相同的极值）
    if (m_dataBuffer.size() >= 5) {
        bool allSame = true;
        for (size_t i = m_dataBuffer.size() - 5; i < m_dataBuffer.size(); i++) {
            if (m_dataBuffer[i] != currentValue) {
                allSame = false;
                break;
            }
        }

        if (allSame && (currentValue == m_currentStats.currentMax || currentValue == m_currentStats.currentMin)) {
            severity = 0.7;
            return true;
        }
    }

    return false;
}

bool OptimizedSawtoothAnomalyDetector::detectDropout(double& severity)
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

    if (consecutiveCount > 10) {  // 连续10个相同值
        severity = std::min(consecutiveCount / 20.0, 1.0);
        return true;
    }

    return false;
}

void OptimizedSawtoothAnomalyDetector::updateBaseline()
{
    // 需要足够样本才开始学习
    if (m_samplesProcessed < 100) return;

    // === 自动学习频率 ===
    if (!m_frequencyHistory.empty()) {
        // 计算频率的中位数（比平均值更稳定）
        std::vector<double> sortedFreq(m_frequencyHistory.begin(), m_frequencyHistory.end());
        std::sort(sortedFreq.begin(), sortedFreq.end());
        double medianFreq = sortedFreq[sortedFreq.size() / 2];

        // 如果没有设置标称频率（为0），则使用学习到的值
        if (m_nominalFrequency == 0 || m_adaptiveEnabled) {
            m_baselineFrequency = medianFreq;
            if (m_nominalFrequency == 0) {
                m_nominalFrequency = medianFreq;
            }
        }
    }

    // === 自动学习幅度范围 ===
    if (!m_amplitudeHistory.empty()) {
        // 使用统计方法确定稳定的幅度
        std::vector<uint16_t> sortedAmp(m_amplitudeHistory.begin(), m_amplitudeHistory.end());
        std::sort(sortedAmp.begin(), sortedAmp.end());

        // 使用四分位数去除异常值
        size_t q1_idx = sortedAmp.size() / 4;
        size_t q3_idx = (sortedAmp.size() * 3) / 4;

        uint16_t q1 = sortedAmp[q1_idx];
        uint16_t q3 = sortedAmp[q3_idx];
        uint16_t iqr = q3 - q1;  // 四分位距

        // 过滤异常值后计算平均值
        uint64_t sum = 0;
        int count = 0;
        for (uint16_t amp : sortedAmp) {
            if (amp >= q1 - iqr * 1.5 && amp <= q3 + iqr * 1.5) {
                sum += amp;
                count++;
            }
        }

        if (count > 0) {
            m_baselineAmplitude = sum / count;
        }
    }

    // === 动态调整阈值（可选）===
    if (m_adaptiveEnabled && m_samplesProcessed > m_learningPeriod / 2) {
        // 根据信号稳定性调整容差
        if (m_currentStats.frequencyStability > 10) {  // 非常稳定
            m_frequencyTolerance = 0.02;  // 2%
            m_amplitudeTolerance = 0.05;   // 5%
        } else if (m_currentStats.frequencyStability > 5) {  // 较稳定
            m_frequencyTolerance = 0.05;   // 5%
            m_amplitudeTolerance = 0.10;   // 10%
        } else {  // 不太稳定
            m_frequencyTolerance = 0.10;   // 10%
            m_amplitudeTolerance = 0.15;   // 15%
        }
    }

    // === 学习完成后输出 ===
    if (m_samplesProcessed == m_learningPeriod) {
        LOG_INFO_CL("=== 自适应学习完成 ===");
        LOG_INFO_CL("学习到的基准频率: {} Hz", m_baselineFrequency);
        LOG_INFO_CL("学习到的基准幅度: {}", m_baselineAmplitude);
        LOG_INFO_CL("学习到的值范围: {} - {}", m_currentStats.currentMin, m_currentStats.currentMax);
        LOG_INFO_CL("频率稳定性指数: {}", m_currentStats.frequencyStability);
        LOG_INFO_CL("调整后的频率容差: {}%", m_frequencyTolerance * 100);
        LOG_INFO_CL("调整后的幅度容差: {}%", m_amplitudeTolerance * 100);
        LOG_INFO_CL("开始异常检测...");
        LOG_INFO_CL("锯齿波参数学习完成，开始监控异常");
    }
}

void OptimizedSawtoothAnomalyDetector::onRecordingTimeout()
{
    if (m_isRecording) {
        m_isRecording = false;
        emit recordingStopped(m_recordedDataPoints);
        qDebug() << QString("锯齿波异常记录结束，共记录 %1 个数据点").arg(m_recordedDataPoints);
    }
}

// 配置函数实现
void OptimizedSawtoothAnomalyDetector::setNominalFrequency(double frequency)
{
    m_nominalFrequency = frequency;
    m_baselineFrequency = frequency;
    qDebug() << QString("设置标称频率: %1 Hz").arg(frequency, 0, 'f', 3);
}

void OptimizedSawtoothAnomalyDetector::setExpectedAmplitude(uint16_t minVal, uint16_t maxVal)
{
    m_expectedMin = minVal;
    m_expectedMax = maxVal;
    m_baselineAmplitude = maxVal - minVal;
    qDebug() << QString("设置期望幅度范围: %1 - %2 (幅度: %3)")
               .arg(minVal).arg(maxVal).arg(m_baselineAmplitude);
}

void OptimizedSawtoothAnomalyDetector::setDetectionThresholds(double freqTolerance, double ampTolerance, double linearityThreshold)
{
    m_frequencyTolerance = freqTolerance;
    m_amplitudeTolerance = ampTolerance;
    m_linearityThreshold = linearityThreshold;

    qDebug() << QString("设置检测阈值: 频率容差=%1%%, 幅度容差=%2%%, 线性度阈值=%3")
               .arg(freqTolerance * 100, 0, 'f', 1)
               .arg(ampTolerance * 100, 0, 'f', 1)
               .arg(linearityThreshold, 0, 'f', 2);
}

void OptimizedSawtoothAnomalyDetector::setSamplingRate(double sampleRate)
{
    m_samplingRate = sampleRate;
    qDebug() << QString("设置采样率: %1 Hz").arg(sampleRate, 0, 'f', 0);
}

void OptimizedSawtoothAnomalyDetector::setRecordingDuration(int seconds)
{
    m_recordingDuration = seconds;
    qDebug() << QString("设置记录持续时间: %1 秒").arg(seconds);
}

void OptimizedSawtoothAnomalyDetector::enableAnomalyType(SawtoothAnomalyType type, bool enabled)
{
    m_enabledAnomalies[type] = enabled;
    qDebug() << QString("异常类型 %1: %2").arg(static_cast<int>(type)).arg(enabled ? "启用" : "禁用");
}

void OptimizedSawtoothAnomalyDetector::setAdaptiveThresholds(bool enabled)
{
    m_adaptiveEnabled = enabled;
    qDebug() << QString("自适应阈值: %1").arg(enabled ? "启用" : "禁用");
}

void OptimizedSawtoothAnomalyDetector::reset()
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

    // 重置状态
    m_currentStats = SawtoothStats();
    m_lastAnomaly = SawtoothAnomalyResult();
    m_previousPhase = SawtoothPhase::Unknown;
    m_phaseStartIndex = 0;
    m_samplesProcessed = 0;
    m_analysisCounter = 0;
    m_recordedDataPoints = 0;

    qDebug() << "锯齿波异常检测器已重置";
}
