// AnomalyRecorderThread.cpp
#include "AnomalyRecorderThread.h"


#define RECORED_POINTS 20000 //两万个点，大概10秒的数据，因为一秒大概2048个数据

AnomalyRecorderThread::AnomalyRecorderThread(QObject *parent)
    : QThread(parent)
    , m_stopRequested(false)
    , m_totalRecorded(0)
{
}

AnomalyRecorderThread::~AnomalyRecorderThread()
{
    stopRecording();
}

// 修改启动记录方法
bool AnomalyRecorderThread::startRecording(const QString& filename, const TriangleAnomalyResult& trigger)
{
    QMutexLocker locker(&m_mutex);

    if (isRunning()) {
        LOG_WARN("记录线程已在运行");
        return false;
    }

    m_filename = filename;
    m_triggerInfo = trigger;  // 保存三角波异常触发信息
    m_startTime = QDateTime::currentMSecsSinceEpoch();
    m_stopRequested = false;
    m_totalRecorded = 0;
    m_dataQueue.clear();

    start();
    return true;
}

void AnomalyRecorderThread::stopRecording()
{
    m_stopRequested = true;
    m_dataAvailable.wakeAll();

    if (!wait(5000)) {  // 等待最多5秒
        LOG_ERROR("记录线程未能正常结束");
        terminate();
        wait();
    }
}

void AnomalyRecorderThread::addData(uint16_t value, qint64 timestamp)
{
    QMutexLocker locker(&m_mutex);

    RecordData data;
    data.value = value;
    data.timestamp = timestamp;
    data.index = m_totalRecorded + m_dataQueue.size() + 1;

    m_dataQueue.enqueue(data);
    //如果当前队列以及超过10000个数据，那么记录
    if (m_dataQueue.size() > RECORED_POINTS)
    {
        m_dataAvailable.wakeOne();
    }
}

void AnomalyRecorderThread::run()
{
    LOG_INFO_CL("异常记录线程启动: {}", m_filename);

    QFile file(m_filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit recordingError(QString("无法创建文件: %1").arg(m_filename));
        return;
    }

    QTextStream stream(&file);

    // 写入文件头
    if (!writeHeader(stream)) {
        emit recordingError("写入文件头失败");
        return;
    }

    // 主循环
    while (!m_stopRequested || !m_dataQueue.isEmpty()) {
        QMutexLocker locker(&m_mutex);

        // 等待数据
        if (m_dataQueue.isEmpty()) {
            m_dataAvailable.wait(&m_mutex);  // 无限等待
        }

        // 批量处理数据
        QList<RecordData> batch;
        int batchSize = qMin(RECORED_POINTS, m_dataQueue.size());  // 每次最多处理10000个

        for (int i = 0; i < batchSize; ++i) {
            batch.append(m_dataQueue.dequeue());
        }

        locker.unlock();  // 解锁，允许其他线程添加数据

        // 写入数据（不需要锁）
        for (const auto& data : batch) {
            writeData(stream, data);
            m_totalRecorded++;
        }

        // 定期刷新
        if (m_totalRecorded % 1000 == 0) {
            stream.flush();
            LOG_DEBUG_CL("已记录 {} 个数据点", m_totalRecorded.load());
        }
    }

    // 写入文件尾
    writeFooter(stream);
    stream.flush();
    file.close();

    LOG_INFO("异常记录完成: {} 个数据点", m_totalRecorded.load());
    emit recordingFinished(m_totalRecorded, m_filename);
}

// 修改文件头写入方法
bool AnomalyRecorderThread::writeHeader(QTextStream& stream)
{
    QString anomalyTypeStr = getTriangleAnomalyTypeString(m_triggerInfo.type);
    QString phaseStr;

    // 转换相位枚举为字符串
    switch (m_triggerInfo.phaseWhenDetected) {
        case TrianglePhase::Rising:
            phaseStr = "上升阶段";
            break;
        case TrianglePhase::Falling:
            phaseStr = "下降阶段";
            break;
        case TrianglePhase::AtPeak:
            phaseStr = "波峰";
            break;
        case TrianglePhase::AtValley:
            phaseStr = "波谷";
            break;
        default:
            phaseStr = "未知";
            break;
    }

    // 写入详细的文件头信息
    stream << "# 三角波异常记录文件\n";
    stream << "# 生成时间: " << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz") << "\n";
    stream << "# 文件格式版本: 2.0\n";
    stream << "#\n";
    stream << "# === 异常触发信息 ===\n";
    stream << "# 异常类型: " << anomalyTypeStr << "\n";
    stream << "# 异常严重程度: " << QString::number(m_triggerInfo.severity * 100, 'f', 2) << "%\n";
    stream << "# 触发值: " << m_triggerInfo.triggerValue << "\n";
    stream << "# 触发时间戳: " << m_triggerInfo.timestamp << " ms\n";
    stream << "# 检测阶段: " << phaseStr << "\n";
    stream << "# 异常描述: " << m_triggerInfo.description << "\n";
    stream << "#\n";
    stream << "# === 记录配置 ===\n";
    stream << "# 记录开始时间: " << QDateTime::fromMSecsSinceEpoch(m_startTime).toString("yyyy-MM-dd HH:mm:ss.zzz") << "\n";
    stream << "# 采样率: 2560 Hz (估算)\n";
    stream << "# 数据格式: 12位ADC值 (0-4095)\n";
    stream << "#\n";
    stream << "# === 数据列说明 ===\n";
    stream << "# Index: 数据点序号 (从1开始)\n";
    stream << "# Timestamp(ms): 绝对时间戳 (毫秒)\n";
    stream << "# Value: ADC采样值 (0-4095)\n";
    stream << "# RelativeTime(ms): 相对于记录开始的时间 (毫秒)\n";
    stream << "#\n";

    // CSV头
    stream << "Index,Timestamp(ms),Value,RelativeTime(ms)\n";

    return stream.status() == QTextStream::Ok;
}

void AnomalyRecorderThread::writeData(QTextStream& stream, const RecordData& data)
{
    qint64 relativeTime = data.timestamp - m_startTime;
    stream << data.index << ","
           << data.timestamp << ","
           << data.value << ","
           << relativeTime << "\n";
}

// 修改文件尾写入方法
void AnomalyRecorderThread::writeFooter(QTextStream& stream)
{
    qint64 endTime = QDateTime::currentMSecsSinceEpoch();
    qint64 recordingDuration = endTime - m_startTime;

    stream << "#\n";
    stream << "# === 记录完成信息 ===\n";
    stream << "# 记录结束时间: " << QDateTime::fromMSecsSinceEpoch(endTime).toString("yyyy-MM-dd HH:mm:ss.zzz") << "\n";
    stream << "# 总数据点: " << m_totalRecorded << "\n";
    stream << "# 记录时长: " << recordingDuration << " ms (" << (recordingDuration / 1000.0) << " 秒)\n";
    stream << "# 平均采样率: " << QString::number((m_totalRecorded * 1000.0) / recordingDuration, 'f', 2) << " Hz\n";
    stream << "#\n";
    stream << "# 文件结束\n";
}

// 新增：三角波异常类型转换函数
QString AnomalyRecorderThread::getTriangleAnomalyTypeString(TriangleAnomalyType type)
{
    switch (type) {      
        case TriangleAnomalyType::RisingSlopeAnomaly:
            return "上升沿斜率异常";
        case TriangleAnomalyType::FallingSlopeAnomaly:
            return "下降沿斜率异常";
        case TriangleAnomalyType::PeakValueAnomaly:
            return "波峰值异常";
        case TriangleAnomalyType::ValleyValueAnomaly:
            return "波谷值异常";
        case TriangleAnomalyType::PeriodAnomaly:
            return "周期异常";
        default:
            return "未知异常";
    }
}
