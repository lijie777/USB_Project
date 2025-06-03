// AnomalyRecorderThread.cpp
#include "AnomalyRecorderThread.h"

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

bool AnomalyRecorderThread::startRecording(const QString& filename, const SawtoothAnomalyResult& trigger)
{
    QMutexLocker locker(&m_mutex);

    if (isRunning()) {
        LOG_WARN("记录线程已在运行");
        return false;
    }

    m_filename = filename;
    m_triggerInfo = trigger;
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
    m_dataAvailable.wakeOne();
}

void AnomalyRecorderThread::run()
{
    LOG_INFO("异常记录线程启动: {}", m_filename);

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
            m_dataAvailable.wait(&m_mutex, 100);  // 最多等待100ms
            continue;
        }

        // 批量处理数据
        QList<RecordData> batch;
        int batchSize = qMin(1000, m_dataQueue.size());  // 每次最多处理1000个

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
            LOG_DEBUG("已记录 {} 个数据点", m_totalRecorded.load());
        }
    }

    // 写入文件尾
    writeFooter(stream);
    stream.flush();
    file.close();

    LOG_INFO("异常记录完成: {} 个数据点", m_totalRecorded.load());
    emit recordingFinished(m_totalRecorded, m_filename);
}

bool AnomalyRecorderThread::writeHeader(QTextStream& stream)
{
    QString anomalyTypeStr;
    // ... (异常类型转换代码，与之前相同)

    stream << "# 锯齿波异常记录文件\n";
    stream << "# 生成时间: " << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz") << "\n";
    // ... (其他头信息)
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

void AnomalyRecorderThread::writeFooter(QTextStream& stream)
{
    stream << "#\n";
    stream << "# 记录结束\n";
    stream << "# 总数据点: " << m_totalRecorded << "\n";
    stream << "# 记录时长: " << (QDateTime::currentMSecsSinceEpoch() - m_startTime) << " ms\n";
}
