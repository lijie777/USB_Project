// AnomalyRecorderThread.h
#ifndef ANOMALYRECORDERTHREAD_H
#define ANOMALYRECORDERTHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QQueue>
#include <QFile>
#include <QTextStream>
#include "OptimizedSawtoothAnomalyDetector.h"
#include "MCLogger.h"

struct RecordData {
    uint16_t value;
    qint64 timestamp;
    int index;
};

class AnomalyRecorderThread : public QThread
{
    Q_OBJECT

public:
    explicit AnomalyRecorderThread(QObject *parent = nullptr);
    ~AnomalyRecorderThread();

    bool startRecording(const QString& filename, const SawtoothAnomalyResult& trigger);
    void stopRecording();
    void addData(uint16_t value, qint64 timestamp);

signals:
    void recordingFinished(int totalPoints, const QString& filename);
    void recordingError(const QString& error);

protected:
    void run() override;

private:
    // 线程控制
    QMutex m_mutex;
    QWaitCondition m_dataAvailable;
    std::atomic<bool> m_stopRequested;

    // 数据队列
    QQueue<RecordData> m_dataQueue;

    // 文件相关
    QString m_filename;
    SawtoothAnomalyResult m_triggerInfo;
    qint64 m_startTime;
    std::atomic<int> m_totalRecorded;

    // 写入文件
    bool writeHeader(QTextStream& stream);
    void writeData(QTextStream& stream, const RecordData& data);
    void writeFooter(QTextStream& stream);
};

#endif
