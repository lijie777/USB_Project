// AnomalyRecorderThread.h
#ifndef ANOMALYRECORDERTHREAD_H
#define ANOMALYRECORDERTHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QQueue>
#include <QFile>
#include <QTextStream>
#include "OptimizedTriangleAnomalyDetector.h"  // 改为三角波检测器
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

    bool startRecording(const QString& filename, const TriangleAnomalyResult& trigger);
    void stopRecording();
    void addData(uint16_t value, qint64 timestamp);

signals:
    void recordingFinished(int totalPoints, const QString& filename);
    void recordingError(const QString& error);
    void acquireAnomalyDataStoped();//停止异常信号采集
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
    TriangleAnomalyResult m_triggerInfo;  // 改为三角波异常结果
    qint64 m_startTime;
    std::atomic<int> m_totalRecorded;

    // 写入文件
    bool writeHeader(QTextStream& stream);
    void writeData(QTextStream& stream, const RecordData& data);
    void writeFooter(QTextStream& stream);
    QString getTriangleAnomalyTypeString(TriangleAnomalyType type);  // 新增辅助函数

};

#endif
