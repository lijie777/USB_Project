#ifndef TRIANGLEDETECTORCONFIGDIALOG_H
#define TRIANGLEDETECTORCONFIGDIALOG_H

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>
#include <QDialogButtonBox>

class TriangleDetectorConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TriangleDetectorConfigDialog(QWidget *parent = nullptr);

    // 设置当前配置
    void setCurrentConfig(double peakTolerance, double valleyTolerance,
                         double slopeTolerance, double jumpThreshold,
                         int learningDataCount,
                         int risingEdgeThreshold, int fallingEdgeThreshold); // 【修改】简化参数

    // 获取配置
    double getPeakTolerance() const { return m_peakToleranceSpin->value(); }
    double getValleyTolerance() const { return m_valleyToleranceSpin->value(); }
    double getSlopeTolerance() const { return m_slopeToleranceSpin->value(); }
    double getJumpThreshold() const { return m_jumpThresholdSpin->value(); }
    int getLearningDataCount() const { return m_learningDataCountSpin->value(); }

    // 【修改】简化为两个长度阈值
    int getRisingEdgeThreshold() const { return m_risingEdgeThresholdSpin->value(); }
    int getFallingEdgeThreshold() const { return m_fallingEdgeThresholdSpin->value(); }

private slots:
    void onResetToDefaults();

private:
    void setupUI();

    QDoubleSpinBox* m_peakToleranceSpin;      // 波峰容差百分比
    QDoubleSpinBox* m_valleyToleranceSpin;    // 波谷容差百分比
    QDoubleSpinBox* m_slopeToleranceSpin;     // 斜率容差百分比
    QDoubleSpinBox* m_jumpThresholdSpin;      // 跳变阈值
    QSpinBox* m_learningDataCountSpin;        // 学习数据点数

    // 【修改】只要两个长度阈值控件
    QSpinBox* m_risingEdgeThresholdSpin;      // 上升沿长度阈值
    QSpinBox* m_fallingEdgeThresholdSpin;     // 下降沿长度阈值

    QPushButton* m_resetButton;
    QDialogButtonBox* m_buttonBox;
};

#endif // TRIANGLEDETECTORCONFIGDIALOG_H
