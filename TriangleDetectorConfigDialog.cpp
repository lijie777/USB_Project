#include "TriangleDetectorConfigDialog.h"

TriangleDetectorConfigDialog::TriangleDetectorConfigDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("三角波检测器配置");
    setModal(true);
    resize(450, 380); // 【修改】调整高度

    setupUI();
}

void TriangleDetectorConfigDialog::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 容差配置组
    QGroupBox* toleranceGroup = new QGroupBox("异常检测容差配置", this);
    QFormLayout* toleranceLayout = new QFormLayout(toleranceGroup);

    // 波峰容差
    m_peakToleranceSpin = new QDoubleSpinBox(this);
    m_peakToleranceSpin->setRange(0.1, 100.0);
    m_peakToleranceSpin->setValue(10.0);
    m_peakToleranceSpin->setDecimals(1);
    m_peakToleranceSpin->setToolTip("波峰值异常检测的容差百分比");
    toleranceLayout->addRow("波峰容差:", m_peakToleranceSpin);

    // 波谷容差
    m_valleyToleranceSpin = new QDoubleSpinBox(this);
    m_valleyToleranceSpin->setRange(0.1, 100.0);
    m_valleyToleranceSpin->setValue(10.0);
    m_valleyToleranceSpin->setDecimals(1);
    m_valleyToleranceSpin->setToolTip("波谷值异常检测的容差百分比");
    toleranceLayout->addRow("波谷容差:", m_valleyToleranceSpin);

    // 斜率容差
    m_slopeToleranceSpin = new QDoubleSpinBox(this);
    m_slopeToleranceSpin->setRange(0.1, 100.0);
    m_slopeToleranceSpin->setValue(10.0);
    m_slopeToleranceSpin->setDecimals(1);
    m_slopeToleranceSpin->setToolTip("斜率异常检测的容差百分比");
    toleranceLayout->addRow("斜率容差:", m_slopeToleranceSpin);

    // 跳变阈值
    m_jumpThresholdSpin = new QDoubleSpinBox(this);
    m_jumpThresholdSpin->setRange(1.0, 1000.0);
    m_jumpThresholdSpin->setValue(20.0);
    m_jumpThresholdSpin->setDecimals(0);
    m_jumpThresholdSpin->setToolTip("数据跳变检测的阈值（具体数值差）");
    toleranceLayout->addRow("跳变阈值:", m_jumpThresholdSpin);

    mainLayout->addWidget(toleranceGroup);

    // 【修改】简化的长度阈值配置组
    QGroupBox* lengthGroup = new QGroupBox("半周期长度阈值配置", this);
    QFormLayout* lengthLayout = new QFormLayout(lengthGroup);

    // 上升沿长度阈值
    m_risingEdgeThresholdSpin = new QSpinBox(this);
    m_risingEdgeThresholdSpin->setRange(5, 10000);
    m_risingEdgeThresholdSpin->setValue(100);
    m_risingEdgeThresholdSpin->setSuffix(" 个点");
    lengthLayout->addRow("上升沿长度阈值:", m_risingEdgeThresholdSpin);

    // 下降沿长度阈值
    m_fallingEdgeThresholdSpin = new QSpinBox(this);
    m_fallingEdgeThresholdSpin->setRange(5, 10000);
    m_fallingEdgeThresholdSpin->setValue(100);
    m_fallingEdgeThresholdSpin->setSuffix(" 个点");
    lengthLayout->addRow("下降沿长度阈值:", m_fallingEdgeThresholdSpin);

    mainLayout->addWidget(lengthGroup);

    // 学习配置组
    QGroupBox* learningGroup = new QGroupBox("学习阶段配置", this);
    QFormLayout* learningLayout = new QFormLayout(learningGroup);

    // 学习数据点数
    m_learningDataCountSpin = new QSpinBox(this);
    m_learningDataCountSpin->setRange(500, 1000000);
    m_learningDataCountSpin->setValue(50000);
    m_learningDataCountSpin->setToolTip("用于学习三角波特征的数据点数");
    learningLayout->addRow("学习数据点数:", m_learningDataCountSpin);

    mainLayout->addWidget(learningGroup);

    // 重置按钮
    m_resetButton = new QPushButton("恢复默认值", this);
    connect(m_resetButton, &QPushButton::clicked, this, &TriangleDetectorConfigDialog::onResetToDefaults);

    // 对话框按钮
    m_buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(m_buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(m_buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(m_resetButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_buttonBox);

    mainLayout->addLayout(buttonLayout);
}

void TriangleDetectorConfigDialog::setCurrentConfig(double peakTolerance, double valleyTolerance,
                                                   double slopeTolerance, double jumpThreshold,
                                                   int learningDataCount,
                                                   int risingEdgeThreshold, int fallingEdgeThreshold)
{
    m_peakToleranceSpin->setValue(peakTolerance);
    m_valleyToleranceSpin->setValue(valleyTolerance);
    m_slopeToleranceSpin->setValue(slopeTolerance);
    m_jumpThresholdSpin->setValue(jumpThreshold);
    m_learningDataCountSpin->setValue(learningDataCount);

    // 【修改】设置长度阈值
    m_risingEdgeThresholdSpin->setValue(risingEdgeThreshold);
    m_fallingEdgeThresholdSpin->setValue(fallingEdgeThreshold);
}

void TriangleDetectorConfigDialog::onResetToDefaults()
{
    m_peakToleranceSpin->setValue(10.0);
    m_valleyToleranceSpin->setValue(10.0);
    m_slopeToleranceSpin->setValue(10.0);
    m_jumpThresholdSpin->setValue(20.0);
    m_learningDataCountSpin->setValue(1500);

    // 【修改】重置长度阈值
    m_risingEdgeThresholdSpin->setValue(10);
    m_fallingEdgeThresholdSpin->setValue(10);
}
