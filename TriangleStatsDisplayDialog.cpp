
#include "TriangleStatsDisplayDialog.h"

TriangleStatsDisplayDialog::TriangleStatsDisplayDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("三角波学习参数");
    setModal(true);
    resize(500, 650);

    setupUI();
}

void TriangleStatsDisplayDialog::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    m_statsDisplay = new QTextEdit(this);
    m_statsDisplay->setReadOnly(true);
    m_statsDisplay->setFont(QFont("Consolas", 10));
    mainLayout->addWidget(m_statsDisplay);

    m_buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok, this);
    connect(m_buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    mainLayout->addWidget(m_buttonBox);
}

void TriangleStatsDisplayDialog::setTriangleStats(const TriangleStats& stats, int peakTolerance, int valleyTolerance, int slopeTolerance)
{
    QString statsText = formatStats(stats, peakTolerance, valleyTolerance, slopeTolerance);
    m_statsDisplay->setHtml(statsText);
}

QString TriangleStatsDisplayDialog::formatStats(const TriangleStats& stats, int peakTolerance, int valleyTolerance, int slopeTolerance)
{
    if (!stats.isValid) {
        return "<h3 style='color: red;'>学习尚未完成或数据无效</h3>";
    }

    QString html = "<h3 style='color: green;'>三角波学习参数</h3>";
    html += "<table border='1' cellpadding='8' cellspacing='0' style='border-collapse: collapse; width: 100%;'>";

    // 基础参数
    html += "<tr><th colspan='2' style='background-color: #4CAF50; color: white;'>基础参数</th></tr>";
    html += QString("<tr><td><b>总周期数</b></td><td>%1 个</td></tr>").arg(stats.totalCycles);
    html += QString("<tr><td><b>平均周期长度</b></td><td>%1 个数据点</td></tr>").arg(stats.avgPeriodLength);
    html += QString("<tr><td><b>平均上升沿长度</b></td><td>%1 个数据点</td></tr>").arg(stats.avgRisingEdgeLength);
    html += QString("<tr><td><b>平均下降沿长度</b></td><td>%1 个数据点</td></tr>").arg(stats.avgFallingEdgeLength);

    // 波峰波谷参数
    html += "<tr><th colspan='2' style='background-color: #2196F3; color: white;'>波峰波谷参数</th></tr>";
    html += QString("<tr><td><b>平均波峰值</b></td><td>%1 ± %2</td></tr>")
               .arg(stats.avgPeakValue, 0, 'f', 2)
               .arg(stats.peakValueStdDev * peakTolerance, 0, 'f', 2);
    html += QString("<tr><td><b>平均波谷值</b></td><td>%1 ± %2</td></tr>")
               .arg(stats.avgValleyValue, 0, 'f', 2)
               .arg(stats.valleyValueStdDev * valleyTolerance, 0, 'f', 2);
    html += QString("<tr><td><b>波峰波谷差值</b></td><td>%1</td></tr>")
               .arg(stats.avgPeakValue - stats.avgValleyValue, 0, 'f', 2);

    // 斜率参数
    html += "<tr><th colspan='2' style='background-color: #FF9800; color: white;'>斜率参数</th></tr>";
    html += QString("<tr><td><b>平均上升斜率</b></td><td>%1 ± %2</td></tr>")
               .arg(stats.avgRisingSlope, 0, 'f', 4)
               .arg(stats.risingSlopeStdDev * slopeTolerance, 0, 'f', 4);
    html += QString("<tr><td><b>平均下降斜率</b></td><td>%1 ± %2</td></tr>")
               .arg(stats.avgFallingSlope, 0, 'f', 4)
               .arg(stats.fallingSlopeStdDev * slopeTolerance, 0, 'f', 4);

    html += "</table>";

    html += "<br><p style='color: #666; font-size: 12px;'>";
    html += "注：± 后面的数值表示 标准差*容差倍数，用于异常检测的阈值计算，具体容差倍数参考检测器配置对话框";
    html += "</p>";

    return html;
}
