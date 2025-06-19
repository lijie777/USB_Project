#ifndef TRIANGLESTATSDISPLAYDIALOG_H
#define TRIANGLESTATSDISPLAYDIALOG_H

#include <QDialog>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QPushButton>
#include <QDialogButtonBox>
#include "OptimizedTriangleAnomalyDetector.h"

class TriangleStatsDisplayDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TriangleStatsDisplayDialog(QWidget *parent = nullptr);

    void setTriangleStats(const TriangleStats& stats, int peakTolerance, int valleyTolerance, int slopeTolerance);

private:
    void setupUI();
    QString formatStats(const TriangleStats& stats, int peakTolerance, int valleyTolerance, int slopeTolerance);

    QTextEdit* m_statsDisplay;
    QDialogButtonBox* m_buttonBox;
};

#endif // TRIANGLESTATSDISPLAYDIALOG_H


