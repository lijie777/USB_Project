#include <QApplication>
#include <QStyleFactory>
#include <QPalette>
#include <QDir>
#include <QMessageBox>
#include "USBVisualizerMainWindow.h"
#include "MCLogger.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);


    // 初始化日志系统
    MCLogger::getInstance().InitLog("USBVisualizer", "logs");

    app.setAttribute(Qt::AA_DontUseNativeMenuBar);
    // 设置应用程序信息
    app.setApplicationName("USB数据实时可视化系统");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("USB Visualizer");

    // 设置深色主题
    app.setStyle(QStyleFactory::create("Fusion"));

    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    app.setPalette(darkPalette);

    // 创建主窗口
    USBVisualizerMainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}
