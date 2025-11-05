#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QThread>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSizePolicy>
#include <QMessageBox>
#include <QFile>
#include <QProcess>
#include <QDialog>
#include <QTextEdit>

#include <vector>
#include <string>

namespace autonav_automated_testing
{

struct TestItem
{
    std::string id;
    std::string title;
    std::string launch_file;
    std::string desc;
    std::string image;
};

class TestOutputDialog : public QDialog
{
    Q_OBJECT
public:
    explicit TestOutputDialog(const QString &test_name, QWidget *parent = nullptr);
    
public slots:
    void update_status(const QString &status);
    void append_output(const QString &output);

private:
    QLabel *status_label_;
    QTextEdit *output_text_;
    QPushButton *close_button_;
};

class Ros2LaunchThread : public QThread
{
    Q_OBJECT
public:
    explicit Ros2LaunchThread(const QString &launch_file_path, QObject *parent = nullptr);
    // void run() override;
    void stop();

signals:
    void status_update(const QString &status);
    void output_ready(const QString &output);

protected:
    void run() override;

private:
    QProcess *process_ = nullptr;
    QString launch_file_path_;
    bool stopped_ = false;
};

class AutomatedTestingWidget : public QWidget
{
    Q_OBJECT
public:
    explicit AutomatedTestingWidget(QWidget *parent = nullptr);

private slots:
    void launch_test(const TestItem &test);

private:
    std::vector<TestItem> tests_;
    void populateTests();
};

class Plugin : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    Plugin();
    void initPlugin(qt_gui_cpp::PluginContext &context) override;
    void shutdownPlugin() override {}

private:
    AutomatedTestingWidget *widget_;
};

} // namespace autonav_automated_testing
