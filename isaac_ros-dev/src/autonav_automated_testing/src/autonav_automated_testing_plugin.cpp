#include "autonav_automated_testing/autonav_automated_testing_plugin.hpp"
#include <QProcess>
#include <QMessageBox>
#include <QDebug>

namespace autonav_automated_testing
{

Ros2LaunchThread::Ros2LaunchThread(const QString &launch_file_path, QObject *parent)
    : QThread(parent), launch_file_path_(launch_file_path) {}

void Ros2LaunchThread::run()
{
    emit status_update("in progress");
    process_ = new QProcess();
    process_->start("ros2", QStringList() << "launch" << launch_file_path_);
    process_->waitForFinished(-1);
    emit status_update("completed");
}

void Ros2LaunchThread::stop()
{
    stopped_ = true;
    if (process_)
    {
        process_->terminate();
        emit status_update("stopped");
    }
}

AutomatedTestingWidget::AutomatedTestingWidget(QWidget *parent)
    : QWidget(parent)
{
    populateTests();

    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel *header = new QLabel("Select a test to run:");
    header->setStyleSheet("font-weight: bold; font-size: 14px;");
    layout->addWidget(header);

    QScrollArea *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    QWidget *content = new QWidget();
    QVBoxLayout *content_layout = new QVBoxLayout(content);

    for (const auto &test : tests_)
    {
        QHBoxLayout *row = new QHBoxLayout();

        QLabel *id_label = new QLabel(QString::fromStdString(test.id));
        id_label->setFixedWidth(100);
        row->addWidget(id_label);

        QPushButton *btn = new QPushButton(QString::fromStdString(test.title));
        connect(btn, &QPushButton::clicked, this, [this, test]() { launch_test(test); });
        row->addWidget(btn);

        QLabel *desc = new QLabel(QString::fromStdString(test.desc));
        desc->setWordWrap(true);
        desc->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
        row->addWidget(desc);

        content_layout->addLayout(row);
    }

    content_layout->addStretch();
    scroll->setWidget(content);
    layout->addWidget(scroll);
    setLayout(layout);
}

void AutomatedTestingWidget::populateTests()
{
    tests_ = {
        {"[Test ID: 001]", "GPS Calibration Test", "t001_GPS_Cal.launch.py",
         "Ensure open area for GPS calibration.", "t001.jpg"},
        {"[Test ID: 002]", "Line Compliance Test", "t002_Line_Comp.launch.py",
         "Robot follows a straight line.", "t002.jpg"},
        {"[Test ID: 003]", "Forward Movement Test", "t003_Forward_Move.launch.py",
         "Robot moves forward on flat surface.", "t003.jpg"},
    };
}

void AutomatedTestingWidget::launch_test(const TestItem &test)
{
    QString launch_path = QString::fromStdString(test.launch_file);

    if (!QFile::exists(launch_path))
    {
        QMessageBox::critical(this, "Error", "Launch file not found:\n" + launch_path);
        return;
    }

    auto *thread = new Ros2LaunchThread(launch_path);
    connect(thread, &Ros2LaunchThread::status_update, this, [this, test](const QString &status) {
        QMessageBox::information(this, "Status",
                                 QString::fromStdString(test.title) + ": " + status);
    });
    thread->start();
}

Plugin::Plugin() : rqt_gui_cpp::Plugin()
{
    setObjectName("AutomatedTestingPlugin");
}

void Plugin::initPlugin(qt_gui_cpp::PluginContext &context)
{
    widget_ = new AutomatedTestingWidget();
    context.addWidget(widget_);
    qDebug() << "AutoNav plugin initialized!";
}

} // namespace autonav_automated_testing

#include "autonav_automated_testing_plugin.moc"