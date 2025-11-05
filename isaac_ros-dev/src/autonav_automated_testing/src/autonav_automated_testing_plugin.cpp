#include "autonav_automated_testing/autonav_automated_testing_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QProcess>
#include <QMessageBox>
#include <QDebug>

namespace autonav_automated_testing
{

Ros2LaunchThread::Ros2LaunchThread(const QString &launch_file_path, QObject *parent)
    : QThread(parent), launch_file_path_(launch_file_path) {}

void Ros2LaunchThread::run()
{
    emit status_update("starting");
    process_ = new QProcess();
    
    // Capture output for debugging
    process_->setProcessChannelMode(QProcess::MergedChannels);
    
    process_->start("ros2", QStringList() << "launch" << launch_file_path_);
    
    if (!process_->waitForStarted(5000))
    {
        emit status_update("failed_to_start");
        emit output_ready("Failed to start ros2 launch command");
        return;
    }
    
    emit status_update("running");
    
    // Wait for process to finish (the automater will handle the test duration)
    process_->waitForFinished(-1);
    
    // Check exit code
    int exit_code = process_->exitCode();
    QString output = process_->readAll();
    
    emit output_ready(output);
    
    if (exit_code == 0)
    {
        emit status_update("completed");
    }
    else
    {
        emit status_update("failed");
    }
}

void Ros2LaunchThread::stop()
{
    stopped_ = true;
    if (process_)
    {
        process_->terminate();
        if (!process_->waitForFinished(3000))
        {
            process_->kill();
        }
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

        // Add image if it exists
        QLabel *image_label = new QLabel();
        QString image_path = QString::fromStdString(test.image);
        QPixmap pixmap(image_path);
        if (!pixmap.isNull())
        {
            image_label->setPixmap(pixmap.scaled(100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
        else
        {
            image_label->setText("No Image");
            image_label->setAlignment(Qt::AlignCenter);
        }
        image_label->setFixedSize(100, 100);
        row->addWidget(image_label);

        content_layout->addLayout(row);
    }

    content_layout->addStretch();
    scroll->setWidget(content);
    layout->addWidget(scroll);
    setLayout(layout);
}

void AutomatedTestingWidget::populateTests()
{
    // Get the package share directory to find images and launch files
    std::string package_share_dir;
    try {
        package_share_dir = ament_index_cpp::get_package_share_directory("autonav_automated_testing");
    } catch (const std::exception& e) {
        qDebug() << "Failed to find package share directory:" << e.what();
        package_share_dir = "";
    }
    
    std::string image_dir = package_share_dir + "/images/";
    std::string launch_dir = package_share_dir + "/launch/";
    
    tests_ = {
        {"[Test ID: t001]", "GPS Calibration Test", launch_dir + "t001_GPS_Cal.launch.py",
         "Ensure open area for GPS calibration.", image_dir + "t001.jpg"},
        {"[Test ID: t002]", "Line Compliance Test", launch_dir + "t002_Line_Comp.launch.py",
         "Robot follows a straight line.", image_dir + "t002.jpg"},
        {"[Test ID: t003]", "Forward Movement Test", launch_dir + "t003_Forward_Move.launch.py",
         "Robot moves forward on flat surface.", image_dir + "t003.jpg"},
        {"[Test ID: t004]", "Object Detection Test", launch_dir + "t004_Obj_Detect.launch.py",
         "Test object detection capabilities.", image_dir + "t004.jpg"},
        {"[Test ID: t005]", "Path Planning Test", launch_dir + "t005_Path_Plan.launch.py",
         "Test autonomous path planning.", image_dir + "t005.jpg"},
        {"[Test ID: t006]", "Simulation Test", launch_dir + "t006_Simulation.launch.py",
         "Run robot in simulation environment.", image_dir + "t006.jpg"},
        {"[Test ID: t007]", "Speed Test", launch_dir + "t007_Speed.launch.py",
         "Test robot speed performance.", image_dir + "t007.jpg"},
        {"[Test ID: t008]", "Acceleration Test", launch_dir + "t008_Acceleration.launch.py",
         "Test robot acceleration performance.", image_dir + "t008.jpg"},
        {"[Test ID: t009]", "Rotation Test", launch_dir + "t009_Rotation.launch.py",
         "Test robot rotation capabilities.", image_dir + "t009.jpg"},
        {"[Test ID: t010]", "Steering Drift Test", launch_dir + "t010_Steering_Drift.launch.py",
         "Test steering accuracy and drift.", image_dir + "t010.jpg"},
        {"[Test ID: t011]", "Incline Performance Test", launch_dir + "t011_Incline_Performance.launch.py",
         "Test robot performance on inclines.", image_dir + "t011.jpg"},
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
    
    // Handle status updates
    connect(thread, &Ros2LaunchThread::status_update, this, [this, test](const QString &status) {
        QString message;
        QMessageBox::Icon icon;
        
        if (status == "starting") {
            message = "Starting test...";
            icon = QMessageBox::Information;
        } else if (status == "running") {
            message = "Test is now running. The automater will manage the test execution.\n\n"
                     "The test will complete automatically when finished.";
            icon = QMessageBox::Information;
        } else if (status == "completed") {
            message = "Test completed successfully!";
            icon = QMessageBox::Information;
        } else if (status == "failed") {
            message = "Test failed or encountered an error.\n"
                     "This is expected if running without robot hardware.";
            icon = QMessageBox::Warning;
        } else if (status == "failed_to_start") {
            message = "Failed to start the test launch file.";
            icon = QMessageBox::Critical;
        } else if (status == "stopped") {
            message = "Test was stopped.";
            icon = QMessageBox::Warning;
        } else {
            return; // Don't show unknown statuses
        }
        
        QMessageBox msgBox(icon, QString::fromStdString(test.title), message);
        msgBox.exec();
    });
    
    // Handle output (for debugging)
    connect(thread, &Ros2LaunchThread::output_ready, this, [this, test](const QString &output) {
        if (!output.isEmpty()) {
            qDebug() << "Test output for" << QString::fromStdString(test.title) << ":\n" << output;
        }
    });
    
    // Clean up thread when finished
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    
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

PLUGINLIB_EXPORT_CLASS(autonav_automated_testing::Plugin, rqt_gui_cpp::Plugin)

#include "autonav_automated_testing_plugin.moc"