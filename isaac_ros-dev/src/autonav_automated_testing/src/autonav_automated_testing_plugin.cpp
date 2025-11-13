#include "autonav_automated_testing/autonav_automated_testing_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QProcess>
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QRegularExpression>
#include <QDir>
#include <QFileInfo>
#include <QTextStream>

namespace autonav_automated_testing
{

TestOutputDialog::TestOutputDialog(const QString &test_name, const std::string &test_id, QWidget *parent)
    : QDialog(parent), test_id_(test_id)
{
    setWindowTitle("Test Output - " + test_name);
    setMinimumSize(700, 500);
    
    // Initialize ROS node for publishing to estop
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("test_output_dialog_" + test_id);
    estop_pub_ = node_->create_publisher<std_msgs::msg::String>("/estop", 10);
    
    QVBoxLayout *layout = new QVBoxLayout(this);
    
    // Status label at the top
    status_label_ = new QLabel("Status: Initializing...");
    status_label_->setStyleSheet("QLabel { background-color: #e3f2fd; padding: 10px; "
                                  "border: 2px solid #2196f3; border-radius: 5px; "
                                  "font-weight: bold; font-size: 14px; }");
    layout->addWidget(status_label_);
    
    // Scrollable text output area (terminal-like)
    output_text_ = new QTextEdit();
    output_text_->setReadOnly(true);
    output_text_->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #d4d4d4; "
                                 "font-family: 'Courier New', monospace; font-size: 11px; }");
    output_text_->setLineWrapMode(QTextEdit::WidgetWidth); // Enable word wrap
    layout->addWidget(output_text_);
    
    // Button layout at the bottom
    QHBoxLayout *button_layout = new QHBoxLayout();
    
    // E-Stop button
    estop_button_ = new QPushButton("E-STOP");
    estop_button_->setStyleSheet("QPushButton { background-color: #f44336; color: white; "
                                  "font-weight: bold; padding: 8px; }");
    connect(estop_button_, &QPushButton::clicked, this, &TestOutputDialog::trigger_estop);
    button_layout->addWidget(estop_button_);
    
    // Load Log button
    load_log_button_ = new QPushButton("Load Log File");
    load_log_button_->setStyleSheet("QPushButton { background-color: #2196f3; color: white; "
                                     "padding: 8px; }");
    connect(load_log_button_, &QPushButton::clicked, this, &TestOutputDialog::load_log_file);
    button_layout->addWidget(load_log_button_);
    
    // Close button
    close_button_ = new QPushButton("Close");
    close_button_->setEnabled(false); // Disabled until test completes
    connect(close_button_, &QPushButton::clicked, this, &QDialog::accept);
    button_layout->addWidget(close_button_);
    
    layout->addLayout(button_layout);
    setLayout(layout);
    
    // Initial message
    append_output(QString("[%1] Test window opened").arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
}

void TestOutputDialog::update_status(const QString &status)
{
    QString status_text;
    QString style_color;
    
    if (status == "starting") {
        status_text = "Status: Starting test...";
        style_color = "#2196f3"; // Blue
    } else if (status == "running") {
        status_text = "Status: Test is running...";
        style_color = "#ff9800"; // Orange
    } else if (status == "completed") {
        status_text = "Status: Test completed successfully ✓";
        style_color = "#4caf50"; // Green
        close_button_->setEnabled(true);
    } else if (status == "failed") {
        status_text = "Status: Test failed (check output below)";
        style_color = "#f44336"; // Red
        close_button_->setEnabled(true);
    } else if (status == "failed_to_start") {
        status_text = "Status: Failed to start test";
        style_color = "#f44336"; // Red
        close_button_->setEnabled(true);
    } else if (status == "stopped") {
        status_text = "Status: Test was stopped";
        style_color = "#9e9e9e"; // Gray
        close_button_->setEnabled(true);
    } else {
        status_text = "Status: " + status;
        style_color = "#2196f3"; // Blue
    }
    
    status_label_->setText(status_text);
    status_label_->setStyleSheet(QString("QLabel { background-color: #e3f2fd; padding: 10px; "
                                         "border: 2px solid %1; border-radius: 5px; "
                                         "font-weight: bold; font-size: 14px; }").arg(style_color));
    
    append_output(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss"), status_text));
}

void TestOutputDialog::append_output(const QString &output)
{
    // Parse output to add appropriate indicators and color coding
    QStringList lines = output.split('\n');
    
    for (const QString &line : lines) {
        if (line.trimmed().isEmpty()) {
            continue;
        }
        
        QString formatted_line;
        QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss");
        QString colored_line = line.trimmed();
        
        // Color code INFO and ERROR in the text
        colored_line.replace("[INFO]", "<span style='color: #4fc3f7;'>[INFO]</span>");
        colored_line.replace("[ERROR]", "<span style='color: #f44336;'>[ERROR]</span>");
        colored_line.replace("[WARN]", "<span style='color: #ff9800;'>[WARN]</span>");
        colored_line.replace("[DEBUG]", "<span style='color: #9c27b0;'>[DEBUG]</span>");
        colored_line.replace("[FATAL]", "<span style='color: #d32f2f; font-weight: bold;'>[FATAL]</span>");
        
        // Check for different types of messages and add appropriate tags
        if (line.contains("launch", Qt::CaseInsensitive) || 
            line.contains("Launched", Qt::CaseInsensitive) ||
            line.contains("launch_ros", Qt::CaseInsensitive)) {
            formatted_line = QString("[%1] [launch] %2").arg(timestamp, colored_line);
        }
        else if (line.contains("automater", Qt::CaseInsensitive) || 
                 line.contains("t00", Qt::CaseInsensitive) ||
                 line.contains("Test", Qt::CaseInsensitive)) {
            formatted_line = QString("[%1] [automater] %2").arg(timestamp, colored_line);
        }
        else if (line.contains("[INFO]") || line.contains("[WARN]") || 
                 line.contains("[ERROR]") || line.contains("[DEBUG]") ||
                 line.contains("[FATAL]")) {
            // ROS 2 node output - extract node name if present
            QRegularExpression re("\\[([^\\]]+)\\]");
            QRegularExpressionMatch match = re.match(line);
            if (match.hasMatch()) {
                QString node_info = match.captured(1);
                // If it's a log level, look for the next bracket which might be the node name
                if (node_info == "INFO" || node_info == "WARN" || node_info == "ERROR" || 
                    node_info == "DEBUG" || node_info == "FATAL") {
                    formatted_line = QString("[%1] [node] %2").arg(timestamp, colored_line);
                } else {
                    formatted_line = QString("[%1] [node:%2] %3").arg(timestamp, node_info, colored_line);
                }
            } else {
                formatted_line = QString("[%1] [node] %2").arg(timestamp, colored_line);
            }
        }
        else if (line.contains("ERROR", Qt::CaseInsensitive) || 
                 line.contains("error", Qt::CaseSensitive)) {
            formatted_line = QString("[%1] [error] %2").arg(timestamp, colored_line);
        }
        else if (line.contains("WARNING", Qt::CaseInsensitive) || 
                 line.contains("warn", Qt::CaseInsensitive)) {
            formatted_line = QString("[%1] [warning] %2").arg(timestamp, colored_line);
        }
        else if (line.startsWith("Process exited")) {
            formatted_line = QString("[%1] [system] %2").arg(timestamp, colored_line);
        }
        else {
            // Generic output
            formatted_line = QString("[%1] [output] %2").arg(timestamp, colored_line);
        }
        
        output_text_->append(formatted_line);
    }
    
    // Auto-scroll to bottom
    QTextCursor cursor = output_text_->textCursor();
    cursor.movePosition(QTextCursor::End);
    output_text_->setTextCursor(cursor);
}

void TestOutputDialog::trigger_estop()
{
    auto msg = std_msgs::msg::String();
    msg.data = "STOP";
    estop_pub_->publish(msg);
    
    append_output("[E-STOP] Emergency stop triggered by user");
    
    QMessageBox::warning(this, "E-Stop Triggered", 
                         "Emergency stop signal sent to /estop topic.\n"
                         "The test should terminate shortly.");
}

void TestOutputDialog::load_log_file()
{
    // Look for log files in /autonav/logs directory
    QString log_dir = "/autonav/logs";
    QString log_pattern = QString::fromStdString(test_id_) + "_*.csv";
    
    QDir dir(log_dir);
    QStringList filters;
    filters << log_pattern;
    QFileInfoList log_files = dir.entryInfoList(filters, QDir::Files, QDir::Time);
    
    if (log_files.isEmpty()) {
        append_output(QString("[Load Log] No log files found matching pattern: %1/%2").arg(log_dir, log_pattern));
        QMessageBox::information(this, "No Log Files", 
                                 QString("No log files found for test ID: %1\n\n"
                                        "Log files are created when tests run on the robot.").arg(QString::fromStdString(test_id_)));
        return;
    }
    
    // Use the most recent log file
    QString log_file_path = log_files.first().absoluteFilePath();
    
    QFile log_file(log_file_path);
    if (!log_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        append_output(QString("[Load Log] Failed to open log file: %1").arg(log_file_path));
        QMessageBox::critical(this, "Error", "Failed to open log file:\n" + log_file_path);
        return;
    }
    
    output_text_->clear();
    append_output(QString("[Load Log] Loading log file: %1").arg(log_file_path));
    append_output(QString("=").repeated(80));
    
    QTextStream in(&log_file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        output_text_->append(line);
    }
    
    log_file.close();
    append_output(QString("=").repeated(80));
    append_output("[Load Log] Log file loaded successfully");
}


Ros2LaunchThread::Ros2LaunchThread(const QString &launch_file_path, QObject *parent)
    : QThread(parent), launch_file_path_(launch_file_path) {}

void Ros2LaunchThread::run()
{
    emit status_update("starting");
    process_ = new QProcess();
    
    // Capture output for debugging
    process_->setProcessChannelMode(QProcess::MergedChannels);
    
    // Connect to read output as it comes
    connect(process_, &QProcess::readyReadStandardOutput, this, [this]() {
        QString output = process_->readAllStandardOutput();
        if (!output.isEmpty()) {
            emit output_ready(output.trimmed());
        }
    });
    
    process_->start("ros2", QStringList() << "launch" << launch_file_path_);
    
    if (!process_->waitForStarted(5000))
    {
        emit status_update("failed_to_start");
        emit output_ready("ERROR: Failed to start ros2 launch command");
        return;
    }
    
    emit status_update("running");
    
    // Wait for process to finish (the automater will handle the test duration)
    process_->waitForFinished(-1);
    
    // Read any remaining output
    QString remaining = process_->readAll();
    if (!remaining.isEmpty()) {
        emit output_ready(remaining.trimmed());
    }
    
    // Check exit code
    int exit_code = process_->exitCode();
    
    if (exit_code == 0)
    {
        emit status_update("completed");
    }
    else
    {
        emit output_ready(QString("Process exited with code: %1").arg(exit_code));
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
        // REMOVED - files deleted to keep branch clean
        // {"[Test ID: t001]", "GPS Calibration Test", launch_dir + "t001_GPS_Cal.launch.py",
        //  "Ensure open area for GPS calibration.", image_dir + "t001.jpg"},
        {"[Test ID: t002]", "Line Compliance Test", launch_dir + "t002_Line_Comp.launch.py",
         "Robot follows a straight line.", image_dir + "t002.jpg"},
        // REMOVED - files deleted to keep branch clean
        // {"[Test ID: t003]", "Forward Movement Test", launch_dir + "t003_Forward_Move.launch.py",
        //  "Robot moves forward on flat surface.", image_dir + "t003.jpg"},
        // {"[Test ID: t004]", "Object Detection Test", launch_dir + "t004_Obj_Detect.launch.py",
        //  "Test object detection capabilities.", image_dir + "t004.jpg"},
        // {"[Test ID: t005]", "Path Planning Test", launch_dir + "t005_Path_Plan.launch.py",
        //  "Test autonomous path planning.", image_dir + "t005.jpg"},
        // {"[Test ID: t006]", "Simulation Test", launch_dir + "t006_Simulation.launch.py",
        //  "Run robot in simulation environment.", image_dir + "t006.jpg"},
        // {"[Test ID: t007]", "Speed Test", launch_dir + "t007_Speed.launch.py",
        //  "Test robot speed performance.", image_dir + "t007.jpg"},
        // {"[Test ID: t008]", "Acceleration Test", launch_dir + "t008_Acceleration.launch.py",
        //  "Test robot acceleration performance.", image_dir + "t008.jpg"},
        // {"[Test ID: t009]", "Rotation Test", launch_dir + "t009_Rotation.launch.py",
        //  "Test robot rotation capabilities.", image_dir + "t009.jpg"},
        // {"[Test ID: t010]", "Steering Drift Test", launch_dir + "t010_Steering_Drift.launch.py",
        //  "Test steering accuracy and drift.", image_dir + "t010.jpg"},
        // {"[Test ID: t011]", "Incline Performance Test", launch_dir + "t011_Incline_Performance.launch.py",
        //  "Test robot performance on inclines.", image_dir + "t011.jpg"},
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

    // Extract test_id from the test.id field (e.g., "[Test ID: t001]" -> "t001")
    std::string test_id = test.id;
    size_t start = test_id.find("t0");
    if (start != std::string::npos) {
        test_id = test_id.substr(start, 4); // Extract "t001", "t002", etc.
    } else {
        test_id = "unknown";
    }

    // Create the output dialog
    auto *dialog = new TestOutputDialog(QString::fromStdString(test.title), test_id);
    
    auto *thread = new Ros2LaunchThread(launch_path);
    
    // Connect status updates to dialog
    connect(thread, &Ros2LaunchThread::status_update, dialog, &TestOutputDialog::update_status);
    
    // Connect output to dialog
    connect(thread, &Ros2LaunchThread::output_ready, dialog, &TestOutputDialog::append_output);
    
    // Clean up thread and dialog when finished
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    connect(dialog, &QDialog::finished, dialog, &QObject::deleteLater);
    
    // Start the thread and show the dialog
    thread->start();
    dialog->show();
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