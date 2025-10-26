import sys
import os
import subprocess
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QDialog, QMessageBox
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal

# --- ROS2 Launch Integration (for reference, not used directly in GUI) ---
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import PathJoinSubstitution

TESTS = [
    {"name": "Test 1", "launch_file": "test1.launch.py"},
    {"name": "Test 2", "launch_file": "test2.launch.py"},
    {"name": "Test 3", "launch_file": "test3.launch.py"},
]

PACKAGE_NAME = "autonav_automated_testing"
LAUNCH_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),  # This launch.py's directory
)

def get_launch_file_path(launch_file):
    # Assumes launch files are in the same directory as this script
    return os.path.join(LAUNCH_DIR, launch_file)

class Ros2LaunchThread(QThread):
    status_update = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self, launch_file_path):
        super().__init__()
        self.launch_file_path = launch_file_path
        self._process = None
        self._stopped = False

    def run(self):
        self.status_update.emit("in progress")
        try:
            # Launch the ROS2 launch file as a subprocess
            self._process = subprocess.Popen(
                ["ros2", "launch", self.launch_file_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            while True:
                if self._stopped:
                    self._process.terminate()
                    self.status_update.emit("stopped")
                    break
                ret = self._process.poll()
                if ret is not None:
                    self.status_update.emit("completed")
                    break
        except Exception as e:
            self.status_update.emit(f"error: {e}")
        self.finished.emit()

    def stop(self):
        self._stopped = True
        if self._process:
            self._process.terminate()

class TestDialog(QDialog):
    def __init__(self, test_name, launch_file_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle(test_name)
        self.launch_file_path = launch_file_path
        self.status = "ready to initiate"
        self.thread = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.status_label = QLabel(f"Status: {self.status}")
        layout.addWidget(self.status_label)

        btn_layout = QHBoxLayout()
        self.run_btn = QPushButton("Run")
        self.run_btn.clicked.connect(self.on_run)
        btn_layout.addWidget(self.run_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.on_stop)
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.stop_btn)

        self.back_btn = QPushButton("Back")
        self.back_btn.clicked.connect(self.reject)
        btn_layout.addWidget(self.back_btn)

        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def on_run(self):
        if self.thread and self.thread.isRunning():
            return
        self.status = "in progress"
        self.status_label.setText(f"Status: {self.status}")
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.thread = Ros2LaunchThread(self.launch_file_path)
        self.thread.status_update.connect(self.update_status)
        self.thread.finished.connect(self.on_finished)
        self.thread.start()

    def on_stop(self):
        if self.thread:
            self.thread.stop()
            self.status = "stopped"
            self.status_label.setText(f"Status: {self.status}")
            self.stop_btn.setEnabled(False)

    def update_status(self, status):
        self.status = status
        self.status_label.setText(f"Status: {self.status}")

    def on_finished(self):
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Automated Testing GUI")
        self.setGeometry(100, 100, 400, 200)
        self.init_ui()

    def init_ui(self):
        central = QWidget()
        layout = QVBoxLayout()
        label = QLabel("Select a test to run:")
        layout.addWidget(label)
        for test in TESTS:
            btn = QPushButton(test["name"])
            btn.clicked.connect(lambda checked, t=test: self.open_test_dialog(t))
            layout.addWidget(btn)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)
        central.setLayout(layout)
        self.setCentralWidget(central)

    def open_test_dialog(self, test):
        launch_file_path = get_launch_file_path(test["launch_file"])
        if not os.path.exists(launch_file_path):
            QMessageBox.critical(self, "Error", f"Launch file not found:\n{launch_file_path}")
            return
        dlg = TestDialog(test["name"], launch_file_path, self)
        dlg.exec()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()