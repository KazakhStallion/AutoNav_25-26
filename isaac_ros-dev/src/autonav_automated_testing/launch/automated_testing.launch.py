import sys
import os
import subprocess
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QDialog, QMessageBox, QScrollArea, QSizePolicy
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal

# --- ROS2 Launch Integration (for reference, not used directly in GUI) ---
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import PathJoinSubstitution

TESTS = [
    {
        "id": "[Test ID: 001]",
        "title": "GPS Calibration Test",
        "launch_file": "t001_GPS_Cal.launch.py",
        "desc": "Prerequisites:\nEnsure a flat, wide 6[m]x6[m] open outdoor area for this test to be executed.\n\nTest Description:\nSets a triangle of GPS waypoints around the robot and the robot will automatically drive to them in sequence.",
        "image": "t001.jpg",
    },
    {
        "id": "[Test ID: 002]",
        "title": "Line Compliance Test",
        "launch_file": "t002_Line_Comp.launch.py",
        "desc": "Prerequisites:\nEnsure a flat, straight 30[m] long line-only test course is in front of the robot for this test to be executed.\n\nTest Description:\nRobot will move forward and will attempt to maintain position between the white lines.",
        "image": "t002.jpg",
    },
    {
        "id": "[Test ID: 003]",
        "title": "Forward Movement Test",
        "launch_file": "t003_Forward_Move.launch.py",
        "desc": "Prerequisites:\nEnsure a flat, short but windy test course for the robot for this test to be executed.\n\nTest Description:\nRobot will attempt to move through the winds without turning around.",
        "image": "t003.jpg",
    },
    {
        "id": "[Test ID: 004]",
        "title": "Object Detection Test",
        "launch_file": "t004_Obj_Detect.launch.py",
        "desc": "Prerequisites:\nEnsure objects are in front of the robot camera and there is sufficient space this test to be executed.\n\nTest Description:\nImages of detection rectangles overlay will be exported and the robot will navigate around the objects.",
        "image": "t004.jpg",
    },
    {
        "id": "[Test ID: 005]",
        "title": "Path Planning Test",
        "launch_file": "t005_Path_Plan.launch.py",
        "desc": "Prerequisites:\nEnsure objects are in front of the robot camera and there is sufficient space for this test to be executed.\n\nTest Description:\nImages of path planning will be exported and the robot will choose a path to move along.",
        "image": "t005.jpg",
    },
    {
        "id": "[Test ID: 006]",
        "title": "Simulation Test",
        "launch_file": "t006_Simulation.launch.py",
        "desc": "Prerequisites:\nTBD.\n\nTest Description:\nTBD.",
        "image": None,
    },
    {
        "id": "[Test ID: 007]",
        "title": "Speed Test",
        "launch_file": "t007_Speed.launch.py",
        "desc": "Prerequisites:\nTBD.\n\nTest Description:\nTBD.",
        "image": None,
    },
    {
        "id": "[Test ID: 008]",
        "title": "Acceleration Test",
        "launch_file": "t008_Acceleration.launch.py",
        "desc": "Prerequisites:\nTBD.\n\nTest Description:\nTBD.",
        "image": None,
    },
    {
        "id": "[Test ID: 009]",
        "title": "Rotation Test",
        "launch_file": "t009_Rotation.launch.py",
        "desc": "Prerequisites:\nTBD.\n\nTest Description:\nTBD.",
        "image": None,
    },
    {
        "id": "[Test ID: 010]",
        "title": "Steering Drift Test",
        "launch_file": "t010_Steering_Drift.launch.py",
        "desc": "Prerequisites:\nTBD.\n\nTest Description:\nTBD.",
        "image": None,
    },
    {
        "id": "[Test ID: 011]",
        "title": "Incline Performance Test",
        "launch_file": "t011_Incline_Performance.launch.py",
        "desc": "Prerequisites:\nTBD.\n\nTest Description:\nTBD.",
        "image": None,
    },
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

        outer_layout = QVBoxLayout()
        header = QLabel("Select a test to run:")
        header.setStyleSheet("font-weight: bold; font-size: 14px;")
        outer_layout.addWidget(header)

        # Scroll area to contain many tests
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        content = QWidget()
        content_layout = QVBoxLayout()
        content.setLayout(content_layout)

        # Create a row for each test inside the scrollable content: [ID label] [Run button] [Description] [Image 100x100]
        for test in TESTS:
            row = QHBoxLayout()
            row.setAlignment(Qt.AlignmentFlag.AlignVCenter)  # Align all items vertically center

            # ID label on the left
            id_label = QLabel(test.get("id", ""))
            id_label.setFixedWidth(120)
            id_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            row.addWidget(id_label, alignment=Qt.AlignmentFlag.AlignVCenter)

            # Run button (title shown)
            btn = QPushButton(test["title"])
            btn.clicked.connect(lambda checked, t=test: self.open_test_dialog(t))
            btn.setFixedWidth(200)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    border: 1px solid #cccccc;
                    padding: 5px;
                }
                QPushButton:hover {
                    background-color: #f5f5f5;
                }
            """)
            row.addWidget(btn, alignment=Qt.AlignmentFlag.AlignVCenter)

            # Description (expandable)
            desc_label = QLabel(test.get("desc", ""))
            desc_label.setWordWrap(True)
            desc_label.setStyleSheet("""
                color: #333333;
                padding: 4px;
                background-color: transparent;
            """)
            desc_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
            desc_label.setMinimumHeight(100)  # Match image height
            desc_label.adjustSize()
            row.addWidget(desc_label, stretch=1, alignment=Qt.AlignmentFlag.AlignVCenter)

            # Image placeholder to the right (100x100)
            img_label = QLabel()
            img_label.setFixedSize(100, 100)
            img_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            img_label.setStyleSheet("""
                background-color: #e0e0e0;
                border: 1px solid #a0a0a0;
                min-width: 100px;
                min-height: 100px;
                max-width: 100px;
                max-height: 100px;
            """)
            img_path = test.get("image")
            if img_path and os.path.exists(img_path):
                from PyQt6.QtGui import QPixmap
                pix = QPixmap(img_path)
                if not pix.isNull():
                    # Force exact 100x100 size without keeping aspect ratio
                    img_label.setPixmap(pix.scaled(100, 100, Qt.AspectRatioMode.IgnoreAspectRatio, Qt.TransformationMode.SmoothTransformation))
                else:
                    img_label.setText("No Image")
            else:
                img_label.setText("No Image")

            # Ensure image is vertically centered in the row
            row.addWidget(img_label, alignment=Qt.AlignmentFlag.AlignVCenter)

            # wrap row in a container widget to preserve spacing and allow styling if needed
            row_widget = QWidget()
            # Add border and padding around the row, keeping image size exact
            row_widget.setStyleSheet("""
                margin: 8px 0;
                padding: 8px;
                min-height: 100px;
                border: 1px solid #e0e0e0;
                border-radius: 2px;
            """)
            row_widget.setLayout(row)
            # Set size policy to ensure the widget can expand but maintains minimum height
            row_widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.MinimumExpanding)
            content_layout.addWidget(row_widget)

        # Add stretch so content groups to top
        content_layout.addStretch()

        scroll.setWidget(content)
        outer_layout.addWidget(scroll)

        # Spacer and Close button (outside scroll area so always visible)
        close_row = QHBoxLayout()
        close_row.addStretch()
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        close_row.addWidget(close_btn)
        outer_layout.addLayout(close_row)

        central.setLayout(outer_layout)
        self.setCentralWidget(central)
        # Make window a bit larger to accommodate descriptions and images
        self.resize(1200, 600)

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