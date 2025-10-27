import os
import subprocess
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QScrollArea, QSizePolicy, QMessageBox
)
from python_qt_binding.QtCore import Qt, QThread, Signal

from rqt_gui_py.plugin import Plugin

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

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))

def get_launch_file_path(launch_file):
    return os.path.join(PACKAGE_DIR, launch_file)

class Ros2LaunchThread(QThread):
    status_update = Signal(str)

    def __init__(self, launch_file_path):
        super().__init__()
        self.launch_file_path = launch_file_path
        self._process = None
        self._stopped = False

    def run(self):
        self.status_update.emit("in progress")
        try:
            self._process = subprocess.Popen(
                ["ros2", "launch", self.launch_file_path],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self._process.wait()
            self.status_update.emit("completed")
        except Exception as e:
            self.status_update.emit(f"error: {e}")

    def stop(self):
        self._stopped = True
        if self._process:
            self._process.terminate()
            self.status_update.emit("stopped")

class AutomatedTestingWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()

        header = QLabel("Select a test to run:")
        header.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(header)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        content_layout = QVBoxLayout()
        content.setLayout(content_layout)

        for test in TESTS:
            row = QHBoxLayout()
            id_label = QLabel(test["id"])
            id_label.setFixedWidth(100)
            row.addWidget(id_label)

            btn = QPushButton(test["title"])
            btn.clicked.connect(lambda _, t=test: self.launch_test(t))
            row.addWidget(btn)

            desc = QLabel(test["desc"])
            desc.setWordWrap(True)
            desc.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
            row.addWidget(desc)

            content_layout.addLayout(row)

        content_layout.addStretch()
        scroll.setWidget(content)
        layout.addWidget(scroll)
        self.setLayout(layout)

    def launch_test(self, test):
        path = get_launch_file_path(test["launch_file"])
        if not os.path.exists(path):
            QMessageBox.critical(self, "Error", f"Launch file not found:\n{path}")
            return

        self.thread = Ros2LaunchThread(path)
        self.thread.status_update.connect(
            lambda s: QMessageBox.information(self, "Status", f"{test['title']}: {s}")
        )
        self.thread.start()


class AutomatedTestingPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("AutomatedTestingPlugin")
        self._widget = AutomatedTestingWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + f" ({context.serial_number()})")
        context.add_widget(self._widget)