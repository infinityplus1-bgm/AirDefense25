# air_defense_project/gui/components/health_check_display.py
"""
QWidget for displaying system health status indicators.
"""
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame
from PyQt5.QtCore import Qt # Required for Qt.AlignCenter

class HealthCheckDisplay(QWidget):
    _COLOR_OK = "#a6e3a1"  # Green
    _COLOR_ERROR = "#f38ba8" # Red
    _COLOR_OFFLINE = "#9399b2" # Grey

    def __init__(self, parent=None):
        super().__init__(parent)
        self.status_labels = {}
        self._initUI()

    def _initUI(self):
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        health_frame = QFrame(self)
        self.frame_layout = QVBoxLayout(health_frame)
        title_label = QLabel("System Health Check", health_frame)
        title_label.setAlignment(Qt.AlignCenter)
        self.frame_layout.addWidget(title_label)
        self.layout.addWidget(health_frame)

    def update_status(self, statuses: dict):
        for node_name, status in statuses.items():
            if node_name not in self.status_labels:
                label = QLabel(self)
                self.frame_layout.addWidget(label)
                self.status_labels[node_name] = label

            label = self.status_labels[node_name]
            if status == 'healthy':
                text = f"✓ {node_name}: Healthy"
                color = self._COLOR_OK
            elif status == 'offline':
                text = f"✗ {node_name}: Offline"
                color = self._COLOR_OFFLINE
            else:
                text = f"✗ {node_name}: {status}"
                color = self._COLOR_ERROR
            
            label.setText(text)
            label.setStyleSheet(f"color: {color};")
