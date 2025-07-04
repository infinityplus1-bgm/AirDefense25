import logging
import logging
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox, QComboBox, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt, QSize
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QHBoxLayout, QLabel

class ImageView(QWidget):
    """
    Displays live camera feed.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()
        self.setFixedSize(960 , 540)

    def _init_ui(self):
        main_layout = QVBoxLayout(self)

        # Group: camera feed
        group = QGroupBox("Camera View")
        group_layout = QVBoxLayout(group)
        group_layout.setContentsMargins(4,4,4,4)

        # Camera display label
        self.label = QLabel(self)
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label.setAlignment(Qt.AlignCenter)
        group_layout.addWidget(self.label)

        main_layout.addWidget(group, stretch=1)
        self.setLayout(main_layout)

    def update_image(self, pixmap: QPixmap):
        """Receives a QPixmap, scales it to 720p, and updates the label."""
        if pixmap.isNull():
            return
        scaled_pixmap = pixmap.scaled(QSize(960 , 540), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        self.label.setPixmap(scaled_pixmap)

    def cleanup_resources(self):
        """Placeholder for any cleanup needed."""
        logging.info("Image view resources cleaned up.")
