## File: gui/components/no_fire_zone.py
from PyQt5.QtWidgets import QWidget, QGroupBox, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QFormLayout, QLineEdit
from PyQt5.QtCore import pyqtSignal

class NoFireZoneWidget(QWidget):
    """
    Widget to define a no-fire zone in degrees with lower and upper limits.
    Emits:
      - zoneDefined(dict) with {'lower_deg': float, 'upper_deg': float}
      - zoneCleared()
    """
    zoneDefined = pyqtSignal(dict)
    zoneCleared = pyqtSignal()

    def __init__(self, parent=None):
        super(NoFireZoneWidget, self).__init__(parent)
        self._init_ui()

    def _init_ui(self):
        group = QGroupBox("No-Fire Zone (Degrees)")
        layout = QVBoxLayout()

        # Instruction
        self.label = QLabel(
            "Enter the angular limits for the no-fire zone.\n"
            "• Lower limit: start angle in degrees (-180 to 180)\n"
            "• Upper limit: end angle in degrees (-180 to 180)"
        )
        layout.addWidget(self.label)

        # Input fields
        form_layout = QFormLayout()
        self.input_lower = QLineEdit()
        self.input_lower.setPlaceholderText("Lower limit (degrees)")
        self.input_upper = QLineEdit()
        self.input_upper.setPlaceholderText("Upper limit (degrees)")
        form_layout.addRow("Lower:", self.input_lower)
        form_layout.addRow("Upper:", self.input_upper)
        layout.addLayout(form_layout)

        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_set = QPushButton("Set Zone")
        self.btn_clear = QPushButton("Clear Zone")
        btn_layout.addWidget(self.btn_set)
        btn_layout.addWidget(self.btn_clear)
        layout.addLayout(btn_layout)

        # Status
        self.status = QLabel("Zone: None")
        layout.addWidget(self.status)

        group.setLayout(layout)
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group)
        main_layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(main_layout)

        # Connections
        self.btn_set.clicked.connect(self.on_set_zone)
        self.btn_clear.clicked.connect(self.on_clear_zone)

    def on_set_zone(self):
        try:
            lower = float(self.input_lower.text())
            upper = float(self.input_upper.text())
        except ValueError:
            self.status.setText("Invalid input: please enter numeric degrees.")
            return

        if not (-180 <= lower <= 180 and -180 <= upper <= 180):
            self.status.setText("Error: Degrees must be between -180 and 180.")
            return

        if lower > upper:
            self.status.setText("Error: Lower limit must be <= upper limit.")
            return

        zone = {'lower_deg': lower, 'upper_deg': upper}
        self.status.setText(f"Zone: [{lower}°, {upper}°]")
        self.zoneDefined.emit(zone)

    def on_clear_zone(self):
        self.input_lower.clear()
        self.input_upper.clear()
        self.status.setText("Zone: None")
        self.zoneCleared.emit()
