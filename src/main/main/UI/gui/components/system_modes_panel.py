## File: gui/components/system_modes_panel.py
"""
QWidget for selecting the system's operational mode using vertical radio buttons.
"""
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QRadioButton, QButtonGroup, QFrame
from PyQt5.QtCore import Qt, pyqtSignal

class SystemModesPanel(QWidget):
    """
    Provides vertical radio buttons for selecting system modes.
    Emits `mode_changed_signal` with the name of the selected mode.
    """
    mode_changed_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._initUI()

    def _initUI(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        group = QFrame(self)
        layout = QVBoxLayout(group)
        layout.setSpacing(5)

        title = QLabel("System Modes", self)
        title.setAlignment(Qt.AlignLeft)
        layout.addWidget(title)

        # Button group for radio buttons
        self.btn_group = QButtonGroup(self)

        # Modes list
        modes = ["Manual", "Phase One", "Phase Two", "Phase Three"]
        for idx, mode in enumerate(modes):
            rb = QRadioButton(mode, self)
            # Default select first
            if idx == 0:
                rb.setChecked(True)
            self.btn_group.addButton(rb, id=idx)
            layout.addWidget(rb)

        # Indicator label
        indicator_layout = QHBoxLayout()
        indicator_label = QLabel("Current Mode:", self)
        self.current_mode_label = QLabel(self.btn_group.checkedButton().text(), self)
        self.current_mode_label.setStyleSheet("font-weight: bold; color: #a6e3a1;") # Green
        indicator_layout.addWidget(indicator_label)
        indicator_layout.addWidget(self.current_mode_label)
        indicator_layout.addStretch(1)
        layout.addLayout(indicator_layout)

        # Connect signal: button toggled
        self.btn_group.buttonToggled.connect(self._on_button_toggled)

        main_layout.addWidget(group)
        self.setLayout(main_layout)

    def _on_button_toggled(self, button, checked):
        if checked:
            mode_text = button.text()
            self.current_mode_label.setText(mode_text)
            self.mode_changed_signal.emit(mode_text)
