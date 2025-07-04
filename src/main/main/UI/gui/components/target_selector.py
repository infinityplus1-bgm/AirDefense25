import logging
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QComboBox, QPushButton, QGroupBox, QLabel, QHBoxLayout
from PyQt5.QtCore import pyqtSignal

class TargetSelector(QWidget):
    """
    A widget for selecting a target from a list and submitting the choice.
    """
    target_selected = pyqtSignal(int)
    target_cleared = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.targets = {}
        self._init_ui()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        group = QGroupBox("Target Selection")
        group_layout = QVBoxLayout(group)

        self.target_combo = QComboBox(self)
        group_layout.addWidget(self.target_combo)

        buttons_layout = QHBoxLayout()
        self.submit_button = QPushButton("Submit Target", self)
        self.submit_button.clicked.connect(self._submit_target)
        self.clear_button = QPushButton("Clear", self)
        self.clear_button.clicked.connect(self._clear_target)
        buttons_layout.addWidget(self.submit_button)
        buttons_layout.addWidget(self.clear_button)
        group_layout.addLayout(buttons_layout)

        # Indicator label
        indicator_layout = QHBoxLayout()
        indicator_label = QLabel("Selected Target:", self)
        self.current_target_label = QLabel("None", self)
        self.current_target_label.setStyleSheet("font-weight: bold; color: #cba6f7;") # Mauve
        indicator_layout.addWidget(indicator_label)
        indicator_layout.addWidget(self.current_target_label)
        indicator_layout.addStretch(1)
        group_layout.addLayout(indicator_layout)

        main_layout.addWidget(group)
        self.setLayout(main_layout)

    def update_targets(self, target_ids):
        self.target_combo.clear()
        self.targets.clear()
        
        if not target_ids:
            self.target_combo.addItem("No targets available")
            self.target_combo.setEnabled(False)
            self.submit_button.setEnabled(False)
        else:
            self.target_combo.setEnabled(True)
            self.submit_button.setEnabled(True)
            for i, target_id in enumerate(target_ids):
                display_text = f"Target ID: {target_id}"
                self.targets[display_text] = target_id
                self.target_combo.addItem(display_text)

    def _submit_target(self):
        selected_target_text = self.target_combo.currentText()
        if selected_target_text in self.targets:
            selected_target_id = self.targets[selected_target_text]
            self.current_target_label.setText(f"ID: {selected_target_id}")
            self.target_selected.emit(selected_target_id)
            logging.info(f"Target submitted: {selected_target_text} (ID: {selected_target_id})")

    def _clear_target(self):
        self.target_combo.setCurrentIndex(-1)
        self.current_target_label.setText("None")
        self.target_cleared.emit()
        logging.info("Target selection cleared.")
