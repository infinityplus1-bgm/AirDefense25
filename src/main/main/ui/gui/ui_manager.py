# src/main/main/ui/gui/ui_manager.py
"""
Manages UI components for the Air Defense GUI.
"""
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QComboBox, QPushButton
from .components import (
    SystemControlButtons,
    NoFireZoneWidget,
    ImageView,
    SystemModesPanel,
    MovementControls,
    HealthCheckDisplay,
    LaserPowerController,
    PidTuningPanel,
    TargetSelector
)
from main.config import VIEW_MODES

class UIManager:
    def __init__(self, main_window):
        self.main_window = main_window
        self._init_components()
        self._setup_layout()

    def _init_components(self):
        """Initializes all UI components."""
        self.no_fire_zone = NoFireZoneWidget()
        self.system_modes_panel = SystemModesPanel()
        self.target_selector = TargetSelector()
        self.system_control_buttons = SystemControlButtons()
        self.image_view = ImageView()
        self.movement_controls = MovementControls()
        self.health_check_display = HealthCheckDisplay()
        self.laser_power_controller = LaserPowerController()
        self.pid_tuning_panel = PidTuningPanel()
        self.return_home_button = QPushButton("Return Home")


    def _setup_layout(self):
        """Sets up the main layout of the application."""
        central_widget = QWidget()
        self.main_window.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        left_layout = self._create_left_layout()
        center_layout = self._create_center_layout()
        right_layout = self._create_right_layout()

        main_layout.addLayout(left_layout, stretch=1)
        main_layout.addLayout(center_layout, stretch=8)
        main_layout.addLayout(right_layout, stretch=1)

    def _create_left_layout(self):
        """Creates the left panel layout."""
        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.addWidget(self.return_home_button)
        layout.addWidget(self.no_fire_zone)
        layout.addWidget(self.target_selector)
        layout.addWidget(self.system_modes_panel)
        layout.addStretch(1)
        return layout

    def _create_center_layout(self):
        """Creates the center panel layout."""
        layout = QVBoxLayout()
        layout.addWidget(self.system_control_buttons, stretch=1)

        layout.addWidget(self.image_view, stretch=10)
        layout.addWidget(self.movement_controls, stretch=5)
        return layout

    def _create_right_layout(self):
        """Creates the right panel layout."""
        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.addWidget(self.health_check_display)
        layout.addWidget(self.laser_power_controller)

        layout.addWidget(self.pid_tuning_panel)
        layout.addStretch(1)
        return layout

    def connect_signals(self, handler):
        """Connects component signals to the main window's handlers."""
        self.system_control_buttons.system_start_requested.connect(handler._start_system)
        self.system_control_buttons.system_stop_requested.connect(handler._stop_system)
        self.return_home_button.clicked.connect(handler._handle_return_home)
        self.no_fire_zone.zoneDefined.connect(handler._handle_no_fire_zone)
        self.no_fire_zone.zoneCleared.connect(handler._handle_clear_no_fire_zone)
        self.system_modes_panel.mode_changed_signal.connect(handler._handle_mode_change)
        self.target_selector.target_selected.connect(handler._handle_target_selection)
        self.target_selector.target_cleared.connect(handler._handle_target_cleared)
        self.movement_controls.movement_command_signal.connect(handler._handle_movement_command)
        self.movement_controls.shoot_command_signal.connect(handler._handle_shoot_command)
        self.laser_power_controller.target_power_changed.connect(handler._handle_laser_target_power_change)
        self.pid_tuning_panel.pid_settings_applied_signal.connect(handler._handle_pid_settings_applied)
