# air_defense_project/main.py
"""
Main entry point for the Air Defense GUI application.
Initializes the QApplication and the main GUI window.
"""
import sys
import os
import rclpy
import threading
from PyQt5.QtWidgets import QApplication
# from gui.main_window import AirDefenseGUI

from main.ui.gui.main_window import AirDefenseGUI
# Common workaround for Qt platform plugin issues.
if "QT_QPA_PLATFORM_PLUGIN_PATH" not in os.environ or not os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""

def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    gui = AirDefenseGUI()
    gui.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
