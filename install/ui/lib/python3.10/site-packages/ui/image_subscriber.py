import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt5.QtWidgets import QGridLayout, QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel, QSpinBox, QCheckBox, QPushButton, QSpacerItem, QSizePolicy, QStatusBar
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal, QMetaObject, QCoreApplication
from PyQt5 import QtCore

from QSwitchControl import SwitchControl

class ImageSubscriberThread(QThread):
    # Signal to send the image to the main thread
    image_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.node = Node("image_subscriber_node")
        self.subscriber = self.node.create_subscription(RosImage, "image_data", self.image_callback, 10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_signal.emit(cv_image)

    def run(self):
        rclpy.spin(self.node)  # Keep the thread running

    def stop(self):
        self.node.destroy_node()
        self.quit()

class ImageProcessingApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Processing with ROS 2")
        self.setGeometry(100, 100, 800, 600)

        # ---------------------------------
        # Central Widget Setup
        # ---------------------------------
        self.centralwidget = QWidget()
        self.centralwidget.setObjectName("centralwidget")
        self.setCentralWidget(self.centralwidget)
        # -------------------------------
        # </Central Widget Setup>
        # -------------------------------

        # ---------------------------------
        # Main Horizontal Layout Setup
        # ---------------------------------
        self.horizontalLayoutWidget = QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_2 = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(20, 20, 20, 20)  # Add margins here
        self.centralwidget.setLayout(self.horizontalLayout_2)
        # -------------------------------
        # </Main Horizontal Layout Setup>
        # -------------------------------   

        # ---------------------------------
        # Left Vertical Layout Setup
        # ---------------------------------
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")

        # Add QLabel to the left vertical layout for displaying images
        self.graphicsView = QLabel(self.horizontalLayoutWidget)
        self.graphicsView.setObjectName("graphicsView")
        self.graphicsView.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.graphicsView.setAlignment(QtCore.Qt.AlignCenter)
        self.verticalLayout.addWidget(self.graphicsView)

        # Add vertical spacer to the left vertical layout
        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.verticalLayout.addItem(self.verticalSpacer)

        # Add form layout for PID values
        self.formLayout = QFormLayout()
        self.formLayout.setObjectName("formLayout")

        # Add P value label and spin box
        self.label = QLabel(self.horizontalLayoutWidget)
        self.label.setObjectName("label")
        self.formLayout.setWidget(0, QFormLayout.LabelRole, self.label)
        self.spinBox = QSpinBox(self.horizontalLayoutWidget)
        self.spinBox.setObjectName("spinBox")
        self.spinBox.setMaximum(100)
        self.formLayout.setWidget(0, QFormLayout.FieldRole, self.spinBox)

        # Add I value label and spin box
        self.label_2 = QLabel(self.horizontalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(1, QFormLayout.LabelRole, self.label_2)
        self.spinBox_2 = QSpinBox(self.horizontalLayoutWidget)
        self.spinBox_2.setObjectName("spinBox_2")
        self.spinBox_2.setMaximum(100)
        self.formLayout.setWidget(1, QFormLayout.FieldRole, self.spinBox_2)

        # Add D value label and spin box
        self.label_3 = QLabel(self.horizontalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(2, QFormLayout.LabelRole, self.label_3)
        self.spinBox_3 = QSpinBox(self.horizontalLayoutWidget)
        self.spinBox_3.setObjectName("spinBox_3")
        self.spinBox_3.setMaximum(100)
        self.formLayout.setWidget(2, QFormLayout.FieldRole, self.spinBox_3)

        # Add form layout to the left vertical layout
        self.verticalLayout.addLayout(self.formLayout)

        # Add left vertical layout to the main horizontal layout
        self.horizontalLayout_2.addLayout(self.verticalLayout)
        # -------------------------------
        # </Left Vertical Layout Setup>
        # -------------------------------

        # ---------------------------------
        # Horizontal Spacer Setup
        # ---------------------------------
        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(self.horizontalSpacer)
        # -------------------------------
        # </Horizontal Spacer Setup>
        # -------------------------------

        # ---------------------------------
        # Right Vertical Layout Setup
        # ---------------------------------
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")

        # Add form layout for checkboxes
        self.formLayout_2 = QFormLayout()
        self.formLayout_2.setObjectName("formLayout_2")

        # Add Shooting label and checkbox
        self.label_4 = QLabel(self.horizontalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.formLayout_2.setWidget(0, QFormLayout.LabelRole, self.label_4)
        self.checkBox = SwitchControl(self.horizontalLayoutWidget, active_color="#6EC531")
        self.checkBox.setObjectName("checkBox")
        self.formLayout_2.setWidget(0, QFormLayout.FieldRole, self.checkBox)

        # Add Tracking label and checkbox
        self.label_5 = QLabel(self.horizontalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.formLayout_2.setWidget(1, QFormLayout.LabelRole, self.label_5)
        self.checkBox_2 = SwitchControl(self.horizontalLayoutWidget, active_color="#6EC531")
        self.checkBox_2.setObjectName("checkBox_2")
        self.formLayout_2.setWidget(1, QFormLayout.FieldRole, self.checkBox_2)

        # Add Detection label and checkbox
        self.label_6 = QLabel(self.horizontalLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.formLayout_2.setWidget(2, QFormLayout.LabelRole, self.label_6)
        self.checkBox_3 = SwitchControl(self.horizontalLayoutWidget, active_color="#6EC531")
        self.checkBox_3.setObjectName("checkBox_3")
        self.formLayout_2.setWidget(2, QFormLayout.FieldRole, self.checkBox_3)

        # Add Movement label and checkbox
        self.label_7 = QLabel(self.horizontalLayoutWidget)
        self.label_7.setObjectName("label_7")
        self.formLayout_2.setWidget(3, QFormLayout.LabelRole, self.label_7)
        self.checkBox_4 = SwitchControl(self.horizontalLayoutWidget, active_color="#6EC531")
        self.checkBox_4.setObjectName("checkBox_4")
        self.formLayout_2.setWidget(3, QFormLayout.FieldRole, self.checkBox_4)

        # Add form layout to the right vertical layout
        self.verticalLayout_3.addLayout(self.formLayout_2)

        # Add grid layout for push buttons
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName("gridLayout")

        # Add push buttons to the grid layout
        self.pushButton_3 = QPushButton(self.horizontalLayoutWidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 1, 2, 1, 1)

        self.pushButton = QPushButton(self.horizontalLayoutWidget)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 1, 1, 1, 1)

        self.pushButton_4 = QPushButton(self.horizontalLayoutWidget)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout.addWidget(self.pushButton_4, 1, 0, 1, 1)

        self.pushButton_2 = QPushButton(self.horizontalLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 0, 1, 1, 1)

        # Add grid layout to the right vertical layout
        self.verticalLayout_3.addLayout(self.gridLayout)

        # Add right vertical layout to the main horizontal layout
        self.horizontalLayout_2.addLayout(self.verticalLayout_3)
        # -------------------------------
        # </Right Vertical Layout Setup>
        # -------------------------------

        # ---------------------------------
        # Retranslate UI
        # ---------------------------------
        self.retranslateUi()
        # -------------------------------
        # </Retranslate UI>
        # -------------------------------

        # Start the ROS subscriber thread
        self.image_thread = ImageSubscriberThread()
        self.image_thread.image_signal.connect(self.update_image)
        self.image_thread.start()

    def retranslateUi(self):
        self.label.setText(QCoreApplication.translate("MainWindow", "P Value", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", "I Value", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", "D Value", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", "Shooting", None))
        self.checkBox.setText(QCoreApplication.translate("MainWindow", "CheckBox", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", "Tracking", None))
        self.checkBox_2.setText(QCoreApplication.translate("MainWindow", "CheckBox", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", "Detection", None))
        self.checkBox_3.setText(QCoreApplication.translate("MainWindow", "CheckBox", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", "Movement", None))
        self.checkBox_4.setText(QCoreApplication.translate("MainWindow", "CheckBox", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", "Right", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", "Down", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", "Left", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", "Up", None))

    def update_image(self, cv_image):
        # Convert the OpenCV image to QImage
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.graphicsView.setPixmap(QPixmap.fromImage(q_image))

def main(args=None):
    # Initialize the ROS 2 node
    rclpy.init()
    
    app = QApplication(sys.argv)
    window = ImageProcessingApp()
    window.show()
    
    # Start the Qt event loop
    sys.exit(app.exec_())
    
    # Shutdown ROS 2 when the application exits
    rclpy.shutdown()

if __name__ == "__main__":  
    main()
