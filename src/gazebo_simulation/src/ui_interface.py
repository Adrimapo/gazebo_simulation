#!/usr/bin/env python
import rospy

import sys
import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates

from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QGraphicsScene, QMainWindow, QVBoxLayout
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from main_window_ui import Ui_MainWindow



class PlotMaker(QMainWindow):
    def __init__(self, win):
        super(PlotMaker, self).__init__(win)

        self.win = win

        # Create a FigureCanvas for matplotlib
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        win.ui.graph.setLayout(layout)

        # Command and real vel subscribers
        self.commandVel = 0
        self.controllVel = 0
        self.pose = 0

        command_sub = rospy.Subscriber('/robot/left_rear_wheel_velocity_controller/command', Float64, self.wheelCommand)
        joint_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.controlCallback1)

        # Timer for saving the plot
        self.data = []
        self.initTime = rospy.Time.now()
        self.timer = QTimer()
        self.timer.timeout.connect(self.updatePlot)
        self.timer.start(100)

        win.ui.velButton.clicked.connect(self.livingPlot)

    def updatePlot(self):
        current_time = rospy.Time.now()
        elapsed_time = round((current_time - self.initTime).to_sec(), 2)
        self.data.append((elapsed_time, self.pose, self.controllVel, self.commandVel))
        np.savetxt('data.csv', self.data, delimiter=',')

    def livingPlot(self):
        # Creates the plot
        ax = self.figure.add_subplot(111)
        ax.clear()

        data_array = np.array(self.data)
        time = data_array[:, 0]
        realVel = data_array[:, 2]
        commandVel = data_array[:, 3]

        # Filter data for the last 20 seconds
        last_20s_indices = np.where(time >= (time[-1] - 20))
        time_last_20s = time[last_20s_indices]
        realVel_last_20s = realVel[last_20s_indices]
        commandVel_last_20s = commandVel[last_20s_indices]

        ax.plot(time_last_20s, realVel_last_20s)
        ax.plot(time_last_20s, commandVel_last_20s)

        GRAPH_LIMIT = 11
        ax.set_ylim(-GRAPH_LIMIT, GRAPH_LIMIT)

        # Adjust x-axis range to show the last 20 seconds
        ax.set_xlim(time[-1] - 20, time[-1])

        # Update the canvas
        self.canvas.draw()


    def wheelCommand(self, msg):
        self.commandVel = msg.data / 2

    def controlCallback1(self, msg):
        wheel_index = 3 # base_link index
        self.pose = msg.pose[wheel_index].position.x
        self.controllVel = msg.twist[wheel_index].linear.x


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Image subscribers
        self.chasisSub = ImageSubscriber("/robot/chasis/image_raw")
        self.effectorSub = ImageSubscriber("/robot/end_effector/image_raw")

        # Connect the image_callback
        self.chasisSub.image_callback.connect(self.update_chasis_image)
        self.effectorSub.image_callback.connect(self.update_effector_image)

        # Updates the filtered image
        self.effectorPixmap = QPixmap()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_filter_image)
        self.timer.start(100) 

        # Alert configuration
        self.ui.alert.setStyleSheet("QLabel { background-color: green;}")
        self.ui.alert.hide()
        

    def update_chasis_image(self, qpixmap):
        resized_pixmap = qpixmap.scaledToWidth(self.ui.chasisImage.width())
        scene = QGraphicsScene()
        scene.addPixmap(resized_pixmap)
        self.ui.chasisImage.setScene(scene)

    def update_effector_image(self, qpixmap):
        self.effectorPixmap = qpixmap.scaledToWidth(self.ui.armImage.width())
        scene = QGraphicsScene()
        scene.addPixmap(self.effectorPixmap)
        self.ui.armImage.setScene(scene)

    def update_filter_image(self):
        if (self.effectorPixmap is not None):
            qpixmap = self.effectorPixmap
            resized_pixmap = qpixmap.scaledToWidth(self.ui.filteredArm.width())

            image = resized_pixmap.toImage()

            if image.isNull():
                return

            # Convert QImage to numpy array
            width = image.width()
            height = image.height()
            buffer = image.bits().asstring(width * height * 4)  # 4 channels (RGBA)
            img_array = np.frombuffer(buffer, dtype=np.uint8).reshape((height, width, 4))

            bgr_image = cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
            hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

            lower_green = np.array([40, 40, 40])
            upper_green = np.array([70, 255, 255])

            # Create a mask using the lower and upper
            mask = cv2.inRange(hsv_image, lower_green, upper_green)
            green_filtered_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

            filtered_bgr_image = cv2.cvtColor(green_filtered_image, cv2.COLOR_HSV2BGR)

            # Convert the filtered BGR image to QImage
            height, width, _ = filtered_bgr_image.shape
            bytes_per_line = width * 3  # 3 channels (BGR)
            qimage = QtGui.QImage(filtered_bgr_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)

            filtered_qpixmap = QtGui.QPixmap.fromImage(qimage)

            # Set the filtered image to QGraphicsScene
            scene = QtWidgets.QGraphicsScene()
            scene.addPixmap(filtered_qpixmap)
            self.ui.filteredArm.setScene(scene)

            # Shows alert if a green object was detected
            if np.any(mask):    
                self.showAlert()
            else:
                self.hideAlert()
    
    def showAlert(self):
        self.ui.alert.show()

    def hideAlert(self):
        self.ui.alert.hide()

    def closeEvent(self, event):
        self.chasisSub.unsubscribe()
        self.effectorSub.unsubscribe()
        super(MainWindow, self).closeEvent(event)


class ImageSubscriber(QtCore.QObject):
    image_callback = QtCore.pyqtSignal(QtGui.QPixmap)

    def __init__(self, topic):
        super(ImageSubscriber, self).__init__()

        self.image_sub = rospy.Subscriber(topic, Image, self.handle_image)

    def handle_image(self, msg):
        # Convert ROS image message to QPixmap
        qimage = QtGui.QImage(msg.data, msg.width, msg.height, QtGui.QImage.Format_RGB888)
        qpixmap = QtGui.QPixmap.fromImage(qimage)

        # Emit the signal with the updated QPixmap
        self.image_callback.emit(qpixmap)

    def unsubscribe(self):
        self.image_sub.unregister()


# Shows the trace bar value on the lcd
class traceBarLcd(QtWidgets.QWidget):
    def __init__(self, slider, lcd):
        super(traceBarLcd, self).__init__()

        # Slider
        slider.setMinimum(0)
        slider.setMaximum(10)
        slider.setTickInterval(1)
        slider.setSingleStep(1)
        slider.setValue(5)
        lcd.display(5)
        slider.valueChanged.connect(self.on_slider_value_changed)
        self.vel = 0

        #Lcd
        self.lcd = lcd
    
    def on_slider_value_changed(self, value):
        self.vel = value
        self.lcd.display(value)
    
    def getVel(self):
        return self.vel

# Press button to publish
class ButtonPublisher(QtWidgets.QWidget):
    def __init__(self, topic, btn, slider, kp):
        super(ButtonPublisher, self).__init__()

        # Button publisher
        self.publisher = rospy.Publisher(topic, Float64, queue_size=10)
        slider.valueChanged.connect(self.on_slider_value_changed)
        self.kp = kp
        self.vel = 5

        # Button
        self.button = btn
        self.button.pressed.connect(self.button_pressed)
        self.button.released.connect(self.button_released)

    def on_slider_value_changed(self, value):
        self.vel = value

    def button_pressed(self):
        msg = Float64()
        msg.data = self.vel * self.kp
        self.publisher.publish(msg)

    def button_released (self):
        msg = Float64()
        msg.data = 0
        self.publisher.publish(msg)

# Publish with one click
class pressedButton(QtWidgets.QWidget):
    def __init__(self, topic, btn, slider, kp):
        super(pressedButton, self).__init__()

        # Button publisher
        self.publisher = rospy.Publisher(topic, Float64, queue_size=10)
        slider.valueChanged.connect(self.on_slider_value_changed)
        self.kp = kp
        self.vel = 5 

        # Button
        self.button = btn
        self.button.clicked.connect(self.button_clicked)

    def on_slider_value_changed(self, value):
        self.vel = value

    def button_clicked(self):
        msg = Float64()
        msg.data = self.vel * self.kp
        self.publisher.publish(msg)


def configureDirection(button, slider, vels):

    p1 = ButtonPublisher('/robot/left_rear_wheel_velocity_controller/command', 
                                button, slider, vels[0])

    p2 = ButtonPublisher('/robot/left_upper_wheel_velocity_controller/command', 
                                 button, slider, vels[1])

    p3 = ButtonPublisher('/robot/right_rear_wheel_velocity_controller/command', 
                                 button, slider, vels[2])

    p4 = ButtonPublisher('/robot/right_upper_wheel_velocity_controller/command', 
                                 button, slider, vels[3])

    return p1, p2, p3, p4


if __name__ == "__main__":

    rospy.init_node("control_interface")
    app = QtWidgets.QApplication(sys.argv)

    # Loads the main window with the images
    main_window = MainWindow()

    # Plot maker to save the plot and show it clicking a buttpn
    plot = PlotMaker(main_window)

    # Arm configuration
    armslider = traceBarLcd(main_window.ui.armSlide, main_window.ui.armNum)

    ## Arm 1
    arm1 = ButtonPublisher('/robot/arm1_joint_controller/command', 
                            main_window.ui.armUpBtn, main_window.ui.armSlide, -2)

    arm2 = ButtonPublisher('/robot/arm1_joint_controller/command', 
                            main_window.ui.armDownBtn, main_window.ui.armSlide, 2)

    ## Arm 2
    arm3 = ButtonPublisher('/robot/arm2_joint_controller/command', 
                            main_window.ui.arm2UpBtn, main_window.ui.armSlide, -2)
    
    arm4 = ButtonPublisher('/robot/arm2_joint_controller/command', 
                            main_window.ui.arm2DownBtn, main_window.ui.armSlide, 2)

    # Gripper
    grip1 = pressedButton('/robot/right_gripper_controller/command', 
                            main_window.ui.openBtn, main_window.ui.armSlide, 2)
    
    grip2 = pressedButton('/robot/left_gripper_controller/command', 
                            main_window.ui.openBtn, main_window.ui.armSlide, -2)
    
    grip3 = pressedButton('/robot/right_gripper_controller/command', 
                            main_window.ui.closeBtn, main_window.ui.armSlide, -2)
    
    grip4 = pressedButton('/robot/left_gripper_controller/command', 
                            main_window.ui.closeBtn, main_window.ui.armSlide, 2)   
    
    # Robot movement configuration
    robotSlider = traceBarLcd(main_window.ui.robotSlide, main_window.ui.velNum)
   
    p1, p2, p3, p4 = configureDirection(main_window.ui.forwBtn, main_window.ui.robotSlide, [2, 2, 2, 2])
    p5, p6, p7, p8 = configureDirection(main_window.ui.downBtn, main_window.ui.robotSlide, [-2, -2, -2, -2])

    p9, p10, p11, p12 = configureDirection(main_window.ui.rightBtn, main_window.ui.robotSlide, [3, 3, -3, -3])
    p13, p14, p15, p16 = configureDirection(main_window.ui.leftBtn, main_window.ui.robotSlide, [-3, -3, 3, 3])

    main_window.show()
    sys.exit(app.exec_())