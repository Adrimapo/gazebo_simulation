# P5-MundoGazebo

<div align = center>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/robot.png"
     alt="Robot"  />
</div>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Docker](#docker)
- [Phase 1](#phase-1)
- [Phase 2](#phase-2)
- [Phase 3](#phase-3)
- [Image filtering](#image-filtering)
- [Comentario sobre la practica](#comentario-sobre-la-practica)
- [References](#references)
- [License](#license)

## Docker

Install docker:
```bash

sudo apt install docker docker.io

# Create the docker group and add the user
sudo group add docker
sudo usermod -aG docker $USER

# Permisions to connect to the server
xhost +
sudo service docker restart

# Restart the computer
reboot 
```


A Docker file was created for this practice. First, the Dockerfile was created:

First we build the docker
```bash
docker build -t gazebo-ros-kinetic .
```
Then we execute it
```bash
docker run --privileged -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /PACKAGE_ROUTE:/root/catkin_ws/src/my_package gazebo-ros-kinetic
```

This way, we can modify our package from our computer (for example, using Visual Studio) and run this code in the Docker.

Additionally, the Docker was configured to compile the package and load the models.

```Dockerfile
# Creates and configure the workspace
RUN mkdir -p /root/catkin_ws/src 
COPY ./gazebo_simulation /root/catkin_ws/src/my_package
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /root/catkin_ws && catkin_make"

# Adds the gazebo models
ENV GAZEBO_MODEL_PATH=/root/catkin_ws/src/my_package/models:${GAZEBO_MODEL_PATH}

# Add the workspace to the .bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
```

-----------------------------------------------------------------------

-----------------------------------------------------------------------


## Phase 1

For this phase, since we already had the URDF files, we only had to execute:

```bash
gz sdf -p MODEL.urdf > MODEL.sdf
```


I must emphasize that this should be done in the simulator we are going to use. Initially, I did it from my local machine, and when I tried it in the Docker, it loaded another version. It is important that the SDF version is correct.

<br>
To add the models to the simulator, the Docker was utilized by using the command:

```Dockerfile
ENV GAZEBO_MODEL_PATH=/root/catkin_ws/src/my_package/models:${GAZEBO_MODEL_PATH}
```

To add the models to the simulator, we took advantage of the Docker by using the command:
```xml
<include>
    <uri>model://ramp</uri>
    <pose>10 0 0 0 0 0</pose>
</include>
```

<div align = center>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/scenario.png"
     alt="Robot"  />
</div>

-----------------------------------------------------------------------

-----------------------------------------------------------------------


## Phase 2

### Add cameras to the robot

1. First, two cube-shaped links were created to show where the camera will be placed.
2. Each link will be connected to a joint (Camera 1 with base_link and Camera 2 with base_gripper_link).
3. The white material was added to the cube.
4. Reference to the camera: This is where the camera specifications are declared, along with their respective topics.

```xml
  <gazebo reference="camera2">
    <sensor type="camera" name="camera2">
      <update_rate>30.0</update_rate>
      <camera name="end_effector">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>end_effector</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
```

-----------------------------------------------------------------------

### Moving wheels

1. We added the ROS control plugin to the URDF.
```xml
<gazebo>
  <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
    <robotNamespace>/robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
   <legacyModeNS>true</legacyModeNS>
  </plugin>
</gazebo>
```

2. We created the wheel transmission:
```xml
<transmission name="right_upper_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <actuator name="right_upper_wheel_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
  <joint name="right_upper_wheel_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
</transmission>
```
3. We created and configured the robot in the config.yaml file.
4. We loaded the robot_state_publisher in the launcher.

```xml
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
  <param name="publish_frequency" type="double" value="30.0"/>
</node>
```

5- We launched the controllers.

```xml

<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
       output="screen" ns="/robot" args="joint_state_controller wheel_velocity_controllers"/>
```

-----------------------------------------------------------------------

### GUI

The graphical user interface was created using PyQt5, and the qtdesigner tool was used to design our GUI.

1. First, we created our interface and saved it.

<div align = center>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/interface.png"
     alt="Robot"  />
</div>

2. We converted it to a .py file:
 
```bash
pyuic5 -o src/main_window_ui.py ui/untitled.ui
``` 

3. We imported the code into a separate program and assigned a publisher to each button, and assigned an image to each image frame from a camera.
4. Image subscriber:

```python
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
        # Unsubscribe from the image topic
        self.image_sub.unregister()
```

5. Button publisher:

```python
class ButtonPublisher(QtWidgets.QWidget):
    def __init__(self, topic, btn, slider, kp, init):
        super(ButtonPublisher, self).__init__()

        # Button publisher
        self.publisher = rospy.Publisher(topic, Float64, queue_size=10)
        slider.valueChanged.connect(self.on_slider_value_changed)
        self.kp = kp
        self.vel = (INIT_VEL + init)

        # Button
        self.init = init
        self.button = btn
        self.button.pressed.connect(self.button_clicked)
        self.button.released.connect(self.button_released)

    def on_slider_value_changed(self, value):
        self.vel = self.init + value

    def button_clicked(self):
        msg = Float64()
        msg.data = self.vel * self.kp
        self.publisher.publish(msg)

    def button_released (self):
        msg = Float64()
        msg.data = 0
        self.publisher.publish(msg)
```

In addition to the button's own functionality, we have a slider callback where it will update the speed of the button itself. [video](https://urjc-my.sharepoint.com/:v:/g/personal/a_madinabeitia_2020_alumnos_urjc_es/ERiKQhS7q4NCq-gn9DWSocwBuGMEmSp8Lo1L_6eug2Tl5Q?e=Uc6zbn)


-----------------------------------------------------------------------

-----------------------------------------------------------------------


## Phase 3
### Save data
A PlotMaker class was created to act independently of the rest of the node. If you want to create a node using this class that reads from topics and saves to CSV without knowing the inner workings of the program, you can use this class.

This class subscribes to the topics of the commanded velocities and the actual velocities. Their values are updated in each callback, and every 0.1 seconds, the CSV will be updated with the new measurements.


### Read data
To display the plots from the .csv files, the file was opened as follows:

```python
with open(fileName, 'r') as csvfile:
  reader = csv.reader(csvfile)
  data_array = np.array(list(reader), dtype=np.float64)
  data.append(data_array)
  legend.append(fileName.split("/")[-1].split(".")[0])
```

And subsequently, the function from the previous practice was used.
```python
draw_graphics(data)
```

### Graphic analysis
Based on the observed velocity in the graph, it was noticed that the speed was not being controlled properly.

- The PID controller in the config.yaml file was modified (only the proportional part), and the results improved significantly.
```yaml
  right_upper_wheel_velocity_controller:
    type: effort_controllers/JointVelocityController
    joint: right_upper_wheel_joint
    pid: {p: 20.0, i: 0.0, d: 0.0}
```

Graph obtained:
<div align = center>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/velGraph1.png"
     alt="Robot"  />
</div>

The desired velocity needs to be multiplied by 2 when publishing since the wheels have a diameter of 0.5. We can also observe that it takes 2 or 3 seconds to reach the desired velocity, which is desirable as it reflects real-world behavior.

### Resultado final
In this [video](https://urjc-my.sharepoint.com/:v:/g/personal/a_madinabeitia_2020_alumnos_urjc_es/EYEM9hEfffFHrIrLtSdmIOgBdinoSxjK0qS2rcsQcmVBCw?e=2kWPhh) we obtain the next graph:

<div align = center>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/graphic.png"
     alt="Robot"  />
</div>

We can see that as the robot reaches the ramp, the PID controller takes effect and tries to compensate for the incline. Other than that, the same behavior as in the previous graph occurs since short-duration pulses are applied, and the robot doesn't reach its peak acceleration. One option to avoid this would be to increase the proportional gain, but that could introduce overshoot.

-----------------------------------------------------------------------

-----------------------------------------------------------------------

## Image filtering

For image filtering, a timer was added to update the image as we were already obtaining it from the subscriber.
```python
self.timer = QtCore.QTimer()
self.timer.timeout.connect(self.update_filter_image)
self.timer.start(100) 
```
This was done because the initial implementation was done pixel by pixel, and this saved processing time. Later on, it was implemented using OpenCV, which reduced the processing load. The following steps were followed to filter the image:

1. Convert the image to the desired color space and format (HSV)
2. Apply the thresholding filtering technique.
3. The OpenCV image is converted back to a QPixmap to display it in the GUI.

<div align = center>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/filter.png"
     alt="Robot"  />
</div>

4. Additionally, a QLabel in the shape of a green square was configured to be displayed if the cube was detected in the image.

### Graph in GUI
An attempt was made to display the graph from section 2 in real-time, but it required significant processing power. Instead, a button was added so that when pressed, the graph would be displayed, showing the commanded velocities for the last 20 seconds.

### Final result
In the following image, we can see that when the cube is detected with the filter, a green square is displayed below. However, in the next image, the cube is not present, so the green square is not shown.

<div align = center>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/interface2.png" alt="img1"  width=400px/>
  <img src = "https://github.com/clases-julio/p5-mundogazebo-amadinabeitia2020/blob/main/fig/interface3.png" alt="img2"  width=400px/>
</div>

[Video filter](https://urjc-my.sharepoint.com/:v:/g/personal/a_madinabeitia_2020_alumnos_urjc_es/ETRumptmf1BIvdBijLuuwMYBhwEs284UX_e4f9HR08qg0Q?e=4Xka8W)

-----------------------------------------------------------------------

-----------------------------------------------------------------------

## Comentario sobre la práctica

Aunque al final se logró completar la práctica, sería deseable que ciertos materiales se proporcionaran completamente funcionales, como es el caso del Docker. Es cierto que a través de este proceso he aprendido mucho, pero podría considerarse injusto. Sin embargo también hay que tener en cuenta la dificil situación de la asignatura pero sería de ayuda más material.

-----------------------------------------------------------------------

-----------------------------------------------------------------------

## References
- [Docker example](https://github.com/igricart/docker.git)
- [Ros perception](https://github.com/ros-perception/image_pipeline)
- [Add cameras to urdf](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
- [Add transmissions](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo#Spawning_Controllers)
- [Gazebo error](https://answers.gazebosim.org/question/7300/errorcould-not-find-joint_state_controller/)
- [Python docker file](https://stackoverflow.com/questions/50333650/install-python-package-in-docker-file#50339110)

## License 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/> </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0

