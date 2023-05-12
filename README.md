# Assignment 3 - Jetbot
## Task: Line Following & Object Detection

The aim of the project is to deploy the programmes on the JetBot robot to enable it to perform patrol and object detection tasks. There are two versions of the programmes: the Python version and the ROS version. 
+ The [Python version programme](https://github.com/alstondu/jetbot-ass3/tree/main/Jetbotpy)  is designed to run in Python 3.6 environment. 
+ The [ROS version programme](https://github.com/alstondu/jetbot-ass3/tree/main/jetbot) is implemented as a series of ROS nodes that can be executed individually using `rosrun` or run simultaneously by running a launch file using `roslaunch`.

## Execution Commands
<details open>
<summary>Python Scripts</summary>

The [Line Following + TOF programme](https://github.com/alstondu/jetbot-ass3/tree/main/Jetbotpy/jetbot) to accomplish tasks about line following and obstcale detection.

```commandline
cd ~
cd Jetbotpy/jetbot/
python3.6 LineFollower+TOF.py
```

The [object detection programme](https://github.com/alstondu/jetbot-ass3/tree/main/Jetbotpy/yolov5) to accomplish task about object recognition.

```commandline
cd ~
cd Jetbotpy/yolov5/
python3.6 detect.py
```

</details>


<details open>
<summary>ROS Nodes</summary>

The ROS node initialization.

```commandline
roscore
```

The ROS node [Robot_Movement.py](https://github.com/alstondu/jetbot-ass3/blob/main/jetbot/scripts/jetbot/Robot_Movement.py) to subscrible the information from publishers and control the moter of Jetbot robot.

```commandline
rosrun jetbot Robot_Movement.py
```

The ROS node [LF_Publisher.py](https://github.com/alstondu/jetbot-ass3/blob/main/jetbot/scripts/jetbot/LF_Publisher.py) to publish the line following command.

```commandline
rosrun jetbot LF_Publisher.py
```

The ROS node [TOF_Publisher.py](https://github.com/alstondu/jetbot-ass3/blob/main/jetbot/scripts/jetbot/TOF_Publisher.py) to publish the TOF command.

```commandline
rosrun jetbot TOF_Publisher.py
```

The ROS node [OR_Publisher.py](https://github.com/alstondu/jetbot-ass3/blob/main/jetbot/scripts/yolov5/OR_Publisher.py) to publish the Object-Detection command.

```commandline
rosrun jetbot OR_Publisher.py
```

</details>


<details open>
<summary>ROS Nodes</summary>

ROS launch command execution

```commandline
roslaunch jetbot jetbot.launch
```

</details>
