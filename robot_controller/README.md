# Overview

This ROS metapackage is for controlling Saturnbot. It includes an Arduino library (called ROSArduinoBridge) and a collection of ROS packages for controlling an Arduino-based robot using standard ROS messages and services.  The stack does **not** depend on ROS Serial. The stack includes a base controller for a differential drive robot that accepts ROS Twist messages and publishes odometry data back to the PC. The base controller requires the use of a motor controller and encoders for reading odometry data.

**Note**:It is intended for ROS Indigo and above, and uses the Catkin buildsystem. It may also be compatible with ROS Hydro.

# System Requirements

**ROS Dependencies**

    $ sudo apt-get install ros-indigo-diagnostic-updater ros-indigo-control-msgs ros-indigo-nav-msgs

**Python Serial:** To install the python-serial package under Ubuntu, use the command:

    $ sudo apt-get install python-serial

On non-Ubuntu systems, use either:

    $ sudo pip install --upgrade pyserial

or

    $ sudo easy_install -U pyserial

# Preparing your Serial Port under Linux

Your Arduino will likely connect to your Linux computer as port /dev/ttyACM# or /dev/ttyUSB# where # is a number like 0, 1, 2, etc., depending on how many other devices are connected.  The easiest way to make the determination is to unplug all other USB devices, plug in your Arduino, then run the command:

    $ ls /dev/ttyACM*

or

    $ ls /dev/ttyUSB*

Hopefully, one of these two commands will return the result you're looking for (e.g. /dev/ttyACM0) and the other will return the error "No such file or directory".

Next you need to make sure you have read/write access to the port.  Assuming your Arduino is connected on /dev/ttyACM0, run the command:

    $ ls -l /dev/ttyACM0

and you should see an output similar to the following:

    crw-rw---- 1 root dialout 166, 0 2013-02-24 08:31 /dev/ttyACM0

Note that only root and the "dialout" group have read/write access.  Therefore, you need to be a member of the dialout group.  You only have to do this once and it should then work for all USB devices you plug in later on.

To add yourself to the dialout group, run the command:

    $ sudo usermod -a -G dialout your_user_name

where your\_user\_name is your Linux login name.  You will likely have to log out of your X-window session then log in again, or simply reboot your machine if you want to be sure.

When you log back in again, try the command:

    $ groups

and you should see a list of groups you belong to including dialout.

# Installation of the saturnbot\_controller Stack


    $ cd ~/YOUR_CATKIN_WORKSPACE/src
    $ git clone https://github.com/Saturn-robot/saturnbot_controller.git
    $ cd ~/YOUR_CATKIN_WORKSPACE
    $ catkin_make

After that, you will install it.


# Configuring the base\_controller Node

You can define your robot's dimensions, PID parameters, and sensor configuration by editing the YAML file in the directory base\_controller/config.  So first move into that directory:

    $ roscd ros_arduino_python/config

Now copy the provided config file to one you can modify:

    $ cp arduino_params.yaml my_arduino_params.yaml

Bring up your copy of the params file (my\_arduino\_params.yaml) in
your favorite text editor.  It should start off looking like this:

<pre>
port: /dev/ttyUSB0
baud: 57600
timeout: 0.1

rate: 50
sensorstate_rate: 10

use_base_controller: False
base_controller_rate: 10

# === Robot drivetrain parameters
#wheel_diameter: 0.146
#wheel_track: 0.2969
#encoder_resolution: 8384 # from Pololu for 131:1 motors
#gear_reduction: 1.0
#motors_reversed: True

# === PID parameters
#Kp: 20
#Kd: 12
#Ki: 0
#Ko: 50
#accel_limit: 1.0

# === Sensor definitions.  Examples only - edit for your robot.
#     Sensor type can be one of the follow (case sensitive!):
#	  * Ping
#	  * GP2D12
#	  * Analog
#	  * Digital
#	  * PololuMotorCurrent
#	  * PhidgetsVoltage
#	  * PhidgetsCurrent (20 Amp, DC)

sensors: {
  #motor_current_left:   {pin: 0, type: PololuMotorCurrent, rate: 5},
  #motor_current_right:  {pin: 1, type: PololuMotorCurrent, rate: 5},
  #ir_front_center:      {pin: 2, type: GP2D12, rate: 10},
  #sonar_front_center:   {pin: 5, type: Ping, rate: 10},
  onboard_led:           {pin: 13, type: Digital, rate: 5, direction: output}
}

# Joint name and configuration is an example only
joints: {
    head_pan_joint: {pin: 3, init_position: 0, init_speed: 90, neutral: 90, min_angle: -90, max_angle: 90, invert: False, continous: False},
    head_tilt_joint: {pin: 5, init_position: 0, init_speed: 90, neutral: 90, min_angle: -90, max_angle: 90, invert: False, continous: False}
}


</pre>

**NOTE**: Do not use tabs in your .yaml file or the parser will barf it back out when it tries to load it.   Always use spaces instead.  **ALSO**: When defining your sensor parameters, the last sensor in the list does **not** get a comma (,) at the end of the line but all the rest **must** have a comma.

Let's now look at each section of this file.

 _Port Settings_

The port will likely be either /dev/ttyACM0 or /dev/ttyUSB0. Set accordingly.

The MegaRobogaiaPololu Arudino sketch connects at 57600 baud by default.

_Polling Rates_

The main *rate* parameter (50 Hz by default) determines how fast the
outside ROS loop runs.  The default should suffice in most cases.  In
any event, it should be at least as fast as your fastest sensor rate
(defined below).

The *sensorstate\_rate* determines how often to publish an aggregated
list of all sensor readings.  Each sensor also publishes on its own
topic and rate.

The *use\_base\_controller* parameter is set to False by default.  Set it to True to use base control (assuming you have the required hardware.)  You will also have to set the PID paramters that follow.

The *base\_controller\_rate* determines how often to publish odometry readings.

_Defining Sensors_

The *sensors* parameter defines a dictionary of sensor names and
sensor parameters. (You can name each sensor whatever you like but
remember that the name for a sensor will also become the topic name
for that sensor.)

The four most important parameters are *pin*, *type*, *rate* and *direction*.
The *rate* defines how many times per second you want to poll that
sensor.  For example, a voltage sensor might only be polled once a
second (or even once every 2 seconds: rate=0.5), whereas a sonar
sensor might be polled at 20 times per second.  The *type* must be one
of those listed (case sensitive!).  The default *direction* is input so
to define an output pin, set the direction explicitly to output.  In
the example above, the Arduino LED (pin 13) will be turned on and off
at a rate of 2 times per second.

_Defining Servo Configurations_

The *joints* parameter defines a dictionary of joint names and servo parameters.  (You can name each joint whatever you like but rememember that joint names will become part of the servo's ROS topic and service names.)

The most important parameter is *pin* which of course must match the pin the servo attaches to on your Arduino.  Most PWM servos operate from 0 to 180 degrees with a "neutral" point of 90 degrees. ROS uses radians instead of degrees for joint positions but it is usually easier for programmers to specify the angular limits in the config file using degrees.  The ROS Arduino Bridge pacakge takes care of the conversion to radians.  An *init_position* of 0 therefore means 0 degrees relative to the neutral point of 90 degrees.  A *max_angle* of 90 degrees maps into 180 degrees at the servo.

_Setting Drivetrain and PID Parameters_

To use the base controller, you will have to uncomment and set the
robot drivetrain and PID parameters.  The sample drivetrain parameters
are for 6" drive wheels that are 11.5" apart.  Note that ROS uses
meters for distance so convert accordingly.  The sample encoder
resolution (ticks per revolution) is from the specs for the Pololu
131:1 motor.  Set the appropriate number for your motor/encoder
combination.  Set the motors_reversed to True if you find your wheels
are turning backward, otherwise set to False.

The PID parameters are trickier to set.  You can start with the sample
values but be sure to place your robot on blocks before sending it
your first Twist command.

# Launching the base\_controller Node

Take a look at the launch file arduino.launch in the
base\_controller/launch directory.  As you can see, it points to a
config file called my\_arduino\_params.yaml.  If you named your config
file something different, change the name in the launch file.

With your Arduino connected and running the saturnbot firmware,
launch the ros\_arduino\_python node with your parameters:

    $ roslaunch ros_arduino_python arduino.launch

You should see something like the following output:

<pre>
process[arduino-1]: started with pid [6098]
Connecting to Arduino on port /dev/ttyUSB0 ...
Connected at 57600
Arduino is ready.
[INFO] [WallTime: 1355498525.954491] Connected to Arduino on port /dev/ttyUSB0 at 57600 baud
[INFO] [WallTime: 1355498525.966825] motor_current_right {'rate': 5, 'type': 'PololuMotorCurrent', 'pin': 1}
[INFO]
etc
</pre>

If you have any Ping sonar sensors on your robot and you defined them
in your config file, they should start flashing to indicate you have
made the connection.

# Viewing Sensor Data

To see the aggregated sensor data, echo the sensor state topic:

    $ rostopic echo /arduino/sensor_state

To see the data on any particular sensor, echo its topic name:

    $ rostopic echo /arduino/sensor/sensor_name

For example, if you have a sensor called ir\_front\_center, you can see
its data using:

    $ rostopic echo /arduino/sensor/ir_front_center

You can also graph the range data using rxplot:

    $ rxplot -p 60 /arduino/sensor/ir_front_center/range


Sending Twist Commands and Viewing Odometry Data
------------------------------------------------

Place your robot on blocks, then try publishing a Twist command:

    $ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{ angular: {z: 0.5} }'

The wheels should turn in a direction consistent with a
counter-clockwise rotation (right wheel forward, left wheel backward).
If they turn in the opposite direction, set the motors_reversed
parameter in your config file to the opposite of its current setting,
then kill and restart the arduino.launch file.

Stop the robot with the command:

    $ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'

To view odometry data:

    $ rostopic echo /odom

or

   $ rxplot -p 60 /odom/pose/pose/position/x:y, /odom/twist/twist/linear/x, /odom/twist/twist/angular/z

# ROS Services for Sensors and Servos

The ros\_arduino\_python package also defines a few ROS services for sensors and servos as follows:

**digital\_set\_direction** - set the direction of a digital pin

    $ rosservice call /arduino/digital_set_direction pin direction

where pin is the pin number and direction is 0 for input and 1 for output.

**digital\_write** - send a LOW (0) or HIGH (1) signal to a digital pin

    $ rosservice call /arduino/digital_write pin value

where pin is the pin number and value is 0 for LOW and 1 for HIGH.

**servo\_write** - set the position of a servo

    $ rosservice call /arduino/servo_write id pos

where id is the index of the servo as defined in the Arduino sketch (servos.h) and pos is the position in radians (0 - 3.14).

**servo\_read** - read the position of a servo

    $ rosservice call /arduino/servo_read id

where id is the index of the servo as defined in the Arduino sketch (servos.h)

# ROS Joint Topics and Services

At the ROS level, a servo is called a joint and each joint has its own topics and services.  To change the position of a joint, publish the position
in radians to the topic:

**/\<joint_name\>/command**

For example, a joint called head_pan_joint in the YAML config file can be controlled using the topic:

**/head_pan_joint/command**

which takes a Float64 argument specifying the desired position in radians.  For example, the command:

    $ rostopic pub -1 /head_pan_joint/command std_msgs/Float64 -- 1.0

will move the servo to angle 1.0 radians from the neutral point; i.e. about 147 degrees when using the default neutral point of 90 degrees.  Using a negative value moves the servo in the other direction:

    $ rostopic pub -1 /head_pan_joint/command std_msgs/Float64 -- -1.0

A number of services are also available for each joint:

**/\<joint_name\>/enable** - Enable or disable a joint.  Disabling also detachs the underlying servo so that it can be moved by hand.

    $ rosservice call /head_pan_joint/enable false

**/\<joint_name\>/relax** - Another way to detach the underlying servo so that it can be moved by hand.

    $ rosservice call /head_pan_joint/relax

**/\<joint_name\>/set_speed** - Set the movement speed of servo in radians per second.

    $ rosservice call /head_pan_joint/set_speed 1.0
