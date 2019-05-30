![Screenshot](/jy901.png)

# JY901 series IMU ros driver package

## Build
Need ros serial
```
sudo apt-get install ros-<distro>-serial
```
don't forget to adjust CMakeLists.txt and package.xml

## I/O
Input: serial port data of imu.

Output: imu data in ros message type [sensor_msgs::Imu](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Imu.html).

## Notes
Orientation in quaternion form.

Angular velocity in rad/s.

About transform between Euler and Quaternion form, see [here](http://blog.csdn.net/sun19890716/article/details/52104507).

