# A ros node for JY901 publishing imu msg
The package is written in python. It published **imu** msgs information to **/imu_raw** revealing the acceleatation, angular velocity and angle info.
I'm using JY901 model from Wit sensor developer (http://www.wit-motion.com/index.php?m=goods&a=details&content_id=43) 
and not sure if this can also be used for others.  
<img src="https://github.com/maggielovedd/jy901-imu-ros/blob/master/demo_photo/JY901_wit_sensor.png" width="300" alt="">  

Most part of the code is provided by Wit for obtaining raw data and I simply packed the info them in imu msgs.
A world frame (position, quaternion) = (000,0001) will be published when running the launch file so that you can test JY901 directly and independtly.
I hope this can help people to visualiza and use the imu of JY901 directly in ROS.
I also add some possible errors during execution, pls check troubleshooting.

## Command
In order to achieve the visualization, I used ros imu-tool (http://wiki.ros.org/imu_tools). Please install it if you haven't:  

```sudo apt-get install ros-<distro>-imu-tools```

To run this package:  

```roslaunch jy901_python_imu jy901_imu.launch```

The final result looks like this (yellow arrow for acceleration):  

<img src="https://github.com/maggielovedd/jy901-imu-ros/blob/master/demo_photo/jy901_imu.png" alt="">

Or if you don't need the visualizationa and just want to get the imu:  
```
roscore  
rosrun jy901_python_imu jy901_imu.py
```

It will print the measurements in terminal:  

<img src="https://github.com/maggielovedd/jy901-imu-ros/blob/master/demo_photo/terminal_output.png" alt="">

## Troubleshooting
Error | Solution
------------ | -------------
env: python\r: No such file or directory | https://stackoverflow.com/questions/19425857/env-python-r-no-such-file-or-directory
could not open port /dev/ttyUSB0 | 1.setup a file /etc/udev/rules.d/70-ttyusb.rules  2.add this KERNEL=="ttyUSB[0-9]*", MODE="0666"  3.unplug and plug usb   https://blog.yelvlab.cn/archives/285/

## Remarks
- My JY901 took the common param (e.g. 10 Hz) in the code, so you may need to adjust these your to own if the setting/product is different.
