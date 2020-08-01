# A ros node for JY901 publishing imu msg
The package is written in python. It published **imu** msgs information to **/imu_raw** by packing the acceleatation, angular velocity and angle data.
I'm using JY901 model from Wit sensor developer (http://www.wit-motion.com/index.php?m=goods&a=details&content_id=43) 
and not sure if this can also be used for other companies.  
<img src="https://github.com/maggielovedd/jy901-imu-ros/blob/master/demo_photo/JY901_wit_sensor.png" width="300" alt="">  

Most part of the code is provided by Wit for obtaining raw data and I simply packed the info them in imu msgs.
I hope this can help people to visualiza and use the imu of JY901 directly.
I also add some possible errors during execution, pls check troubleshooting below.

## Note
In order to achieve the visualization, I used ros imu-tool (http://wiki.ros.org/imu_tools). Please install it if you haven't, just type:   
```sudo apt-get install ros-<distro>-imu-tools```

## Command
To run this:  

```roslaunch jy901_python_imu jy901_imu.launch```

The final result looks like this (arrow for acceleration):  

<img src="https://github.com/maggielovedd/jy901-imu-ros/blob/master/demo_photo/jy901_imu.png" alt="">

Or if you don't need the visualizationa and just want to get the imu:  
```
roscore  
rosrun jy901_python_imu jy901_imu.py
```

It will print the measurements in terminal:

## Troubleshooting
Error | Solution
------------ | -------------
env: python\r: No such file or directory | https://stackoverflow.com/questions/19425857/env-python-r-no-such-file-or-directory
could not open port /dev/ttyUSB0 | https://blog.yelvlab.cn/archives/285/umn

## Remards
The visualization part can be used, but the axis orientation may be incorrect. I will debug it later.
My JY901 took the common param (e.g. 10 Hz) in the code, so you may need to adjust these your to own if the setting/product is different.
