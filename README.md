# Neato XV 11 LIDAR for ROS on Raspberry Pi 3

This code is only compatible with "Firmware version 2" XV 11 Lidar.

## Electronics

The LIDAR is connected to the serial port of the raspberry (3,3V / 8N1 / 115220).

A PWM is generated on pin 12 (GPIO18) and the program use a simple proportionnal controller to regulate the rotational velocity at 300 RPM.

<img with="100%" src="https://raw.githubusercontent.com/julienbayle/xv_11_lidar_raspberry/master/lidar.png" />

## Install

LIDAR is connected to the Rasberry using the miniuart interface. To activate *miniuart*,
edit "/boot/firmware/config.txt" and add or update the following lines :

```
enable_uart=1
core_freq = 250
dtoverlay=pi3-miniuart-bt
```

Then edit "/boot/firmware/cmdline.txt" and remove the following part :
```
console=*,115200
```

Then allow the user to use the uart interface :
```bash
USER=`whoami`
sudo usermod -a -G dialout $USER
sudo systemctl disable hciuart
sudo reboot
```

Lidar motor is controlled via a PWM on GPIO18 (pin number 12). We need to allow the user to control this pin. For that, a service script will be used that will also turn lidar off at startup :

```bash
sudo cp xv_11_lidar_raspberry/cfg/lidar /etc/init.d/
sudo chmod +x /etc/init.d/lidar
sudo update-rc.d lidar defaults
```

Do reboot to test :
```bash
sudo reboot
```

If everything is OK, lidar could be turned on and off using the following command lines :
```bash
lidar start
lidar stop
```

When started, lidar data should be received on /dev/ttyAMA0. To check that, screen is used :

```bash
screen /dev/ttyAMA0 115200
```
To exit GNU screen, type Control-A k. 

OK, everything is fine, let's use ROS.

## Basic ROS usage

Start the ROS node :

```bash
roscore &
rosrun xv_11_lidar_raspberry xv_11_lidar_raspberry &
```

Start lidar using ROS :
```bash
rostopic pub /lidar_active std_msgs/Bool "data: true"
```

Stop lidar using ROS :
```bash
rostopic pub /lidar_active std_msgs/Bool "data: true"
```

Show lidar data :
```bash
rostopic echo /scan
rosrun rviz rviz
```

In RVIZ, load cfg/demo.rviz