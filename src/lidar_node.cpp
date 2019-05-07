#include "xv_11_lidar_raspberry/lidar.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <boost/asio.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>

xv_11_lidar_raspberry::XV11Lidar *lidar = NULL;

void active_callback(const std_msgs::BoolConstPtr& msg) 
{
  if(msg->data && lidar != NULL)
  {
    ROS_INFO("Start LIDAR");
    lidar->start();
  }
  else if(lidar != NULL)
  {
    ROS_INFO("Stop LIDAR");
    lidar->stop();
  }
}

int main (int argc, char **argv)
{
  // Init ROS
  ros::init(argc, argv, "lidar");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  // Read parameters
  std::string port;
  int baud_rate;
  std::string frame_id;
  int initial_pwm;
  priv_nh.param("port", port, std::string("/dev/ttyAMA0"));
  priv_nh.param("baud_rate", baud_rate, 115200);
  priv_nh.param("frame_id", frame_id, std::string("neato_laser"));
  priv_nh.param("initial_pwm", initial_pwm, 400);

  // Start LIDAR
  lidar = new xv_11_lidar_raspberry::XV11Lidar(port, baud_rate, frame_id, initial_pwm, n);
  ros::Subscriber laser_active_sub = n.subscribe("lidar_active", 1, active_callback);

  // Spin
  ros::spin();

  delete lidar;
}

