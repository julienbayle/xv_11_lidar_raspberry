#include <xv_11_lidar_raspberry/lidar.h>
#include "xv_11_lidar_raspberry/motor.h"
#include <time.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace xv_11_lidar_raspberry {
  XV11Lidar::XV11Lidar(const std::string& port, 
                      uint32_t baud_rate, 
                      const std::string& frame_id, 
                      int initial_pwm, 
                      ros::NodeHandle& n): 
		shutting_down_(false), 
		pwm_(initial_pwm),
		initial_pwm_(initial_pwm),
		io_(),
		rpms_(0),
		scan_(new sensor_msgs::LaserScan),
		serial_(io_, port) 
	{
    xv_11_lidar_raspberry::startPwm(pwm_);
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

		scan_pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 5);
    scan_->header.frame_id = frame_id;
		scan_->angle_min = 0.0;
		scan_->angle_max = 2.0*M_PI;
		scan_->angle_increment = (2.0*M_PI/360.0);
		scan_->range_min = 0.06;
		scan_->range_max = 5.0;
		scan_->ranges.resize(360);
		scan_->intensities.resize(360);
		scan_->time_increment = 0.2;

		sync_frame(0);
		boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));
		
  }

	XV11Lidar::~XV11Lidar()
	{ 
		// Release serial port
		shutting_down_ = true; 

		// Wait 200 ms
		struct timespec req = {0};
		req.tv_sec = 0;
		req.tv_nsec = 200 * 1000000L;
		nanosleep(&req, (struct timespec *)NULL);
		io_.stop();

		// Stop lidar
		xv_11_lidar_raspberry::stopPwm();
	}

	void XV11Lidar::start()
	{
		xv_11_lidar_raspberry::startPwm(initial_pwm_);
	}

	void XV11Lidar::stop()
	{
		xv_11_lidar_raspberry::stopPwm();
	}


	void XV11Lidar::sync_frame(uint8_t step)
	{
		// Reset buffer
		if(step == 0)
			raw_bytes_[0] = raw_bytes_[1] = 0x00;
		
		if (!shutting_down_)
		{
				boost::asio::async_read(serial_, boost::asio::buffer(&raw_bytes_[step],1),
	  		boost::bind(&XV11Lidar::sync_frame_handler, this,
					boost::asio::placeholders::error, 
					boost::asio::placeholders::bytes_transferred) );
		}
	}

	void XV11Lidar::sync_frame_handler(const boost::system::error_code& error, size_t bytes_transferred)
	{
		if(error != 0 or bytes_transferred!= 1)
		{
			ROS_ERROR("Lidar communication failure");
		}
		else
		{
			// First frame value : 0xFA
			// Second frame value : 0xA0
			if (raw_bytes_[1] == 0xA0)
				read_frame();
			else if (raw_bytes_[0] == 0xFA && raw_bytes_[1] == 0x00)
				sync_frame(1);
			else
				sync_frame(0);
		}
	}

	void XV11Lidar::read_frame()
	{
		boost::asio::async_read(serial_, boost::asio::buffer(&raw_bytes_[2],1978),
	  		boost::bind(&XV11Lidar::read_frame_handler, this,
					boost::asio::placeholders::error, 
					boost::asio::placeholders::bytes_transferred) );
	}

	void XV11Lidar::read_frame_handler(const boost::system::error_code& error, size_t bytes_transferred)
	{
		//read data in sets of 4
		scan_->header.stamp = ros::Time::now();		
		for(uint16_t i = 0; i < raw_bytes_.size(); i=i+22) {
			if(raw_bytes_[i] == 0xFA && raw_bytes_[i+1] == (0xA0+i/22)) {	// CRC check
				rpms_ = (raw_bytes_[i+3] << 8 | raw_bytes_[i+2])/64; 
		
				for(uint16_t j = i+4; j < i+20; j=j+4) {
					int index = (4*i)/22 + (j-4-i)/4;
					scan_->ranges[index] = (((raw_bytes_[j+1] & 0x3F)<< 8) + raw_bytes_[j]) / 1000.0;
					scan_->intensities[index] = (raw_bytes_[j+3] << 8) + raw_bytes_[j+2];
				}
			}
			else
			{
				// CRC Failed, drop this scan
				ROS_INFO_STREAM("LIDAR frame dropped");
				i = raw_bytes_.size();
			}
		}

		// Update PWM
		if (rpms_ > 200 && rpms_ < 400)
		{
    	pwm_ +=  (300 - rpms_)/1;
			//ROS_INFO_STREAM("Velocity:" << rpms_ << "PWM: " << pwm_);
			xv_11_lidar_raspberry::setPwm(pwm_);
		}

		// Publish
		scan_pub_.publish(scan_);

		// Resets and wait for next frame
		sync_frame(0);
	}

};
