#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <string>

namespace bacc = boost::accumulators;

namespace xv_11_lidar_raspberry {
    
    class XV11Lidar {
        public:
            XV11Lidar(const std::string& port, 
                      uint32_t baud_rate, 
                      const std::string& frame_id, 
                      int initial_pwm, 
                      ros::NodeHandle& n);
            ~XV11Lidar();

            void start();
            void stop();
            void sync_frame(uint8_t step);
            void read_frame();
            void read_frame_handler(const boost::system::error_code& error, size_t bytes_transferred);
            void sync_frame_handler(const boost::system::error_code& error, size_t bytes_transferred);

        private:

            uint16_t rpms_;
            // RPMS (rolling mean)
            typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
            typedef bacc::tag::rolling_window RollingWindow;
            RollingMeanAcc rpms_acc_;

            int initial_pwm_;
            int pwm_;

            sensor_msgs::LaserScan::Ptr scan_;
            ros::Publisher scan_pub_;

            boost::asio::io_service io_;
            boost::asio::serial_port serial_;
            boost::array<uint8_t, 1980> raw_bytes_;

            bool shutting_down_;
    };
}
