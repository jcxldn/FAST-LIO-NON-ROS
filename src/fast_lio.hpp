#ifndef FAST_LIO_H_
#define FAST_LIO_H_

#include "laserMapping.hpp"

class FastLio
{
public:
    FastLio(/* args */);
    ~FastLio();

    void feed_imu(const ImuConstPtr &imu_data);
    void feed_lidar(const CstMsgConstPtr &lidar_data);
    void process();
    std::vector<double> get_pose();
    void write_to_file(const std::vector<double> &pose);
    void write_to_file(const double &time);

private:
    custom_messages::OdomMsgPtr odom_result;
    ofstream output_file, exec_time_file;
    std::unique_ptr<LaserMapping> laser_mapping;
};

#endif