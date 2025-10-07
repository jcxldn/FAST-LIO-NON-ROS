
#include "fast_lio.hpp"

FastLio::FastLio(/* args */) : odom_result(new custom_messages::Odometry), output_file("../data/output.txt"), exec_time_file("../data/time.txt")
{
    laser_mapping = std::make_unique<LaserMapping>();
}

FastLio::~FastLio()
{
    if (output_file.is_open())
        output_file.close();
    if (exec_time_file.is_open())
        exec_time_file.close();
}

void FastLio::feed_imu(const ImuConstPtr &imu_data)
{
    laser_mapping->imu_cbk(imu_data);
}

void FastLio::feed_lidar(const CstMsgConstPtr &lidar_data)
{
    laser_mapping->livox_pcl_cbk(lidar_data);
}

void FastLio::process()
{
    laser_mapping->run(odom_result);
}

std::vector<double> FastLio::get_pose()
{
    std::vector<double> odom;
    odom.push_back(odom_result->pose.pose.position.x);
    odom.push_back(odom_result->pose.pose.position.y);
    odom.push_back(odom_result->pose.pose.position.z);
    odom.push_back(odom_result->pose.pose.orientation.x);
    odom.push_back(odom_result->pose.pose.orientation.y);
    odom.push_back(odom_result->pose.pose.orientation.z);
    odom.push_back(odom_result->pose.pose.orientation.w);
    return odom;
}

void FastLio::write_to_file(const std::vector<double> &pose)
{
    output_file << pose[0] << "," << pose[1] << "," << pose[2] << "\n";
}

void FastLio::write_to_file(const double &time)
{
    exec_time_file << time << "\n";
}