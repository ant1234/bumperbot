#include <bumperbot_localization/kalman_filter.hpp>

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string & name) 
    : Node(name), 
      mean_(0.0), 
      variance_(1000.0),
      imu_angular_z_(0.0),
      is_first_odom(true),
      last_angular_z_(0.0),
      motion_(0.0)
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odomCallback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&KalmanFilter::imuCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("bumperbot_controller/odom_kalman", 10);


    void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry & odom)
    {
        kalman_odom_ = odom;

        if(is_first_odom_)
        {
            mean_ = odom.twist.twist.angular.z;
            last_angular_z_ = odom.tiwst.twist.angular.z;
            is_first_odom_ = false;
            return;
        }

        statePrediction();
        measurementUpdate();
    }

    void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu & imu)
    {
        imu_angular_z_ = imu.angular_velocity.z;
    }
}