#include <bumperbot_localization/odometry_motion_model.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>
#include <cmath>

using std::placeholders::_1;

double angle_diff(double a, double b) {
    a = atan2(sin(a), cos(a));
    b = atan2(sin(b), cos(b));
    double d1 = a - b;
    double d2 = 2 * M_PI - fabs(d1);
    if (d1 > 0){
        d2 *= -1.0;
    }
    if (fabs(d1) < fabs(d2)){
        return d1;
    } else {
        return d2;
    }
}

OdometryMotionModel::OdometryMotionModel(const std::string & name) 
    : Node(name),
    alpha1_(0.0),
    alpha2_(0.0),
    alpha3_(0.0),
    alpha4_(0.0),
    nr_samples_(300),
    last_odom_x_(0.0),
    last_odom_y_(0.0),
    last_odom_theta_(0.0),
    is_first_odom_(true)
{
    declare_parameter("alpha1", 0.1);
    declare_parameter("alpha2", 0.1);
    declare_parameter("alpha3", 0.1);
    declare_parameter("alpha4", 0.1);
    declare_parameter("nr_samples", 300);
    
    alpha1_ = get_parameter("alpha1").as_double();
    alpha2_ = get_parameter("alpha2").as_double();
    alpha3_ = get_parameter("alpha3").as_double();
    alpha4_ = get_parameter("alpha4").as_double();
    nr_samples_ = get_parameter("nr_samples").as_int();

    if(nr_samples > 0) {
        samples_.poses = std::vector<geometry_msgs::msg::Pose>(nr_samples_, geometry_msgs::msg::Pose());
    } else {
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid number of samples requested: " << nr_samples);
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom", 10, std::bind(&OdometryMotionModel::odomCallback, this, _1));
    pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("odometry_motion_model", 10);

}

void OdometryMotionModel::odomCallback(const nav_msgs::msg::Odometry & odom)
{
    tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tfs::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(is_first_odom_) {
        last_odom_x_ =  odom.pose.pose.position.x;
        last_odom_y_ =  odom.pose.pose.position.y;
        last_odom_theta_ =  yaw;
        samples_.header.frame_id = odom.header.frame_id;
        is_first_odom_ = false;
        return;
    }

    double odom_x_increment = odom.pose.pose.position.x - last_odom_x_;
    double odom_y_increment = odom.pose.pose.position.y - last_odom_y_;
    double odom_theta_increment = angle_diff(yaw, last_odom_theta_);

    double delta_rot1 = 0.0;
    if(sqrt(std::pow(odom_y_increment, 2) + std::pow(odom_x_increment, 2)) > 0.01){
        delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), yaw);
    }
    double delta_trasl = sqrt(std::pow(odom_y_increment, 2) + std::pow(odom_x_increment, 2));
    double delta_rot2 = angle_diff(odom_theta_increment, delta_rot1);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryMotionModel>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
