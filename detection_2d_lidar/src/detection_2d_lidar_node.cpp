#include "detection_2d_lidar/detection_2d_lidar_node.hpp"

Detection2DLidarNode::Detection2DLidarNode(): Node("detection_2d_lidar")
{
    RCLCPP_INFO(get_logger(), "Creating");

    this->declare_parameter("p_max_group_distance", 0.1);
    this->declare_parameter("p_distance_proportion", 0.00628);
    this->declare_parameter("p_max_split_distance", 0.2);
    this->declare_parameter("p_min_group_points", 5);
    this->declare_parameter("p_max_merge_separation", 0.02);
    this->declare_parameter("p_max_merge_spread", 0.01);
    this->declare_parameter("p_max_circle_radius", 0.7);
    this->declare_parameter("p_radius_enlargement", 0.25);
    this->declare_parameter("p_min_obstacle_size", 0.01);

    this->get_parameter("p_max_group_distance", p_max_group_distance);
    this->get_parameter("p_distance_proportion", p_distance_proportion);
    this->get_parameter("p_max_split_distance", p_max_split_distance);
    this->get_parameter("p_min_group_points", p_min_group_points);
    this->get_parameter("p_max_merge_separation", p_max_merge_separation);
    this->get_parameter("p_max_merge_spread", p_max_merge_spread);
    this->get_parameter("p_max_circle_radius", p_max_circle_radius);
    this->get_parameter("p_radius_enlargement", p_radius_enlargement);
    this->get_parameter("p_min_obstacle_size", p_min_obstacle_size);
}

Detection2DLidarNode::~Detection2DLidarNode()
{

}

void Detection2DLidarNode::initPubSub()
{
    const size_t queue_size = 10;

    // Subscriber to laser scan
    rclcpp::SensorDataQoS qos;
    qos.keep_last(queue_size);
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", qos, std::bind(&Detection2DLidarNode::callbackLidarData, this, std::placeholders::_1));

    // Publisher for Rviz visualization
    visualization_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

    // TODO: publisher for obstacles
}

void Detection2DLidarNode::callbackLidarData(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
    this->header = laser_scan->header;

    // convert from polar to cartesian coordinates (only for ranges lying within min and max limits)
    this->points.clear();
    float i = 0.0;
    for (const float &r : laser_scan->ranges)
    {
        geometry_msgs::msg::Point p;
        if (r >= laser_scan->range_min && r <= laser_scan->range_max)
        {
            p.x = r * cos(laser_scan->angle_min + (i * laser_scan->angle_increment));
            p.y = r * sin(laser_scan->angle_min + (i * laser_scan->angle_increment));
            p.z = 0.0;
        }
        i = i + 1.0;
    }
    this->points.shrink_to_fit();

    // reset state and detect obstacles
    this->resetState();
    this->detectObstacles();
}

void Detection2DLidarNode::resetState()
{
    this->groups.clear();
    this->obstacle_lines.clear();
    this->obstacle_circles.clear();
}

void Detection2DLidarNode::detectObstacles()
{

}