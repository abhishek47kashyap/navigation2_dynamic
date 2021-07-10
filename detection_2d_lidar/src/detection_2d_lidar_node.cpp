#include "detection_2d_lidar/detection_2d_lidar_node.hpp"
#include <algorithm>

Detection2DLidarNode::Detection2DLidarNode(): Node("detection_2d_lidar")
{
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

    RCLCPP_INFO(get_logger(), "Node detection_2d_lidar ready");

    initPubSub();
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
    header = laser_scan->header;

    // convert from polar to cartesian coordinates (only for ranges lying within min and max limits)
    points.clear();
    float i = 0.0;
    for (const float &r : laser_scan->ranges)
    {
        geometry_msgs::msg::Point p;
        if (r >= laser_scan->range_min && r <= laser_scan->range_max)
        {
            p.x = r * cos(laser_scan->angle_min + (i * laser_scan->angle_increment));
            p.y = r * sin(laser_scan->angle_min + (i * laser_scan->angle_increment));
            p.z = 0.0;

            points.push_back(p);
        }
        i = i + 1.0;
    }
    points.shrink_to_fit();
    RCLCPP_INFO(get_logger(), "Converted polar to cartesian, no of points = %ld", points.size());

    // reset state and detect obstacles
    resetState();
    detectObstacles();
}

void Detection2DLidarNode::resetState()
{
    groups.clear();
    obstacle_lines.clear();
    obstacle_circles.clear();
}

void Detection2DLidarNode::detectObstacles()
{
    grouping();
}

void Detection2DLidarNode::grouping()
{
    RCLCPP_INFO(get_logger(), "Starting grouping");

    // create a copy of the points
    std::vector<geometry_msgs::msg::Point> copy_of_points(points);

    while (!copy_of_points.empty())   // as points will be assigned to a group, they will be removed
    {
        Group grp;
        geometry_msgs::msg::Point a_random_point;

        // start off a new group with a random point that is yet un-grouped
        a_random_point = copy_of_points.back();
        copy_of_points.pop_back();
        grp.add_point(a_random_point);

        // iterate over all other un-grouped points and check for proximity to a_random_point
        for (const geometry_msgs::msg::Point &_p : copy_of_points)
        {
            const float r = distance_from_origin(_p);
            const float d = distance_between_points(_p, a_random_point);

            // grouping condition check
            if (d < p_max_group_distance + r * p_distance_proportion)
            {
                grp.add_point(_p);

                // update the point with respect to which other points will be checked
                a_random_point.x = _p.x;
                a_random_point.y = _p.y;
                a_random_point.z = _p.z;
                // TODO: overload '='
            }
        }

        // at this point, all points satisfying the above condition have been added to group
        groups.push_back(grp);

        // prune copy_of_points
        auto membership_check = [grp](const geometry_msgs::msg::Point& p) -> bool { 
            return std::find(grp.points.begin(), grp.points.end(), p) != grp.points.end();   // check if 'p' is in 'grp'
            };
        copy_of_points.erase(std::remove_if(copy_of_points.begin(), copy_of_points.end(), membership_check), copy_of_points.end());
    }

    RCLCPP_INFO(get_logger(), "Grouping completed, number of groups = %ld", groups.size());
}

void Detection2DLidarNode::splitting()
{
    
}

void Detection2DLidarNode::segmentation()
{
    
}

void Detection2DLidarNode::segmentMerging()
{
    
}

void Detection2DLidarNode::circleFitting()
{
    
}

void Detection2DLidarNode::obstacleClassification()
{
    
}

void Detection2DLidarNode::removeSmallObstacles()
{
    
}

void Detection2DLidarNode::visualization()
{
    
}