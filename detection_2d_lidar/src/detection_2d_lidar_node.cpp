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

        // prune copy_of_points (TODO: can this be optimized?)
        auto membership_check = [grp](const geometry_msgs::msg::Point& p) -> bool { 
            return std::find(grp.points.begin(), grp.points.end(), p) != grp.points.end();   // check if 'p' is in 'grp'
            };
        copy_of_points.erase(std::remove_if(copy_of_points.begin(), copy_of_points.end(), membership_check), copy_of_points.end());
    }

    RCLCPP_INFO(get_logger(), "Grouping completed, number of groups = %ld", groups.size());
}

void Detection2DLidarNode::splitting()
{
    std::vector<Group> groups_after_splitting;

    for (Group& grp : groups)
    {
        // if group already has too few points no point splitting it
        if (grp.num_points() <= p_min_group_points)
            continue;
        

    } 
}

void Detection2DLidarNode::lineFitting()
{
    for (Group& grp : groups)
        grp.calculate_best_fit_line();
    
    RCLCPP_INFO(get_logger(), "Line fitting completed");
}

void Detection2DLidarNode::segmentMerging()
{
    
}

void Detection2DLidarNode::circleFitting()
{
    for (Group& grp : groups)
        grp.calculate_best_fit_circle();  // defaults to LEAST SQUARES METHOD
}

void Detection2DLidarNode::obstacleClassification()
{
    for (const Group& grp : groups)
    {
        // check whether circle (after enlargement) is to be categorized as a circle obstacle or line obstacle
        if (grp.best_fit_circle.radius + p_radius_enlargement <= p_max_circle_radius)
            obstacle_circles.push_back(grp);
        else
            obstacle_lines.push_back(grp);
    }

    RCLCPP_INFO(get_logger(), "%ld groups separated into %ld lines and %ld circles", groups.size(), obstacle_lines.size(), obstacle_circles.size());
}

void Detection2DLidarNode::removeSmallObstacles()
{
    /*
        Only keep those obstacles which satisfy these conditions:
            - line obstacles with length >= p_min_obstacle_size
            - circle obstacles with length >= p_min_obstacle_size

            If p_min_obstacle_size is 0, then this method remove_small_obstacles() should have no effect.
            If p_min_obstacle_size is +infinity, then no obstacles, however big, will be remaining.
    */

    // conditions in which to return preemptively
    if (p_min_obstacle_size <= 0.0)
        return;
    else if (p_min_obstacle_size == std::numeric_limits<float>::max())
    {
        obstacle_lines.clear();
        obstacle_circles.clear();
        return;
    }

    // before filtering (to show a message)
    const size_t num_obstacle_lines = obstacle_lines.size();
    const size_t num_obstacle_circles = obstacle_circles.size();

    // filter out lines smaller than p_min_obstacle_size
    auto line_length_check = [&](Group& _grp) -> bool {return _grp.best_fit_line.length() <= p_min_obstacle_size;};
    obstacle_lines.erase(std::remove_if(obstacle_lines.begin(), obstacle_lines.end(), line_length_check), obstacle_lines.end());
    // filter out circle with radius smaller than p_min_obstacle_size
    auto circle_radius_check = [&](Group& _grp) -> bool {return _grp.best_fit_circle.radius <= p_min_obstacle_size;};
    obstacle_circles.erase(std::remove_if(obstacle_circles.begin(), obstacle_circles.end(), circle_radius_check), obstacle_circles.end());

    // print message if any lines or circles were removed
    if ((obstacle_lines.size() < num_obstacle_lines) || (obstacle_circles.size() < num_obstacle_circles))
        RCLCPP_INFO(get_logger(), "After removing small obstacles, %ld lines and %ld circles remain", obstacle_lines.size(), obstacle_circles.size());
}

void Detection2DLidarNode::visualization()
{
    
}