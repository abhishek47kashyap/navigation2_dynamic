#ifndef DETECTION_2D_LIDAR_NODE_H
#define DETECTION_2D_LIDAR_NODE_H

#include "detection_2d_lidar/detection_2d_lidar.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class Detection2DLidarNode : public rclcpp::Node
{
public:
    Detection2DLidarNode();
    ~Detection2DLidarNode();

protected:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr laser_scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::ConstSharedPtr visualization_marker_pub_;

    float p_max_group_distance;
    float p_distance_proportion;
    float p_max_split_distance;
    size_t p_min_group_points;    // TODO: what should be its data type?
    float p_max_merge_separation;
    float p_max_merge_spread;
    float p_max_circle_radius;
    float p_radius_enlargement;
    float p_min_obstacle_size;

    std_msgs::msg::Header header;
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<Group> groups;
    std::vector<Group> obstacle_lines;
    std::vector<Group> obstacle_circles;

    void initPubSub();
    void callbackLidarData(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);
    void resetState();
    void detectObstacles();
    void grouping();
    void splitting();
    void lineFitting();
    void segmentMerging();
    void circleFitting();
    void obstacleClassification();
    void removeSmallObstacles();
    void visualization();
};

#endif //DETECTION_2D_LIDAR_NODE_H