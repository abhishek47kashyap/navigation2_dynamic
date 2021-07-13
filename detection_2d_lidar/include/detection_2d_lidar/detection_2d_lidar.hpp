
#ifndef DETECTION_2D_LIDAR_DATA_STRUCTURES_H
#define DETECTION_2D_LIDAR_DATA_STRUCTURES_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

float distance_from_origin(const geometry_msgs::msg::Point& _point);
float distance_between_points(const geometry_msgs::msg::Point& _point1, const geometry_msgs::msg::Point& _point2);
float distance_point_from_line(const geometry_msgs::msg::Point& _endPoint1, const geometry_msgs::msg::Point& _endPoint2, const geometry_msgs::msg::Point& _anyPoint);

template <typename T>
std::vector<std::pair<T, T>> pairwise_combination(const std::vector<T>& dataset)
{
    std::vector<std::pair<T, T>> combs;
    for (long unsigned int i = 0; i <= dataset.size() - 2; ++i) 
    {
        for (long unsigned int j = i+1; j <= dataset.size() - 1; ++j)
            combs.push_back(std::pair<T, T>(dataset[i], dataset[j]));
    }
    return combs;
}

bool same_3d_point(const geometry_msgs::msg::Point& ptA, const geometry_msgs::msg::Point& ptB);


struct line {
  float slope, intercept;
  geometry_msgs::msg::Point endpoints[2];

  float length()
  {
      return distance_between_points(endpoints[0], endpoints[1]);
  }
};


struct circle {
    float radius;
    geometry_msgs::msg::Point center;
};


class Group{
    void _circle_fitting_barycenter_method();
    void _circle_fitting_equilateral_triangle_method();
    void _circle_fitting_least_squares_method();

    void _get_group_center_of_mass(float *mean_x, float *mean_y);

public:
    line best_fit_line;
    circle best_fit_circle;

    std::vector<geometry_msgs::msg::Point> points;
    std::vector<geometry_msgs::msg::Point>::size_type num_points();
    void add_point(const geometry_msgs::msg::Point& _point);
    void add_points(const std::vector<geometry_msgs::msg::Point>& _points);
    void overwrite(const std::vector<geometry_msgs::msg::Point>& _points);
    void clear();
    void calculate_best_fit_line();

    enum class CircleFittingMethods{BARYCENTER_METHOD, EQUILATERAL_TRIANGLE_METHOD, LEAST_SQUARES_METHOD};
    void calculate_best_fit_circle(CircleFittingMethods method = CircleFittingMethods::LEAST_SQUARES_METHOD);
};


#endif //DETECTION_2D_LIDAR_DATA_STRUCTURES_H
