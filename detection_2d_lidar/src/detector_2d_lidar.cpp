#include "detection_2d_lidar/detection_2d_lidar.hpp"
#include <cmath>
#include <limits>

std::vector<geometry_msgs::msg::Point>::size_type Group::num_points()
{
    return points.size();
}

void Group::add_points(const std::vector<geometry_msgs::msg::Point>& _points)
{
    for (geometry_msgs::msg::Point _point : _points)
        points.push_back(_point);
    
    points.shrink_to_fit();
}

void Group::overwrite(const std::vector<geometry_msgs::msg::Point>& _points)
{
    points = _points;
    points.shrink_to_fit();
}

void Group::clear()
{
    points.clear();
}

void Group::calculate_best_fit_line()
{
    /* get mean X and mean Y */
    float mean_x, mean_y;
    _get_group_center_of_mass(&mean_x, &mean_y);

    /* calculate slope and y-intercept: https://en.wikipedia.org/wiki/Simple_linear_regression#Fitting_the_regression_line */
    float numerator = 0.0, denominator = 0.0;
    for (geometry_msgs::msg::Point _point : points)
    {
        numerator += (_point.x - mean_x) * (_point.y - mean_y);
        denominator += pow((_point.x - mean_x), 2);
    }
    const float slope = numerator / denominator;
    const float intercept = mean_y - (slope * mean_x);

    /* to get endpoints, calculate closest and farthest points from origin and then project them on the best fit line */
    float min_range = std::numeric_limits<float>::max();
    float max_range = 0.0;
    geometry_msgs::msg::Point endpoint_near, endpoint_far;
    for (geometry_msgs::msg::Point _point : points)
    {
        float _dist_from_origin = distance_from_origin(_point);
        
        if (_dist_from_origin < min_range)
        {
            min_range = _dist_from_origin;
            endpoint_near.x = _point.x;
        }
        else if (_dist_from_origin > max_range)
        {
            _dist_from_origin = max_range;
            endpoint_far.x = _point.x;
        }
    }
    endpoint_near.y = slope * endpoint_near.x + intercept;
    endpoint_far.y = slope * endpoint_far.x + intercept;
    
    /* set parameters of best_fit_line */
    best_fit_line.slope = slope;
    best_fit_line.intercept = intercept;
    best_fit_line.endpoints[0] = endpoint_near;
    best_fit_line.endpoints[1] = endpoint_far;
}

void Group::calculate_best_fit_circle(CircleFittingMethods method)
{
    if (method == CircleFittingMethods::BARYCENTER_METHOD)
        _circle_fitting_barycenter_method();
    else if (method == CircleFittingMethods::EQUILATERAL_TRIANGLE_METHOD)
        _circle_fitting_equilateral_triangle_method();
    else if (method == CircleFittingMethods::LEAST_SQUARES_METHOD)
        _circle_fitting_least_squares_method();
}

void Group::_circle_fitting_barycenter_method()
{
    /* center would be the "center of mass" */
    float center_x, center_y;
    _get_group_center_of_mass(&center_x, &center_y);
    best_fit_circle.center.x = center_x;
    best_fit_circle.center.y = center_y;

    /* radius calculation */
    best_fit_circle.radius = 0.0;
    for (geometry_msgs::msg::Point _point : points)
    {
        float d = distance_between_points(_point, best_fit_circle.center);
        if (d > best_fit_circle.radius)
            best_fit_circle.radius = d;
    }
}

void Group::_circle_fitting_equilateral_triangle_method()
{
    /* Construct equilateral triangle with best fit line as base, and then get its circum-circle
        https://math.stackexchange.com/a/1484688/756875 
    */

    const float x1 = best_fit_line.endpoints[0].x;
    const float x2 = best_fit_line.endpoints[1].x;
    const float y1 = best_fit_line.endpoints[0].y;
    const float y2 = best_fit_line.endpoints[1].y;
    const float midpoint_x = (x1 + x2) / 2.0;
    const float midpoint_y = (y1 + y2) / 2.0;
    const float L = best_fit_line.length();

    geometry_msgs::msg::Point candidate_third_vertices[2];
    geometry_msgs::msg::Point third_vertex;
    candidate_third_vertices[0].x = midpoint_x + (sqrt(3) * (y1 - y2) * L);
    candidate_third_vertices[0].y = midpoint_y + (sqrt(3) * (x2 - x1) * L);
    candidate_third_vertices[1].x = midpoint_x - (sqrt(3) * (y1 - y2) * L);
    candidate_third_vertices[1].y = midpoint_y - (sqrt(3) * (x2 - x1) * L);

    /* out of the two candidates, pick the one farthest from origin (such that the triangle points away) */
    if (distance_from_origin(candidate_third_vertices[0]) > distance_from_origin(candidate_third_vertices[1]))
        third_vertex = candidate_third_vertices[0];
    else
        third_vertex = candidate_third_vertices[1];
    
    /* calculate radius and center of best fit circle */
    best_fit_circle.radius = L / sqrt(3);
    best_fit_circle.center.x = (x1 + x2 + third_vertex.x) / 3.0;
    best_fit_circle.center.x = (y1 + y2 + third_vertex.y) / 3.0;
}

void Group::_circle_fitting_least_squares_method()
{

}

void Group::_get_group_center_of_mass(float *mean_x, float *mean_y)
{
    float all_x = 0.0, all_y = 0.0;
    for (geometry_msgs::msg::Point _point : points)
    {
        all_x += _point.x;
        all_y += _point.y;
    }
    *mean_x = all_x / (float)num_points();
    *mean_y = all_y / (float)num_points();
}

float distance_from_origin(const geometry_msgs::msg::Point& _point)
{
    return sqrt(pow(_point.x, 2) + pow(_point.y, 2));
}

float distance_between_points(const geometry_msgs::msg::Point& _point1, const geometry_msgs::msg::Point& _point2)
{
    return sqrt(pow((_point1.x - _point2.x), 2) + pow((_point1.y - _point2.y), 2));
}

float distance_point_from_line(const geometry_msgs::msg::Point& _endPoint1, const geometry_msgs::msg::Point& _endPoint2, const geometry_msgs::msg::Point& _anyPoint)
{
    const float numerator = abs(((_endPoint1.x - _endPoint2.x) * (_endPoint2.y - _anyPoint.y)) - ((_endPoint2.x - _anyPoint.x) * (_endPoint1.y - _anyPoint.y)));
    const float denominator = sqrt(pow((_endPoint1.x - _endPoint2.x), 2) + pow((_endPoint1.y - _anyPoint.y), 2));
    return numerator / denominator;
}
