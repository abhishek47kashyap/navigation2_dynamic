"""
Example data-sets for 2D lidar datasets:
  ipb.uni-bonn.de/datasets (Cyrill Stachniss: Pre-2014 Robotics 2D-Laser Datasets)
  github.com/ZRazer/2D-laser-datasets (bunch of bag files)
  robotcar-dataset.robots.ox.ac.uk/documentation/#d-lidar
  asrl.utias.utoronto.ca/datasets/mrclam/#Download
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
from copy import deepcopy
import itertools
from sklearn.linear_model import LinearRegression
import random

from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker


def random_color():
    color_rgb = [random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)]
    return [i/255 for i in color_rgb]


COLOR_PALETTES = [random_color() for i in range(100)]


class Line:
    def __init__(self):
        self.slope: float = 0.0
        self.intercept: float = 0.0
        self.endpoints = [Point(), Point()]


class Circle:
    def __init__(self):
        self.radius: float
        self.center = Point()


class Group:
    def __init__(self):
        self.list_of_points = []  # points that belong to a group
        self.best_fit_line = Line()
        self.best_fit_circle = Circle()

    @property
    def num_points(self):
        return len(self.list_of_points)

    def calculate_best_fit_line(self):
        # create numpy 'vectors' of X and Y coordinates
        x_coordinates = np.array([pt.x for pt in self.list_of_points]).reshape((-1, 1))
        y_coordinates = np.array([pt.y for pt in self.list_of_points])

        # compute best fit line
        regressor = LinearRegression().fit(x_coordinates, y_coordinates)
        self.best_fit_line.slope = regressor.coef_
        self.best_fit_line.intercept = regressor.intercept_

        # to get endpoints, calculate closest and farthest points from origin and then project them on the best fit line
        min_range, max_range = float('inf'), 0.0
        for pt in self.list_of_points:
            r = distance_from_origin(pt)
            if r < min_range:
                min_range = r
                self.best_fit_line.endpoints[0].x = pt.x
            if r > max_range:
                max_range = r
                self.best_fit_line.endpoints[1].x = pt.x

        # (reshaping and [0] indexing because of expected data type/structure requirement)
        self.best_fit_line.endpoints[0].y = regressor.predict(np.array(self.best_fit_line.endpoints[0].x).reshape(1, -1))[0]
        self.best_fit_line.endpoints[1].y = regressor.predict(np.array(self.best_fit_line.endpoints[1].x).reshape(1, -1))[0]

    def calculate_best_fit_circle(self):
        """
        Construct equilateral triangle with best fit line as base, and then get its circum-circle
        """

        # get X and Y centroids
        self.best_fit_circle.center.x = 0.0
        self.best_fit_circle.center.y = 0.0
        for point in self.list_of_points:
            self.best_fit_circle.center.x += point.x
            self.best_fit_circle.center.y += point.y
        self.best_fit_circle.center.x /= self.num_points
        self.best_fit_circle.center.y /= self.num_points

        # get radius
        self.best_fit_circle.radius = 0
        for point in self.list_of_points:
            d = distance_between_points(point, self.best_fit_circle.center)
            if d > self.best_fit_circle.radius:
                self.best_fit_circle.radius = d

        # # get midpoint of the best fit line and its perpendicular slope
        # midpoint_x = (self.best_fit_line.endpoints[0].x + self.best_fit_line.endpoints[1].x) / 2
        # midpoint_y = (self.best_fit_line.endpoints[0].y + self.best_fit_line.endpoints[1].y) / 2
        # slope_of_perpendicular = -1 / self.best_fit_line.slope
        #
        # # out of the two candidate vertices, pick the one that points away from the origin
        # candidate_third_vertices = [Point(), Point()]
        # candidate_third_vertices[0].x = (midpoint_x + slope_of_perpendicular * np.sqrt(3))[0]
        # candidate_third_vertices[0].y = (midpoint_y + slope_of_perpendicular * np.sqrt(3))[0]
        # candidate_third_vertices[1].x = (midpoint_x - slope_of_perpendicular * np.sqrt(3))[0]
        # candidate_third_vertices[1].y = (midpoint_y - slope_of_perpendicular * np.sqrt(3))[0]
        # third_vertex = candidate_third_vertices[0] \
        #     if distance_from_origin(candidate_third_vertices[0]) > distance_from_origin(candidate_third_vertices[1]) \
        #     else candidate_third_vertices[1]
        #
        # # calculate radius and center of the best fit circle
        # self.best_fit_circle.radius = distance_between_points(self.best_fit_line.endpoints[0],
        #                                                       self.best_fit_line.endpoints[1]) / np.sqrt(3)
        # self.best_fit_circle.center.x = (self.best_fit_line.endpoints[0].x +
        #                                  self.best_fit_line.endpoints[1].x +
        #                                  third_vertex.x) / 3
        # self.best_fit_circle.center.y = (self.best_fit_line.endpoints[0].y +
        #                                  self.best_fit_line.endpoints[1].y +
        #                                  third_vertex.y) / 3


class Detection2dLidar(Node):
    def __init__(self):
        super().__init__('detection_2d_lidar')

        # declare parameters on Parameter Server and get their values
        self.declare_parameters(namespace='',
                                parameters=[('p_max_group_distance', 0.1),
                                            ('p_distance_proportion', 0.00628),
                                            ('p_max_split_distance', 0.2),
                                            ('p_min_group_points', 5),
                                            ('p_max_merge_separation', 0.2),
                                            ('p_max_merge_spread', 0.2),
                                            ('p_max_circle_radius', 0.6),
                                            ('p_radius_enlargement', 0.25)])
        self.p_max_group_distance = self.get_parameter('p_max_group_distance').value
        self.p_distance_proportion = self.get_parameter('p_distance_proportion').value
        self.p_max_split_distance = self.get_parameter('p_max_split_distance').value
        self.p_min_group_points = self.get_parameter('p_min_group_points').value
        self.p_max_merge_separation = self.get_parameter('p_max_merge_separation').value
        self.p_max_merge_spread = self.get_parameter('p_max_merge_spread').value
        self.p_max_circle_radius = self.get_parameter('p_max_circle_radius').value
        self.p_radius_enlargement = self.get_parameter('p_radius_enlargement').value

        self.header = None

        # Initialize some empty lists
        self.points = []  # cartesian points (XY coordinates)
        self.groups = []  # list of Group() objects
        self.obstacles_lines = []  # list of obstacles represented as lines
        self.obstacles_circles = []  # list of obstacles represented as circles

        # for keeping track of which markers to delete
        self._point_IDs = []
        self._obstacle_IDs = []

        # Subscribe to /scan topic on which input lidar data is available
        self.create_subscription(LaserScan, 'scan', self.callback_lidar_data,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publish to detector ('detection' topic has subscriber in kf_hungarian_tracker)
        self.detection_pub = self.create_publisher(ObstacleArray, 'detection', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'marker', 10)

    def callback_lidar_data(self, laser_scan):
        self.header = laser_scan.header
        theta = np.arange(laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment)
        r = np.array(laser_scan.ranges)

        # making sure len(theta) == len(r)  [is this check even required?]
        if len(theta) != len(r):
            if len(theta) < len(r):
                r = r[:len(theta)]  # truncate r
                self.get_logger().warn(
                    "Polar coordinates theta and r have unequal lengths (%d and %d resp.), r has been truncated" % (
                        len(theta), len(r)))
            else:
                theta = theta[:len(r)]  # truncate theta
                self.get_logger().warn(
                    "Polar coordinates theta and r have unequal lengths (%d and %d resp.), theta has been truncated" % (
                        len(theta), len(r)))

        # convert points from polar coordinates to cartesian coordinates
        indices_of_valid_r = [i for i in range(len(r)) if laser_scan.range_min <= r[i] < laser_scan.range_max]
        r, theta = [r[i] for i in indices_of_valid_r], [theta[i] for i in indices_of_valid_r]
        self.points = [Point(x=r[i] * np.cos(theta[i]), y=r[i] * np.sin(theta[i]), z=0.0)  # 2D lidar doesn't have Z
                       for i in range(len(r))]

        self.reset_state()
        self.detect_obstacles()

    def reset_state(self):
        """
        Reset member variables for every new message
        """
        self.groups = []  # list of Group() objects
        self.obstacles_lines = []  # list of obstacles represented as lines
        self.obstacles_circles = []  # list of obstacles represented as circles

    def detect_obstacles(self):
        """
        Pipeline as followed in the Tysik paper
        i)  Grouping
        ii) Splitting
        iii) Segmentation
        iv) Segment merge
        v) Circle extraction
        vi) Circle merging
        """

        self.grouping()
        self.splitting()
        self.segmentation()   # calculates best fit line
        self.segment_merging()
        self.circle_extraction()   # calculates best fit circle and categorizes into line vs. circle obstacles

    def grouping(self):
        """
        Section 2.1 of the paper
        Clusters the points based on 2 user defined parameters: p_max_group_distance, p_distance_proportion

        Returns:
            list of all groups
        """
        points_remaining = deepcopy(self.points)

        while len(points_remaining) > 0:  # as points will be assigned to a group, they will be removed
            # start off a group with a point
            a_group = Group()
            point_from = points_remaining.pop(0)  # can be any point from points_remaining
            a_group.list_of_points.append(point_from)

            if len(points_remaining) == 0:  # in case that was the last point
                self.groups.append(a_group)
                break

            for p in points_remaining:
                r = distance_from_origin(p)
                d_from_prev_point = distance_between_points(p, point_from)

                if d_from_prev_point < self.p_max_group_distance + r * self.p_distance_proportion:  # add to group
                    a_group.list_of_points.append(p)
                    point_from = p

            # at this point, all points satisfying the above condition have been added to group
            self.groups.append(a_group)
            points_remaining = [point for point in points_remaining if point not in a_group.list_of_points]

        self.get_logger().info("grouping() completed, number of groups = %d" % len(self.groups))

        # visualizing in RViz
        # self.marker_pub.publish(self.visualize_groups())

    def splitting(self):
        """
        Section 2.2 of the paper

        Splits a group using the Ramer-Douglas-Peucker algorithm, also known as the iterative end point fit algorithm.

        The actual algorithm is recursive, but for our purpose may be just one "sweep" is enough
        """

        groups_after_splitting = []

        for grp in self.groups:
            # Skip group if it has too few points
            if grp.num_points <= self.p_min_group_points:
                continue

            # Find longest line
            pairwise_points = list(itertools.combinations(grp.list_of_points, 2))
            longest_length = 0
            end_point_1, end_point_2 = Point(), Point()
            for pair in pairwise_points:
                point1, point2 = pair[0], pair[1]
                d = distance_between_points(point1, point2)
                if d > longest_length:
                    longest_length = d
                    end_point_1.x, end_point_1.y = point1.x, point1.y
                    end_point_2.x, end_point_2.y = point2.x, point2.y

            # Find farthest point from longest line (this point would be somewhere "between" the end_points)
            d_max = 0
            farthest_point = Point()
            for pt in grp.list_of_points:
                if pt in [end_point_1, end_point_2]:
                    continue  # end points of the longest line should not be used

                d_point_from_line = distance_point_from_line(end_point_1, end_point_2, pt)
                if d_point_from_line > d_max:
                    d_max = d_point_from_line
                    farthest_point.x = pt.x
                    farthest_point.y = pt.y

            # Perform check whether group should be split
            r = distance_from_origin(farthest_point)
            if d_max > self.p_max_split_distance + r * self.p_distance_proportion:
                # points "between" end_point_1 and farthest_point go into grp1, rest into grp2
                grp1, grp2 = Group(), Group()
                grp1.list_of_points.append(end_point_1)
                grp2.list_of_points.append(end_point_2)
                # put farthest_point in the correct group as well
                if distance_between_points(end_point_1, farthest_point) < distance_between_points(end_point_2,
                                                                                                  farthest_point):
                    grp1.list_of_points.append(farthest_point)
                else:
                    grp2.list_of_points.append(farthest_point)
                # now go through the remaining points
                for pt in grp.list_of_points:
                    if pt in [end_point_1, end_point_2, farthest_point]:  # these have already been dealt with
                        continue

                    dist_point_from_grp1_line = distance_point_from_line(end_point_1, farthest_point, pt)
                    dist_point_from_grp2_line = distance_point_from_line(farthest_point, end_point_2, pt)

                    if dist_point_from_grp1_line < dist_point_from_grp2_line:
                        grp1.list_of_points.append(pt)
                    else:
                        grp2.list_of_points.append(pt)

                # all points have now been split into 2 groups
                groups_after_splitting.append(grp1)
                groups_after_splitting.append(grp2)

            else:  # group does not have to be split, append the un-split group
                groups_after_splitting.append(grp)

        self.groups = groups_after_splitting
        self.get_logger().info("splitting() completed, number of groups = %d" % len(self.groups))

        # visualizing in RViz
        # self.marker_pub.publish(self.visualize_groups())

    def segmentation(self):
        """
        Section 2.3 of the paper

        Regress a line to each group
        """
        for grp in self.groups:
            grp.calculate_best_fit_line()

        # self.get_logger().info("segmentation() completed")

    def segment_merging(self):
        """
        Section 2.4 of the paper

        Merge segments by checking 2 conditions
        """
        merged_groups = []
        pairwise_groups = list(itertools.combinations(self.groups, 2))

        for every_pair in pairwise_groups:
            segments_close_enough = False
            group1, group2 = every_pair[0], every_pair[1]

            # get the 4 endpoints and check distances between them
            all_4_endpoints = [group1.best_fit_line.endpoints[0], group1.best_fit_line.endpoints[1],
                               group2.best_fit_line.endpoints[0], group2.best_fit_line.endpoints[1]]
            pairwise_points = list(itertools.combinations(all_4_endpoints, 2))
            for pair_of_pts in pairwise_points:
                pt1, pt2 = pair_of_pts[0], pair_of_pts[1]
                if distance_between_points(pt1, pt2) < self.p_max_merge_separation:
                    segments_close_enough = True
                    break

            if segments_close_enough:
                merged_grp = Group()
                merged_grp.list_of_points = group1.list_of_points
                merged_grp.list_of_points.extend(group2.list_of_points)

                # Spread test, to check if the lines are collinear
                merged_grp.calculate_best_fit_line()
                d_max = 0
                for pt in all_4_endpoints:
                    d = distance_point_from_line(merged_grp.best_fit_line.endpoints[0],
                                                 merged_grp.best_fit_line.endpoints[1],
                                                 pt)
                    if d > d_max:
                        d_max = d

                if d_max < self.p_max_merge_spread:  # confirmed that the 2 groups should be replace by one
                    merged_groups.append(merged_grp)

            else:
                merged_groups.append(group1)
                merged_groups.append(group2)

        # it's possible duplicates have crept into merged_groups, so they need to be removed
        self.groups = list(set(merged_groups))
        self.get_logger().info("segment_merging() completed, number of groups = %d" % len(self.groups))

        # visualizing in RViz
        # self.marker_pub.publish(self.visualize_groups())

    def circle_extraction(self):
        """
        Section 2.5 of the paper

        Only circles less than a certain radius is considered to be a circular obstacle.
        That's because some best fit lines can be very long and the circle for those obstacles would be too huge!
        """
        # compute best fit circles for all the groups
        for grp in self.groups:
            grp.calculate_best_fit_circle()

            # check whether circle (after enlargement) is to be added to list of circle obstacles or line obstacles
            if grp.best_fit_circle.radius + self.p_radius_enlargement <= self.p_max_circle_radius:
                self.obstacles_circles.append(grp)
            else:
                self.obstacles_lines.append(grp)

        self.get_logger().info("%d groups separated into %d lines and %d circles" %
                               (len(self.groups), len(self.obstacles_lines), len(self.obstacles_circles)))

        # visualizing in RViz
        # self.marker_pub.publish(self.visualize_obstacles())

    def visualize_groups(self):
        """
        Visualize all groups in self.groups, where every group gets its own color.
        :return: a MarkerArray() to be published to RViz
        """
        marker_list = []
        dummy_id = 0
        for grp in self.groups:
            color_for_this_group = COLOR_PALETTES[self.groups.index(grp)]
            for pt in grp.list_of_points:
                marker = Marker()
                marker.id = dummy_id
                marker.header = self.header
                marker.type = Marker.SPHERE
                marker.action = 0  # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
                marker.color.a = 0.5
                marker.color.r = color_for_this_group[0]
                marker.color.g = color_for_this_group[1]
                marker.color.b = color_for_this_group[2]
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = pt.x
                marker.pose.position.y = pt.y
                marker_list.append(marker)
                if dummy_id not in self._point_IDs:
                    self._point_IDs.append(dummy_id)
                dummy_id += 1

        # delete markers from previous message that are not present in current message
        for id_to_del in self._point_IDs[dummy_id:]:
            marker = Marker()
            marker.id = id_to_del
            marker.action = Marker.DELETE
            marker_list.append(marker)
        self._point_IDs = self._point_IDs[:dummy_id]

        # publish
        marker_array = MarkerArray()
        marker_array.markers = marker_list
        return marker_array

    def visualize_obstacles(self):
        """
        Visualize line and circle obstacles
        :return: a MarkerArray() to be published to RViz
        """
        marker_list = []
        dummy_id = 0

        # Obstacles represented by lines
        for grp in self.obstacles_lines:
            marker = Marker()
            marker.id = dummy_id
            marker.header = self.header
            marker.type = Marker.LINE_STRIP
            marker.action = 0  # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.scale.x = 0.1
            marker.points = grp.best_fit_line.endpoints
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker_list.append(marker)
            if dummy_id not in self._obstacle_IDs:
                self._obstacle_IDs.append(dummy_id)
            dummy_id += 1

        # Obstacles represented by circles
        for grp in self.obstacles_circles:
            marker = Marker()
            marker.id = dummy_id
            marker.header = self.header
            marker.type = Marker.CYLINDER
            marker.action = 0  # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.scale.x, marker.scale.y = float(grp.best_fit_circle.radius * 2), float(grp.best_fit_circle.radius * 2)  # set different xy values for ellipse
            marker.scale.z = 0.1
            marker.pose.position.x = grp.best_fit_circle.center.x
            marker.pose.position.y = grp.best_fit_circle.center.y
            marker.pose.position.z = marker.scale.z / 2
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker_list.append(marker)
            if dummy_id not in self._obstacle_IDs:
                self._obstacle_IDs.append(dummy_id)
            dummy_id += 1

        # delete detections from previous message that are not present in current message
        for id_to_del in self._obstacle_IDs[dummy_id:]:
            marker = Marker()
            marker.id = id_to_del
            marker.action = Marker.DELETE
            marker_list.append(marker)
        self._obstacle_IDs = self._obstacle_IDs[:dummy_id]

        # publish
        marker_array = MarkerArray()
        marker_array.markers = marker_list
        return marker_array


def distance_from_origin(p: Point):
    return np.sqrt(p.x ** 2 + p.y ** 2)


def distance_between_points(p1: Point, p2: Point):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def distance_point_from_line(end_point_a: Point, end_point_b: Point, any_point: Point):
    numerator = np.abs((end_point_a.x - end_point_b.x)*(end_point_b.y - any_point.y) -
                       (end_point_b.x - any_point.x)*(end_point_a.y - end_point_b.y))
    denominator = np.sqrt((end_point_a.x - end_point_b.x) ** 2 + (end_point_a.y - end_point_b.y) ** 2)
    return numerator / denominator


def main(args=None):
    rclpy.init(args=args)
    detection_2d_lidar = Detection2dLidar()
    rclpy.spin(detection_2d_lidar)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
