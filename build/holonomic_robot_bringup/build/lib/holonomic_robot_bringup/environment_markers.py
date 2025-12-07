#!/usr/bin/env python3
"""
Environment markers publisher for visualizing the room walls in RVIZ2.
Publishes MarkerArray representing a 3m x 2m rectangular room.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class EnvironmentMarkers(Node):
    def __init__(self):
        super().__init__('environment_markers')
        
        # Define 8-point polygon (octagonal room)
        self.polygon_points = [
            (-1.5, -1.0),   # Point 0: Bottom left
            (-1.5, 1.0),    # Point 1: Top left
            (-0.5, 1.5),    # Point 2: Top left corner
            (0.5, 1.5),     # Point 3: Top right corner
            (1.5, 1.0),     # Point 4: Top right
            (1.5, -1.0),    # Point 5: Bottom right
            (0.5, -1.5),    # Point 6: Bottom right corner
            (-0.5, -1.5),   # Point 7: Bottom left corner
        ]
        
        self.wall_thickness = 0.1  # thickness of wall markers
        
        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'environment_markers', 10)
        
        # Timer to publish markers (1 Hz is sufficient for static environment)
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Environment markers node started')
    
    def create_wall_marker(self, marker_id, x1, y1, x2, y2):
        """Create a line marker representing a wall."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'walls'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Scale (line width)
        marker.scale.x = self.wall_thickness
        
        # Color (gray walls)
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        
        # Wall endpoints
        p1 = Point()
        p1.x = x1
        p1.y = y1
        p1.z = 0.0
        
        p2 = Point()
        p2.x = x2
        p2.y = y2
        p2.z = 0.0
        
        marker.points = [p1, p2]
        
        return marker
    
    def publish_markers(self):
        """Publish markers for all polygon walls."""
        marker_array = MarkerArray()
        
        # Create walls connecting each polygon point
        num_points = len(self.polygon_points)
        for i in range(num_points):
            x1, y1 = self.polygon_points[i]
            x2, y2 = self.polygon_points[(i + 1) % num_points]  # Wrap around to first point
            
            marker_array.markers.append(
                self.create_wall_marker(i, x1, y1, x2, y2)
            )
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
