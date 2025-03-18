#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import random

class ProbabilityCirclePublisher(Node):
    def __init__(self):
        super().__init__('probability_circle_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'circles', 10)
        self.timer = self.create_timer(1.0, self.publish_circles)
        self.get_logger().info("Probability Circle Publisher Node has been started")

    def publish_circles(self):
        marker_array = MarkerArray()

        # Example: Generate random probabilities for a set of circles
        num_circles = 10
        for i in range(num_circles):
            probability = random.uniform(0.0, 1.0)  # Random probability between 0 and 1

            # Create a circle marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "circles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set the circle's position (random in this example)
            marker.pose.position.x = random.uniform(-5.0, 5.0)
            marker.pose.position.y = random.uniform(-5.0, 5.0)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0  # Neutral orientation

            # Set the scale (size of the circle)
            marker.scale.x = 1.0  # Diameter of the circle
            marker.scale.y = 1.0
            marker.scale.z = 0.1  # Flat circle in Z-axis

            # Set the color based on the probability
            color_intensity = float(probability)  # Ensure probability is a float
            marker.color = ColorRGBA(r=float(0), 
                                     g=float(128/255), 
                                     b=float(1), 
                                     a=color_intensity)  # Transparency

            # Add the marker to the array
            marker_array.markers.append(marker)

            # Create a text marker to display the probability
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "text"
            text_marker.id = num_circles + i  # Ensure unique ID
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Set the text position to match the circle's position
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = marker.pose.position.z + 0.5  # Slightly above the circle
            text_marker.pose.orientation.w = 1.0  # Neutral orientation

            # Set the scale (size of the text)
            text_marker.scale.z = 0.3  # Height of the text

            # Set the color of the text
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White text

            # Set the text to display the probability
            text_marker.text = f"{probability:.2f}"

            # Add the text marker to the array
            marker_array.markers.append(text_marker)

        # Publish the marker array
        self.publisher.publish(marker_array)
        self.get_logger().info(f"Published {num_circles} circles with probabilities")

def main(args=None):
    rclpy.init(args=args)
    node = ProbabilityCirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
