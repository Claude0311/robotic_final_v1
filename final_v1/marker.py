#! /usr/bin/env python

from visualization_msgs.msg import Marker
import rclpy
from rclpy.node         import Node
from numpy import random

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Marker, 'marker1', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.bike = Marker()
        self.bike.action = Marker.ADD
        self.bike.type = Marker.SPHERE
        self.bike.header.frame_id = "/world"
        self.bike.scale.x = 0.5
        self.bike.scale.y = 0.5
        self.bike.scale.z = 0.5
        self.bike.color.a = 1.
        self.bike.pose.position.x = 0.
        self.bike.pose.position.y = 0.
        self.bike.pose.position.z = 3.
        self.bike.pose.orientation.x = 0.
        self.bike.pose.orientation.y = 0.
        self.bike.pose.orientation.z = 0.
        self.bike.pose.orientation.w = 1.
        self.v = 0

    def timer_callback(self):
        # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(self.bike)
        dt = 0.001
        self.v += 9.8*dt
        self.bike.pose.position.z -= self.v*dt
        if self.bike.pose.position.z<-3.:
            self.bike.pose.position.z=3.
            x,y = random.normal(0,3,size=[2])
            self.bike.pose.position.x = x
            self.bike.pose.position.y = y
            self.v = 0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

