#! /usr/bin/env python

from visualization_msgs.msg import Marker
from sensor_msgs.msg    import JointState
import rclpy
from rclpy.node         import Node
from numpy import random
from hw6code.KinematicChain    import KinematicChain
import numpy as np
from hw5code.TransformHelpers  import *

class MinimalPublisher(Node):
    v = 0.
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Marker, 'marker1', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        print('try sub')

        self.subscription_1 = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_rcvd,
            10)
        self.subscription_1  # prevent unused variable warning

        self.chain = KinematicChain(Node('marker_chain'), 'world', 'link_39', self.jointnames())
        self.q = np.radians(np.array([0]*39).reshape((-1,1)))

        self.bike = Marker()
        self.bike.action = Marker.ADD
        # self.bike.type = Marker.MESH_RESOURCE
        # self.bike.mesh_resource = "package://final_v1/meshes/Rock.stl"
        self.bike.type = 2
        self.bike.color.a = 1. #"0.7 0.5 0.1 1"
        self.bike.color.r = .7
        self.bike.color.g = .5
        self.bike.color.b = .1

        self.bike.header.frame_id = "/world"
        self.bike.scale.x = 0.2
        self.bike.scale.y = 0.2
        self.bike.scale.z = 0.2
        self.bike.color.a = 1.

        
        x,y = self.random_startpt()
        self.bike.pose.position.x = x
        self.bike.pose.position.y = y
        self.bike.pose.position.z = 3.
        self.bike.pose.orientation.x = 0.
        self.bike.pose.orientation.y = 0.
        self.bike.pose.orientation.z = 0.
        self.bike.pose.orientation.w = 1.
    
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        j = []
        for ind in range(1,40):
            j.append('joint%i'%ind)
        return j

    def joint_rcvd(self,msg):
        self.q = msg.position

    def timer_callback(self):
        # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(self.bike)
        dt = 0.001
        self.v += 9.8*dt
        self.bike.pose.position.z -= self.v*dt
        if self.bike.pose.position.z<-3.:
            self.bike.pose.position.z=3.
            x,y = self.random_startpt()
            self.bike.pose.position.x = x
            self.bike.pose.position.y = y
            self.v = 0
    
    def random_startpt(self):
        self.chain.setjoints(self.q)
        T = self.chain.data.T
        i = random.randint(len(T)-1)
        Pi = p_from_T(T[i])
        Pf = p_from_T(T[i+1])
        ratio = random.rand()
        P = Pi*ratio+Pf*(1-ratio)
        # print('random link', i)
        return P[0][0],P[1][0]




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

